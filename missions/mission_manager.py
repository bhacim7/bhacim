from config import cfg


class MissionManager:
    """
    Tüm görevlerin yönetim merkezi (Orkestra Şefi).
    Sensör verilerini alır, aktif göreve iletir ve görev geçişlerini yönetir.
    """

    def __init__(self):
        self.tasks = {}  # Kayıtlı görevlerin listesi
        self.current_task = None  # Şu an çalışan görev nesnesi
        self.current_task_name = ""
        self.is_active = False  # Görev sistemi çalışıyor mu?

    def register_task(self, name, task_instance):
        """Sisteme yeni bir görev tanıtır."""
        self.tasks[name] = task_instance

    def start_mission(self, initial_task_name):
        """Görev serisini başlatır."""
        if initial_task_name in self.tasks:
            self.is_active = True
            self._switch_to_task(initial_task_name)
        else:
            print(f"[MANAGER HATA] Başlangıç görevi bulunamadı: {initial_task_name}")

    def stop_mission(self):
        """Her şeyi durdurur."""
        self.is_active = False
        self.current_task = None
        print("[MANAGER] Görevler durduruldu.")

    def update(self, sensors, robot_state):
        """
        Her döngüde çağrılır.
        Args:
            sensors: dict {'lidar', 'vision', 'gps', 'audio_cmd', ...}
            robot_state: LocalizationManager objesi (x, y, yaw, lat, lon)

        Returns:
            (left_pwm, right_pwm) : Motor komutları
        """
        # 1. ACİL DURUM KESMESİ (TASK 6 - SESLİ KOMUT)
        # -----------------------------------------------------------
        # Eğer Task 6 modülü bir komut yakaladıysa (3 veya 5),
        # robot ne yapıyorsa yapsın bırakır ve o göreve geçer.

        audio_cmd, freq_name = sensors.get('audio_cmd', (None, None))

        if audio_cmd is not None:
            target_task = None
            if audio_cmd == 3:
                target_task = "TASK3_APPROACH"
            elif audio_cmd == 5:
                target_task = "TASK5_APPROACH"

            # Eğer zaten o görevde değilsek değiştir
            if target_task and target_task != self.current_task_name:
                print(f"\n!!! [KESME] SES ALGILANDI ({freq_name}) -> {target_task} !!!\n")
                self._switch_to_task(target_task)
                return 1500, 1500  # Bir tur bekle (Reset için)

        # 2. AKTİF GÖREVİ YÜRÜT
        # -----------------------------------------------------------
        if not self.is_active or not self.current_task:
            return 1500, 1500  # Boşta (Idle)

        # Görevi çalıştır ve motor isteklerini al
        left_pwm, right_pwm = self.current_task.execute(sensors, robot_state)

        # 3. GÖREV BİTİŞ KONTROLÜ
        # -----------------------------------------------------------
        if self.current_task.is_finished():
            next_task_name = self.current_task.get_next_task()

            if next_task_name == "MISSION_COMPLETE":
                print("[MANAGER] TÜM GÖREVLER BAŞARIYLA TAMAMLANDI.")
                self.stop_mission()
                return 1500, 1500

            elif next_task_name:
                print(f"[MANAGER] Görev Tamamlandı -> Sıradaki: {next_task_name}")
                self._switch_to_task(next_task_name)

            else:
                # Sonraki görev tanımlı değilse dur
                self.stop_mission()

        return left_pwm, right_pwm

    def _switch_to_task(self, task_name):
        """Güvenli görev değişimi yapar."""
        if task_name not in self.tasks:
            print(f"[MANAGER HATA] Hedef görev yok: {task_name}")
            return

        # Eski görevden çıkış prosedürü (varsa)
        # if self.current_task: self.current_task.exit()

        self.current_task_name = task_name
        self.current_task = self.tasks[task_name]

        print(f"--- GÖREV BAŞLIYOR: {task_name} ---")

        # Yeni görevi başlat (Reset vb.)
        self.current_task.enter()