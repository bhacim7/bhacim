import math
import numpy as np
from config import cfg, wp
from navigation.controller import RobotController
from utils.math_tools import haversine


class Task5Docking:
    """
    GÖREV 5: Automated Docking (Otomatik Park)

    1. Marina girişine yaklaş.
    2. Lidar ile sağ/sol boşlukları tara.
    3. Boş olan tarafa kör manevra (Timer ile) yap.
    4. Geri çık ve görevi bitir.
    """

    def __init__(self):
        # Durumlar
        self.STATE_APPROACH = "APPROACH_DOCK"
        self.STATE_SCAN = "SCAN_FOR_GAP"
        self.STATE_DOCKING = "MANEUVER_IN"
        self.STATE_EXITING = "MANEUVER_OUT"
        self.STATE_DONE = "DOCKING_COMPLETE"

        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.next_task = "MISSION_COMPLETE"  # Bu son görev

        self.controller = RobotController()

        # Değişkenler
        self.dock_side = "RIGHT"  # Varsayılan: Sağ taraf
        self.dock_timer = 0  # Manevra zamanlayıcısı
        self.entry_point = None  # Çıkışta geri dönülecek nokta

    def enter(self):
        print("[TASK 5] Başlatıldı: Docking")
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.dock_timer = 0

    def execute(self, sensors, robot_state):
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        lidar_scan = sensors.get('lidar', [])

        # -----------------------------------------------------------
        # A) MARİNA GİRİŞİNE GİT
        # -----------------------------------------------------------
        if self.current_state == self.STATE_APPROACH:
            target = (wp.T5_DOCK_APPROACH_LAT, wp.T5_DOCK_APPROACH_LON)
            dist = haversine(rlat, rlon, target[0], target[1])

            if dist < 4.0:
                print("[TASK 5] Girişe Varıldı -> Boşluk Aranıyor")
                self.current_state = self.STATE_SCAN
                self.entry_point = (rlat, rlon)

            # (Main loop GPS sürüşü yapacak)
            return 1500, 1500

        # -----------------------------------------------------------
        # B) BOŞLUK TESPİTİ (LIDAR ANALİZİ)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_SCAN:
            # Lidar verisinden sağ/sol ortalamalarını çıkar
            # Controller sınıfında bu analizi yapan bir metodumuz yoktu,
            # o yüzden burada manuel yapıyoruz.

            right_dists = []
            left_dists = []

            for _, angle, dist_mm in lidar_scan:
                dist_m = dist_mm / 1000.0
                if dist_m > cfg.LIDAR_MAX_DIST or dist_m < 0.1: continue

                # Açıyı normalize et
                if angle > 180: angle -= 360

                # Sağ Sektör (70 ile 110 derece arası)
                if 70 < angle < 110:
                    right_dists.append(dist_m)
                # Sol Sektör (-110 ile -70 derece arası)
                elif -110 < angle < -70:
                    left_dists.append(dist_m)

            avg_right = np.mean(right_dists) if right_dists else 0.0
            avg_left = np.mean(left_dists) if left_dists else 0.0

            # Karar Ver (Eşik: 2 metre üstü boşluktur)
            GAP_THRESH = 2.0

            if avg_right > GAP_THRESH:
                print(f"[TASK 5] SAĞDA BOŞLUK ({avg_right:.1f}m) -> PARK (SAĞ)")
                self.dock_side = "RIGHT"
                self.current_state = self.STATE_DOCKING
                self.dock_timer = 0

            elif avg_left > GAP_THRESH:
                print(f"[TASK 5] SOLDA BOŞLUK ({avg_left:.1f}m) -> PARK (SOL)")
                self.dock_side = "LEFT"
                self.current_state = self.STATE_DOCKING
                self.dock_timer = 0

            else:
                # Boşluk bulamazsan yavaşça ilerle (Lidar Koridor)
                # (Duvarlara sürtmeden ilerle ki boşluğu görebil)
                # Burada controller'dan yardım alabiliriz
                _, l_min, _, r_min = self.controller.analyze_lidar_sectors(lidar_scan)
                left, right = self.controller.calculate_lidar_corridor_drive(l_min, r_min)
                return left, right

        # -----------------------------------------------------------
        # C) İÇERİ GİRİŞ MANEVRASI (TIMER İLE KÖR SÜRÜŞ)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_DOCKING:
            self.dock_timer += 1

            # Yöne Göre PWM Ayarları
            # Varsayılan RIGHT: Sola yavaş, Sağa hızlı (Sola dönmemesi için) -> Aslında "Sağa Dön" = Sol Motor İleri, Sağ Motor Geri/Yavaş

            # Sağa Dönüş: Sol Motor > Sağ Motor
            turn_pwm_sol = 1650
            turn_pwm_sag = 1350

            if self.dock_side == "LEFT":  # Sola Dönüş: Sağ Motor > Sol Motor
                turn_pwm_sol = 1350
                turn_pwm_sag = 1650

            # Zaman Çizelgesi (Döngü sayısı, FPS'e bağlı, genelde 20Hz)
            # 0 - 1.5 sn: Dön
            if self.dock_timer < 30:
                # Dönüş Yap
                return turn_pwm_sol, turn_pwm_sag

            # 1.5 - 4.5 sn: Düz Gir
            elif self.dock_timer < 90:
                print("[TASK 5] İçeri Giriliyor...")
                return 1600, 1600

            # Park Bitti -> Dur ve Çıkışa Geç
            else:
                print("[TASK 5] Park Tamamlandı. Çıkışa geçiliyor.")
                self.current_state = self.STATE_EXITING
                self.dock_timer = 0  # Timer'ı sıfırla
                return 1500, 1500

        # -----------------------------------------------------------
        # D) ÇIKIŞ MANEVRASI (GERİ GERİ)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_EXITING:
            self.dock_timer += 1

            # 0 - 2.5 sn: Geri Çık
            if self.dock_timer < 50:
                print("[TASK 5] Geri Çıkılıyor...")
                return 1400, 1400

            # 2.5 - 4.0 sn: Burnunu Çıkışa Çevir
            # Geri çıktık, şimdi burnumuzu geldiğimiz yöne (Başlangıç noktasına) çevirmeliyiz.
            # Sağ parka girdiysek -> Çıkış solumuzda kalır -> Sola dön
            elif self.dock_timer < 80:
                print("[TASK 5] Çıkış Yönüne Dönülüyor...")

                # Eğer SAĞ parka girdiysek, geri çıkınca çıkış SOLUMUZDA kalır -> Sola Dön
                if self.dock_side == "RIGHT":
                    return 1350, 1650  # Sola Dön
                else:
                    return 1650, 1350  # Sağa Dön

            # Manevra Bitti -> Başlangıca Git
            else:
                # Başlangıç noktasına yakın mıyız?
                if self.entry_point:
                    dist = haversine(rlat, rlon, self.entry_point[0], self.entry_point[1])
                    if dist < 3.0:
                        print("[TASK 5] Güvenli Bölgeye Dönüldü. GÖREV TAMAMLANDI.")
                        self.finished = True
                        return 1500, 1500

                # Başlangıca gitmek için basit Lidar Koridor sürüşü yap (Geri geri değil, ileri)
                lidar_info = self.controller.analyze_lidar_sectors(lidar_scan)
                left, right = self.controller.calculate_lidar_corridor_drive(lidar_info[1], lidar_info[3])
                return left, right

        return 1500, 1500

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task