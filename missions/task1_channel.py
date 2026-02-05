from config import cfg, wp
from navigation.path_planner import PathPlanner
from navigation.controller import RobotController
from utils.math_tools import haversine


class Task1Channel:
    """
    GÖREV 1: Kanal Geçişi (Evacuation Route)
    1. Aşama: Kanal girişine GPS ile yaklaş.
    2. Aşama: Kanal içinden (Şamandıraların arasından) çıkış noktasına git.
    """

    def __init__(self):
        # Durumlar
        self.STATE_APPROACH = "APPROACH_ENTRY"  # Girişe git
        self.STATE_NAVIGATE = "NAVIGATE_CHANNEL"  # Kanalı geç
        self.path_lost_time = None
        
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.next_task = "TASK2_APPROACH"

        # Navigasyon Araçları
        self.planner = PathPlanner()
        self.controller = RobotController()

        # Hedefler (waypoints.py'dan)
        self.entry_pt = (wp.T1_GATE_ENTER_LAT, wp.T1_GATE_ENTER_LON)
        self.exit_pt = (wp.T1_GATE_EXIT_LAT, wp.T1_GATE_EXIT_LON)

        # Başlangıç konumu (Mesafe hesabı için)
        self.start_pose = None

    def enter(self):
        """Görev başladığında bir kez çalışır."""
        print("[TASK 1] Başlatıldı: Kanal Geçişi")
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.start_pose = None

    def execute(self, sensors, robot_state):
        """
        Her döngüde çalışır.
        Returns: (left_pwm, right_pwm)
        """
        # Robot verilerini çöz
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        # Navigasyon Haritası (Main loop'tan gelmeli)
        nav_map = sensors.get('nav_map')
        map_info = sensors.get('map_info')  # {'center':..., 'res':..., 'size':...}

        if rlat == 0.0 or rlon == 0.0:
            print("[TASK 1] GPS bekleniyor...")
            return 1500, 1500

        # Başlangıç konumunu kaydet
        if self.start_pose is None:
            self.start_pose = (rlat, rlon)

        # -----------------------------------------------------------
        # DURUM 1: GİRİŞ KAPISINA YAKLAŞMA
        # -----------------------------------------------------------
        if self.current_state == self.STATE_APPROACH:
            dist_to_entry = haversine(rlat, rlon, self.entry_pt[0], self.entry_pt[1])

            # 5 metre kala navigasyon moduna geç
            if dist_to_entry < 5.0:
                print("[TASK 1] Giriş kapısına varıldı -> Kanal Modu")
                self.current_state = self.STATE_NAVIGATE

            # Girişe gitmek için geçici bir hedef nokta oluşturuyoruz
            # Burada A* yerine doğrudan GPS heading kullanılabilir ama
            # biz sistemin tutarlılığı için A* hedefi olarak giriş kapısını veriyoruz.
            # (Not: Dünya koordinatı -> Map koordinatı dönüşümü planner içinde yapılıyor)
            # Ancak basitlik için burada Planner'a hedef olarak GİRİŞ noktasını veremeyiz
            # çünkü planner metre (X,Y) çalışır, GPS (Lat/Lon) değil.

            # Bu yüzden APPROACH aşamasında basit GPS sürüşü (Controller üzerinden değil)
            # veya mapping sisteminin GPS->Metre dönüşümünü kullanmalıyız.
            # Şimdilik basitlik adına: Eğer harita merkezindeysek (0,0), hedefi ona göre ayarla.
            # AMA DAHA İYİSİ: Planlayıcıyı sadece engeller varken kullanmak.

            # BASİT YAKLAŞIM: GPS ile git, engel varsa LIDAR refleksi devreye girsin.
            # controller.calculate_pure_pursuit yerine basit pusula sürüşü.
            # (Şimdilik kod karışmasın diye burayı geçiyorum, main loop'ta GPS heading var)

            # --- GEÇİCİ: Doğrudan Kanal Moduna Atla (Harita zaten sürekli çalışıyor) ---
            # Eğer GPS ile hedef belirlemek istiyorsan localization sınıfına
            # lat/lon -> x/y (metre) dönüşümü eklememiz gerekir.
            # Şimdilik harita merkezine (0,0) göre hareket edeceğiz.
            pass

            # -----------------------------------------------------------
        # DURUM 2: KANAL İÇİNDE İLERLEME (ÇIKIŞA GİT)
        # -----------------------------------------------------------

        # Hedef: Çıkış Noktası (Metre cinsinden X,Y lazım)
        # GPS farkından metre hesaplayarak hedefi buluyoruz (Flat Earth)
        # 1 derece lat ~= 111132m, 1 derece lon ~= 111319m * cos(lat)

        # Harita Merkezi (Robot ilk başladığı yer) referans alındığı için:
        # Bu hesaplama biraz karmaşık olabilir, bu yüzden Task 1'de genellikle
        # "Vision ile kapı ortala" mantığı kullanılır.
        # Ancak biz A* kullanacağız. Haritada "ileri" (X+) yönünde gitmeye çalışalım.

        # HEDEF: Robotun önünde (veya harita koordinatlarında) 10 metre ilerisi
        # Engel varsa A* etrafından dolanır.
        target_x = rx + 10.0  # Haritada sürekli ileri gitmeye çalış
        target_y = 0.0  # Merkeze sadık kalmaya çalış (Kanal ortası)

        # Eğer haritada engel varsa A* rota çizer
        path = self.planner.plan_path((rx, ry), (target_x, target_y), nav_map, map_info)

        # Motor Güçlerini Hesapla
        if path:
            # A) A* YOL BULDU -> Sorun yok, yola devam et.
            self.path_lost_time = None  # Tehlike geçti, sayacı sıfırla.
            
            # Controller ile rotayı takip et
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
            
            # NOT: Eğer Task 1 içindeysen ve 'Bitiş Kontrolü' kodların varsa
            # 'return left, right' yapmadan önce o kontrolleri burada yapabilirsin
            # veya return'ü en sona bırakabilirsin. Ama en temizi burdan dönmektir.
            
        else:
            # B) A* YOL BULAMADI -> Merkezi Kurtarma Modunu Çağır
            import time
            if self.path_lost_time is None:
                self.path_lost_time = time.time()
                print(f"[{self.__class__.__name__}] YOL KAYIP! Failsafe Modu...")

            # Controller'daki ortak zekayı (Bekle -> Dön -> Geri Git) çağır
            left, right = self.controller.get_failsafe_command(self.path_lost_time)

        # Eğer kodunun devamında 'Bitiş Kontrolü' (dist_to_exit vs.) varsa
        # ve return'ü aşağıda yapıyorsan, yukarıdaki if/else içinde return yapma,
        # değişkenleri (left, right) güncelle, akış aşağı devam etsin.
        # AMA genelde return edip çıkmak daha güvenlidir.
        
        # Burada Bitiş Kontrolü (Exit Point) kodların varsa aynen kalsın:
        # Bitiş Kontrolü
        dist_to_exit = haversine(rlat, rlon, self.exit_pt[0], self.exit_pt[1])
        travelled = haversine(rlat, rlon, self.start_pose[0], self.start_pose[1])

        if dist_to_exit < 3.0 and travelled > 15.0:
            print("[TASK 1] Çıkış kapısına ulaşıldı.")
            self.finished = True
            return 1500, 1500
        
        return left, right
        

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task