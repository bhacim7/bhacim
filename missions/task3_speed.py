import math
from config import cfg, wp
from navigation.path_planner import PathPlanner
from navigation.controller import RobotController
from utils.math_tools import haversine


class Task3Speed:
    """
    GÖREV 3: Speed Challenge (Sürat Kapısı ve Dönüş)

    Senaryo:
    1. Kapıdaki ışık/şamandıra rengini algıla.
    2. Renk KIRMIZI ise -> SAĞDAN git, Sarı şamandırayı SAĞINA alarak dön.
    3. Renk YEŞİL ise   -> SOLDAN git, Sarı şamandırayı SOLUNA alarak dön.
    4. Geldiğin rotadan geri dön.
    """

    def __init__(self):
        # Durumlar
        self.STATE_APPROACH = "APPROACH_GATE"
        self.STATE_DECIDE = "DECIDE_DIRECTION"
        self.STATE_SPRINT = "SPRINT_TO_YELLOW"
        self.STATE_CIRCLE = "CIRCLE_YELLOW"
        self.STATE_RETURN = "RETURN_HOME"
        self.path_lost_time = None


        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.next_task = "TASK5_APPROACH"

        # Araçlar
        self.planner = PathPlanner()
        self.controller = RobotController()

        # Değişkenler
        self.path_history = []
        self.turn_direction = "RIGHT"  # Varsayılan: Sağ (Kırmızı senaryosu)
        self.target_buoy = None  # Sarı şamandıra konumu
        self.circle_phase = 0

        # Hız Ayarı (Normalden daha hızlı)
        self.speed_boost = cfg.T3_SPEED_PWM

    def enter(self):
        print("[TASK 3] Başlatıldı: Speed Challenge")
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.path_history = []
        self.circle_phase = 0

    def execute(self, sensors, robot_state):
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        nav_map = sensors.get('nav_map')
        map_info = sensors.get('map_info')
        vision_objs = sensors.get('vision_objs', [])

        # -----------------------------------------------------------
        # A) KAPIYA YAKLAŞ (GPS)
        # -----------------------------------------------------------
        if self.current_state == self.STATE_APPROACH:
            target = (wp.T3_GATE_SEARCH_LAT, wp.T3_GATE_SEARCH_LON)
            dist = haversine(rlat, rlon, target[0], target[1])

            # 5 metre kala algılama moduna geç
            if dist < 5.0:
                print("[TASK 3] Kapıya yaklaşıldı -> Karar Modu")
                self.current_state = self.STATE_DECIDE

            # (Basit GPS sürüşü - Main loop halledebilir veya buraya eklenebilir)
            return 1500, 1500

        # -----------------------------------------------------------
        # B) YÖN KARARI VER (IŞIK/RENK ALGILAMA)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_DECIDE:
            # Görüntüdeki en yakın ve güvenilir objeye bak
            red_count = 0
            green_count = 0

            for obj in vision_objs:
                if obj['dist'] > 8.0: continue  # Çok uzaktakine bakma

                if obj['label'] == 'RED':
                    red_count += 1
                elif obj['label'] == 'GREEN':
                    green_count += 1

            # Karar Mekanizması
            if red_count > green_count:
                self.turn_direction = "RIGHT"
                print(f"[TASK 3] Kırmızı Algılandı -> YÖN: SAĞ")
            elif green_count > red_count:
                self.turn_direction = "LEFT"
                print(f"[TASK 3] Yeşil Algılandı -> YÖN: SOL")
            else:
                # Kimseyi görmezsen varsayılanı koru
                print(f"[TASK 3] Renk Seçilemedi -> Varsayılan: {self.turn_direction}")

            self.current_state = self.STATE_SPRINT
            return 1500, 1500

        # -----------------------------------------------------------
        # C) SARIYA HÜCUM (SPRINT)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_SPRINT:
            # Ekmek Kırıntısı Kaydı
            if not self.path_history:
                self.path_history.append((rlat, rlon))
            else:
                last_pt = self.path_history[-1]
                if haversine(rlat, rlon, last_pt[0], last_pt[1]) > 2.0:
                    self.path_history.append((rlat, rlon))

            # Sarı şamandırayı bul
            for obj in vision_objs:
                if obj['label'] == 'YELLOW':
                    self.target_buoy = obj  # {'x':..., 'y':...} (Global)
                    break

            # Eğer sarı şamandıra görüldüyse ona git, görülmediyse GPS noktasına
            target_x, target_y = rx + 10.0, 0.0  # Varsayılan ileri

            if self.target_buoy:
                dist_to_buoy = math.sqrt((self.target_buoy['x'] - rx) ** 2 + (self.target_buoy['y'] - ry) ** 2)
                if dist_to_buoy < 4.0:
                    print("[TASK 3] Sarıya Varıldı -> Tur Modu")
                    self.current_state = self.STATE_CIRCLE
                    return 1500, 1500

                target_x = self.target_buoy['x']
                target_y = self.target_buoy['y']

            else:
                # Vision yoksa GPS'e bak (Yedek)
                # GPS hedefi harita koordinatına çevrilemediği için
                # burada basitçe ileri gidiyoruz.
                pass

            # A* ve Hızlı Sürüş
            path = self.planner.plan_path((rx, ry), (target_x, target_y), nav_map, map_info)

            if path:
            # A) A* YOL BULDU -> Sorun yok, yola devam et.
                self.path_lost_time = None  # Tehlike geçti, sayacı sıfırla.
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
            # dist_to_exit = ...
            # if dist_to_exit < 1.0: ...
            
            return left, right

        # -----------------------------------------------------------
        # D) TUR AT (CIRCLE)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_CIRCLE:
            # Hedef şamandıra yoksa son konumu kullan
            bx, by = rx, ry
            if self.target_buoy:
                bx, by = self.target_buoy['x'], self.target_buoy['y']

            R = 1.5  # Yarıçap

            # Yöne göre offset listesi
            # RIGHT (Sağdan dön): Saat yönü tersi (CCW) -> Sağ, Arka, Sol, Ön
            # LEFT (Soldan dön): Saat yönü (CW) -> Sol, Arka, Sağ, Ön

            offsets = []
            if self.turn_direction == "RIGHT":
                offsets = [(0, -R), (R, 0), (0, R), (-R, 0)]  # CCW (ZED Y ekseni ters olabilir, dikkat)
                # Not: ZED koordinat sisteminde Y sağ/sol, X ileri ise:
                # X+, Y- (Sağ ön), X- Y- (Sağ arka)... bu biraz deneme yanılma ister.
                # Standart Kartezyen (X ileri, Y Sol):
                # CCW: (0, -R) [Sağ], (-R, 0) [Arka], (0, R) [Sol], (R, 0) [Ön]
            else:
                offsets = [(0, R), (-R, 0), (0, -R), (R, 0)]  # CW

            if self.circle_phase >= 4:
                print("[TASK 3] Tur Bitti -> Kaçış")
                self.current_state = self.STATE_RETURN
                return 1500, 1500

            ox, oy = offsets[self.circle_phase]
            tx, ty = bx + ox, by + oy

            # Noktaya git
            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path,
                                                                    current_speed_pwm=self.speed_boost)

            if math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2) < 1.0:
                self.circle_phase += 1
                # print(f"Faz {self.circle_phase} OK")

            return left, right

        # -----------------------------------------------------------
        # E) GERİ DÖN (RETROGRADE)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_RETURN:
            if not self.path_history:
                print("[TASK 3] Görev Tamamlandı.")
                self.finished = True
                return 1500, 1500

            target_lat, target_lon = self.path_history[-1]
            dist = haversine(rlat, rlon, target_lat, target_lon)

            if dist < 3.0:
                self.path_history.pop()

            # (Yönlendirme logic'i main'den veya basit pusula kontrolünden gelmeli)
            # Hızlı dönüş için boost kullan
            return 1500 + self.speed_boost, 1500 + self.speed_boost

        return 1500, 1500

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task