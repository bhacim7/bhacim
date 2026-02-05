import math
import cv2
import json
import time
from config import cfg, wp
from navigation.path_planner import PathPlanner
from navigation.controller import RobotController
from utils.math_tools import haversine


class Task2Avoidance:
    """
    GÖREV 2: Debris Clearance (Engel Sahası)

    Özellikler:
    - Görsel Hafıza (Landmark Memory): Görülen şamandıraları kaydeder.
    - Ekmek Kırıntısı (Breadcrumbs): Gidilen rotayı kaydeder ve geri dönüşte kullanır.
    - Tur Atma (Circling): Yeşil şamandıra etrafında tur atar.
    """

    def __init__(self):
        # Durumlar
        self.STATE_APPROACH = "APPROACH_ENTRY"
        self.STATE_TRAVERSE = "TRAVERSE_ZONE"
        self.STATE_SEARCH_GREEN = "SEARCH_GREEN"
        self.STATE_CIRCLE = "CIRCLE_GREEN"
        self.STATE_RETURN = "RETURN_HOME"
        self.path_lost_time = None


        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.next_task = "TASK3_APPROACH"

        # Araçlar
        self.planner = PathPlanner()
        self.controller = RobotController()

        # Hafıza
        self.landmarks = []  # [{'id': 1, 'label': 'RED', 'x': 0, 'y': 0, 'conf': 1}, ...]
        self.path_history = []  # [(lat, lon), (lat, lon), ...]
        self.landmark_id_counter = 0

        # Circling Değişkenleri
        self.circle_phase = 0
        self.target_buoy = None  # Tur atılacak şamandıra

    def enter(self):
        print("[TASK 2] Başlatıldı: Engel Sahası & Hafıza Modu")
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.path_history = []
        self.landmarks = []

    def execute(self, sensors, robot_state):
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        nav_map = sensors.get('nav_map')  # Lidar haritası (0=Engel, 255=Yol)
        map_info = sensors.get('map_info')
        vision_objs = sensors.get('vision_objs', [])  # YOLO çıktıları

        # 1. HAFIZA GÜNCELLEME (SLAM Benzeri Mantık)
        self._update_landmark_memory(vision_objs, rx, ry)

        # 2. HAFIZADAKİ ENGELLERİ HARİTAYA EKLE
        # Lidar'ın göremediği ama Kameranın gördüğü şamandıraları haritaya çiziyoruz.
        # Bu sayede A* algoritması onlara çarpmaz.
        if nav_map is not None:
            # Haritaya çizim yapmak için map_info'daki (center, res) lazım
            # Ancak mapping sınıfında bir helper olsa daha iyiydi.
            # Şimdilik basitçe mapping sınıfının logic'ini çağıracağız
            # veya mapping sınıfı instance'ını sensors içinde almalıyız.
            pass  # (Mapping sınıfı main loop'ta zaten bunları işleyebilir, burada sanal engel ekleme opsiyonel)

        # -----------------------------------------------------------
        # DURUM MAKİNESİ
        # -----------------------------------------------------------

        # A) GİRİŞE GİT
        if self.current_state == self.STATE_APPROACH:
            target = (wp.T2_ZONE_ENTRY_LAT, wp.T2_ZONE_ENTRY_LON)
            dist = haversine(rlat, rlon, target[0], target[1])

            if dist < 3.0:
                print("[TASK 2] Sahaya Girildi -> Kayıt Başlıyor")
                self.current_state = self.STATE_TRAVERSE
                self.path_history.append((rlat, rlon))

            # (Basit GPS sürüşü veya Lidar Koridor burada kullanılabilir)
            # Şimdilik boş dönüş yapıyoruz, main loop GPS ile sürebilir.
            return 1500, 1500

        # B) SAHAYI GEÇ (EKMEK KIRINTISI MODU)
        elif self.current_state == self.STATE_TRAVERSE:
            # Kayıt Al (Her 3 metrede bir)
            last_pt = self.path_history[-1]
            if haversine(rlat, rlon, last_pt[0], last_pt[1]) > 3.0:
                self.path_history.append((rlat, rlon))
                # print(f"[PATH] İz Bırakıldı: {len(self.path_history)}")

            # Hedef: Sahanın Sonu
            target_gps = (wp.T2_ZONE_END_LAT, wp.T2_ZONE_END_LON)

            # Sona yaklaştık mı?
            if haversine(rlat, rlon, target_gps[0], target_gps[1]) < 4.0:
                print("[TASK 2] Saha Sonu -> Yeşil Şamandıra Aranıyor")
                self.current_state = self.STATE_SEARCH_GREEN

            # A* Rota Hesabı (Harita merkezinden ileriye doğru)
            # Hedefi robotun 10 metre ilerisine koy (Sanal Hedef)
            target_x = rx + 10.0
            target_y = 0.0  # Merkez hattı koru

            # Haritada işaretli (Vision Memory) engellerden kaçarak yol bul
            # Not: self.landmarks listesindeki engelleri nav_map üzerine
            # siyah daire olarak çizmemiz lazım. (Main loop'ta mapping.py bunu yapacak)

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

        # C) YEŞİL ŞAMANDIRA ARA
        elif self.current_state == self.STATE_SEARCH_GREEN:
            # Hafızadaki en güvenilir Yeşil şamandırayı bul
            best_green = None
            min_dist = 999

            for lm in self.landmarks:
                if lm['label'] == 'GREEN' and lm['conf'] > 3:  # En az 3 kere görülmüş olsun
                    d = math.sqrt((lm['x'] - rx) ** 2 + (lm['y'] - ry) ** 2)
                    if d < min_dist:
                        min_dist = d
                        best_green = lm

            if best_green and min_dist < 5.0:
                print(f"[TASK 2] Yeşil Bulundu (ID: {best_green['id']}) -> Tur Atılıyor")
                self.target_buoy = best_green
                self.current_state = self.STATE_CIRCLE
                self.circle_phase = 0

            # Bulana kadar yavaşça ilerle
            return 1550, 1550

        # D) TUR AT (KARE YÖRÜNGE)
        elif self.current_state == self.STATE_CIRCLE:
            if not self.target_buoy:
                self.current_state = self.STATE_RETURN
                return 1500, 1500

            bx, by = self.target_buoy['x'], self.target_buoy['y']
            R = 2.0  # Yarıçap (Metre)

            # Kare Yörünge Noktaları (CW: Left -> Front -> Right -> Behind)
            offsets = [(0, R), (R, 0), (0, -R), (-R, 0)]

            if self.circle_phase >= 4:
                print("[TASK 2] Tur Tamamlandı -> Eve Dönüş")
                self.current_state = self.STATE_RETURN
                return 1500, 1500

            # Hedef Nokta
            ox, oy = offsets[self.circle_phase]
            tx, ty = bx + ox, by + oy

            # Noktaya git (A* kullanmadan direkt git çünkü şamandıra etrafındayız)
            # Buraya basit bir "noktaya git" P kontrolcüsü lazım.
            # Controller'daki pure pursuit mantığını tek nokta için kullanabiliriz.
            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)

            # Noktaya ulaştık mı?
            dist = math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2)
            if dist < 0.8:
                self.circle_phase += 1
                print(f"[TASK 2] Faz {self.circle_phase} tamam.")

            return left, right

        # E) EVE DÖNÜŞ (RETROGRADE)
        elif self.current_state == self.STATE_RETURN:
            if not self.path_history:
                print("[TASK 2] Geri Dönüş Bitti. Görev Tamamlandı.")
                self._generate_report()
                self.finished = True
                return 1500, 1500

            # Listenin sonundaki (en son geçtiğimiz) noktayı hedef al
            target_lat, target_lon = self.path_history[-1]
            dist = haversine(rlat, rlon, target_lat, target_lon)

            if dist < 2.5:
                self.path_history.pop()  # Noktaya vardık, listeden sil
                # print(f"[RETURN] Nokta geçildi. Kalan: {len(self.path_history)}")

            # O noktaya gitmek için GPS veya Pusula kullanabiliriz.
            # Burada basitçe "O noktaya dön ve git" mantığı kurabiliriz.
            # Ancak A* kullanmak daha güvenli olurdu. Şimdilik basit tutalım.
            # (Burada controller.py içine basit GPS sürüşü eklenmeli veya main loop halletmeli)

            # Basit Yöntem: Pusula açısı hesapla ve dön
            # (Bu kısım için main loop'taki GPS mantığına güveniyoruz veya buraya ekliyoruz)

            return 1550, 1550  # Geçici hız (Yönlendirmeyi controller yapmalı)

        return 1500, 1500

    def _update_landmark_memory(self, vision_objs, rx, ry):
        """Kameradan gelen verilerle hafızayı günceller."""
        MATCH_THRESH = 1.5  # Metre

        for obj in vision_objs:
            # Sadece belirli renkleri kaydet
            if obj['label'] not in ['RED', 'GREEN', 'YELLOW', 'BLACK']: continue

            ox, oy = obj['x'], obj['y']
            found = False

            for lm in self.landmarks:
                dist = math.sqrt((lm['x'] - ox) ** 2 + (lm['y'] - oy) ** 2)
                if dist < MATCH_THRESH:
                    # Güncelle (Ortalamasını al - Stabilizasyon)
                    lm['x'] = (lm['x'] * 0.7) + (ox * 0.3)
                    lm['y'] = (lm['y'] * 0.7) + (oy * 0.3)
                    lm['conf'] += 1
                    found = True
                    break

            if not found:
                self.landmark_id_counter += 1
                self.landmarks.append({
                    'id': self.landmark_id_counter,
                    'label': obj['label'],
                    'x': ox,
                    'y': oy,
                    'conf': 1
                })

    def _generate_report(self):
        """Tespit edilen nesneleri raporlar."""
        report = {
            "task": "Task 2 Debris Clearance",
            "timestamp": time.time(),
            "hazards": [],
            "survivors": []
        }

        for lm in self.landmarks:
            if lm['conf'] < 3: continue # Gürültüyü ele

            entry = {
                "id": lm['id'],
                "x": round(lm['x'], 2),
                "y": round(lm['y'], 2),
                "label": lm['label']
            }

            if lm['label'] in ['RED', 'BLACK']:
                report["hazards"].append(entry)
            elif lm['label'] == 'GREEN':
                report["survivors"].append(entry)

        try:
            with open("task2_report.json", "w") as f:
                json.dump(report, f, indent=4)
            print("[TASK 2] Rapor kaydedildi: task2_report.json")
        except Exception as e:
            print(f"[TASK 2 HATA] Rapor kaydedilemedi: {e}")

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task