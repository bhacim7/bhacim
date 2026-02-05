import math
import numpy as np
from config import cfg, wp
from navigation.controller import RobotController
from utils.math_tools import haversine


class Task5Docking:
    """
    GÖREV 5: Automated Docking (Otomatik Park)

    1. Marina girişine yaklaş.
    2. Lidar ve Kamera ile boşluk ara.
    3. En düşük numaralı (Soldaki) yeşil şamandıraya park et.
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
        self.target_dock = None # Görsel hedef

    def enter(self):
        print("[TASK 5] Başlatıldı: Docking")
        self.current_state = self.STATE_APPROACH
        self.finished = False
        self.dock_timer = 0
        self.target_dock = None

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
        # B) BOŞLUK TESPİTİ (KAMERA + LIDAR)
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_SCAN:
            vision_objs = sensors.get('vision_objs', [])

            # 1. VISUAL SEARCH (Görsel Arama)
            greens = [o for o in vision_objs if o['label'] == 'GREEN' and o['dist'] < 8.0]

            if greens:
                # En soldaki (Lowest Number) şamandırayı seç
                # Robota göre açısı en büyük (pozitif) olan en soldadır.
                best_green = None
                max_angle = -999.0

                for g in greens:
                    # Açı farkı
                    angle_to_obj = math.atan2(g['y'] - ry, g['x'] - rx) - ryaw
                    # Normalize (-pi, pi)
                    angle_to_obj = (angle_to_obj + math.pi) % (2 * math.pi) - math.pi

                    if angle_to_obj > max_angle:
                        max_angle = angle_to_obj
                        best_green = g

                if best_green:
                    print(f"[TASK 5] Yeşil Bulundu (Açı: {math.degrees(max_angle):.1f}) -> HEDEF SEÇİLDİ")
                    self.target_dock = best_green
                    self.current_state = self.STATE_DOCKING
                    self.dock_timer = 0

                    # Çıkış yönünü belirle
                    if max_angle > 0: self.dock_side = "LEFT"
                    else: self.dock_side = "RIGHT"

                    return 1500, 1500

            # 2. FALLBACK TO LIDAR (Eğer kamera bulamazsa)
            # Lidar verisinden sağ/sol ortalamalarını çıkar
            right_dists = []
            left_dists = []

            for _, angle, dist_mm in lidar_scan:
                dist_m = dist_mm / 1000.0
                if dist_m > cfg.LIDAR_MAX_DIST or dist_m < 0.1: continue

                if angle > 180: angle -= 360

                if 70 < angle < 110:
                    right_dists.append(dist_m)
                elif -110 < angle < -70:
                    left_dists.append(dist_m)

            avg_right = np.mean(right_dists) if right_dists else 0.0
            avg_left = np.mean(left_dists) if left_dists else 0.0

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
                # Boşluk bulamazsan yavaşça ilerle
                _, l_min, _, r_min = self.controller.analyze_lidar_sectors(lidar_scan)
                left, right = self.controller.calculate_lidar_corridor_drive(l_min, r_min)
                return left, right

        # -----------------------------------------------------------
        # C) İÇERİ GİRİŞ MANEVRASI
        # -----------------------------------------------------------
        elif self.current_state == self.STATE_DOCKING:
            self.dock_timer += 1

            # GÖRSEL HEDEF VARSA ONA GİT
            if self.target_dock:
                 tx, ty = self.target_dock['x'], self.target_dock['y']
                 dist = math.sqrt((tx - rx)**2 + (ty - ry)**2)

                 # Çok yaklaştıysak veya süre dolduysa bitir
                 if dist < 1.0 or self.dock_timer > 150: # 7.5 sn (20Hz)
                     print("[TASK 5] Görsel Park Tamamlandı.")
                     self.current_state = self.STATE_EXITING
                     self.dock_timer = 0
                     return 1500, 1500

                 # Hedefe yönel (Pure Pursuit)
                 path = [(tx, ty)]
                 left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
                 return left, right

            # YOKSA KÖR MANEVRA YAP (Lidar Fallback)
            turn_pwm_sol = 1650
            turn_pwm_sag = 1350

            if self.dock_side == "LEFT":
                turn_pwm_sol = 1350
                turn_pwm_sag = 1650

            if self.dock_timer < 30:
                return turn_pwm_sol, turn_pwm_sag
            elif self.dock_timer < 90:
                print("[TASK 5] İçeri Giriliyor...")
                return 1600, 1600
            else:
                print("[TASK 5] Park Tamamlandı. Çıkışa geçiliyor.")
                self.current_state = self.STATE_EXITING
                self.dock_timer = 0
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
            elif self.dock_timer < 80:
                print("[TASK 5] Çıkış Yönüne Dönülüyor...")
                if self.dock_side == "RIGHT":
                    return 1350, 1650  # Sola Dön
                else:
                    return 1650, 1350  # Sağa Dön

            # Manevra Bitti -> Başlangıca Git
            else:
                if self.entry_point:
                    dist = haversine(rlat, rlon, self.entry_point[0], self.entry_point[1])
                    if dist < 3.0:
                        print("[TASK 5] Güvenli Bölgeye Dönüldü. GÖREV TAMAMLANDI.")
                        self.finished = True
                        return 1500, 1500

                lidar_info = self.controller.analyze_lidar_sectors(lidar_scan)
                left, right = self.controller.calculate_lidar_corridor_drive(lidar_info[1], lidar_info[3])
                return left, right

        return 1500, 1500

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task