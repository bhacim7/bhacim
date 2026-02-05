import cv2
import numpy as np
import math
from config import cfg


class CostmapManager:
    """
    Robotun çevresini haritalandıran (SLAM benzeri) ve
    navigasyon için uygun hale getiren sınıf.
    """

    def __init__(self):
        # Harita Ayarları
        self.width_px = 800
        self.height_px = 800
        self.resolution = 0.10  # 1 piksel = 10 cm

        # Harita Merkezi (Robot başladığında burası (0,0) kabul edilir)
        self.center_m = (0, 0)

        # Harita Matrisi (0-255 uint8)
        # 127: Bilinmiyor (Gri)
        # 255: Boş (Beyaz)
        # 0:   Engel (Siyah)
        self.costmap = np.full((self.height_px, self.width_px), 127, dtype=np.uint8)

        # Navigasyon Haritası (Şişirilmiş engeller)
        self.nav_map = None

    def init_map(self, start_x=0, start_y=0):
        """Harita merkezini ayarlar ve matrisi sıfırlar."""
        self.center_m = (start_x, start_y)
        self.costmap.fill(127)
        print("[MAP] Harita başlatıldı.")

    def world_to_pixel(self, world_x, world_y):
        """Metre cinsinden koordinatı harita pikseline çevirir."""
        cw = self.width_px // 2
        ch = self.height_px // 2

        dx = world_x - self.center_m[0]
        dy = world_y - self.center_m[1]

        # Harita koordinat sistemi (Y aşağı artar, Dünya'da Y yukarı artar -> -dy)
        px = int(cw + (dx / self.resolution))
        py = int(ch - (dy / self.resolution))

        if 0 <= px < self.width_px and 0 <= py < self.height_px:
            return (px, py)
        return None

    def update(self, lidar_scan, robot_pose, robot_roll=0, robot_pitch=0):
        """
        Lidar verisiyle haritayı günceller.
        robot_pose: (x, y, yaw_radians)
        lidar_scan: [(quality, angle_deg, dist_mm), ...]
        """
        MAX_TILT = 7.0  # Derece (Hem Roll hem Pitch için limit)
        
        # Eğer robot çok yalpalıyorsa haritayı dondur
        if abs(robot_roll) > MAX_TILT or abs(robot_pitch) > MAX_TILT:
            # print(f"[MAP] Eğim Yüksek! Roll:{robot_roll:.1f} Pitch:{robot_pitch:.1f} -> Harita Donduruldu.")
            return

        rx, ry, ryaw = robot_pose
        p_robot = self.world_to_pixel(rx, ry)
        if not p_robot: return

        # Performans için maskeler oluştur
        empty_mask = np.zeros_like(self.costmap)
        occupied_mask = np.zeros_like(self.costmap)

        FREE_GAIN = 2
        OCCUPIED_GAIN = 80

        # Seyreltme (Her 3 noktadan 1'ini işle - Optimizasyon)
        for i, point in enumerate(lidar_scan):
            angle_deg = point[1]
            dist_mm = point[2]
            dist_m = dist_mm / 1000.0

            # Filtreler
            if dist_m < 0.1 or dist_m > 20.0: continue
            if dist_m <= 5.0 and (i % 3 != 0): continue  # Yakındakileri seyrelt

            # Arkadaki veriyi yoksay (Kamera/Lidar montajına göre değişebilir)
            if 135 < angle_deg < 225: continue

            # Engel Koordinatı Hesapla
            angle_rad = math.radians(angle_deg)
            global_angle = ryaw - angle_rad  # Lidar açısı ters ise +/- değişebilir

            obs_x = rx + (dist_m * math.cos(global_angle))
            obs_y = ry + (dist_m * math.sin(global_angle))

            p_obs = self.world_to_pixel(obs_x, obs_y)

            if p_obs:
                # 1. Işın (Ray): Robot ile Engel arası BOŞTUR
                cv2.line(empty_mask, p_robot, p_obs, FREE_GAIN, 1)
                # 2. Nokta (Hit): Engelin olduğu yer DOLUDUR
                cv2.circle(occupied_mask, p_obs, 2, OCCUPIED_GAIN, -1)

        # Haritayı güncelle (Olasılıksal Toplama)
        # Boş yerleri beyazlaştır (255'e yaklaş)
        self.costmap = cv2.add(self.costmap, empty_mask)
        # Dolu yerleri siyahlaştır (0'a yaklaş)
        self.costmap = cv2.subtract(self.costmap, occupied_mask)

    def get_navigation_map(self, safe_margin_m=None):
        """
        A* algoritması için engelleri şişirilmiş (Inflated) haritayı döndürür.
        Dönüş: 0=Engel, 255=Yol
        """
        if safe_margin_m is None:
            safe_margin_m = cfg.ROBOT_RADIUS_M + cfg.INFLATION_MARGIN_M

        # 1. Sadece kesin engelleri al (Threshold < 100 olanlar engeldir)
        # 255 (Beyaz) -> 0 (Siyah) dönüşümü yapıyoruz ki cv2.dilate çalışsın
        obstacles_mask = (self.costmap < 100).astype(np.uint8) * 255

        # 2. Şişirme (Dilation)
        kernel_size = int((safe_margin_m / self.resolution) * 2) + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated_obstacles = cv2.dilate(obstacles_mask, kernel, iterations=1)

        # 3. Sonuç Haritası (255=Yol, 0=Engel)
        nav_grid = np.full_like(self.costmap, 255)
        nav_grid[inflated_obstacles > 0] = 0

        self.nav_map = nav_grid
        return nav_grid

    

    def add_virtual_obstacle(self, world_x, world_y, radius_m=0.6):
        """
        [SENIOR UPDATE]
        Haritaya manuel/sanal engel ekler.
        Kamera bir şey gördüğünde Lidar görmese bile burayı doldurur.
        """
        # 1. Koordinatı piksele çevir (Senin yardımcı fonksiyonunu kullanıyoruz)
        px_coords = self.world_to_pixel(world_x, world_y)
        
        if px_coords:
            # 2. Yarıçapı piksele çevir (0.6m ideal güvenlik payıdır)
            radius_px = int(radius_m / self.resolution)
            
            # 3. Haritayı Siyaha Boya (0 = Engel)
            # -1 parametresi dairenin içini doldur demektir.
            cv2.circle(self.costmap, px_coords, radius_px, 0, -1)
            
            # [OPSİYONEL] Eğer o an nav_map oluşmuşsa onu da boya ki anlık tepki verelim
            if self.nav_map is not None:
                cv2.circle(self.nav_map, px_coords, radius_px, 0, -1)  