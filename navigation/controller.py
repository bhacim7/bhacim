import math
import numpy as np
from config import cfg
import time

class RobotController:
    """
    Robotun hareket mantığını (Motor PWM hesaplamaları) yöneten sınıf.
    Rota takibi (Pure Pursuit) ve Lidar tabanlı refleks sürüşü burada yapılır.
    """

    def __init__(self):
        self.last_heading_error = 0.0

    def calculate_pure_pursuit(self, robot_pose, path, current_speed_pwm=None):
        """
        [SENIOR UPDATE]
        - Adaptive Lookahead: Virajlarda kısa bakış.
        - Target Smoothing: Hedef titremesini önleme.
        """
        if not path or len(path) < 2:
            return 1500, 1500, None

        rx, ry, ryaw = robot_pose

        # --- 1. ADAPTIVE LOOKAHEAD (KIVRAK DÖNÜŞ) ---
        # Standart bakış: 1.5m, Viraj varsa: 0.8m
        current_lookahead = 1.5
        
        # Yolun ilerisine bakıp viraj var mı diye kontrol et
        check_idx = min(len(path)-1, 5) 
        ref_p = path[check_idx]
        desired_angle = math.atan2(ref_p[1] - ry, ref_p[0] - rx)
        
        # Açı farkını hesapla (0-360 sorununu çözerek)
        angle_diff = desired_angle - ryaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi # -PI, +PI normalize
        angle_diff_deg = abs(math.degrees(angle_diff))

        if angle_diff_deg > 20:
            current_lookahead = 0.8 # VİRAJ VAR! Yakına bak.
        
        # --- 2. HEDEF NOKTAYI BUL ---
        target_point = path[-1]
        for p in path:
            dist = math.sqrt((p[0] - rx) ** 2 + (p[1] - ry) ** 2)
            if dist > current_lookahead:
                target_point = p
                break

        # --- 3. TARGET SMOOTHING (HEDEF YUMUŞATMA) ---
        # Hedef nokta aniden zıplamasın (Lidar gürültüsü vs. için)
        if not hasattr(self, 'prev_target'):
            self.prev_target = None
        
        if self.prev_target is not None:
            # Low-Pass Filter: %70 Eski, %30 Yeni
            sm_x = (self.prev_target[0] * 0.7) + (target_point[0] * 0.3)
            sm_y = (self.prev_target[1] * 0.7) + (target_point[1] * 0.3)
            target_point = (sm_x, sm_y)
        
        self.prev_target = target_point
        # ---------------------------------------------

        # 4. Açıyı Hesapla ve PWM Üret
        target_angle = math.atan2(target_point[1] - ry, target_point[0] - rx)
        alpha = target_angle - ryaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # PWM Hesaplama
        base = current_speed_pwm if current_speed_pwm else cfg.BASE_PWM
        kp = 250.0 
        
        # Hız kesme (Virajda yavaşla)
        turn_slowdown = 0
        if abs(math.degrees(alpha)) > 20:
            turn_slowdown = 50

        correction = int(alpha * kp)
        
        left_pwm = int(base - turn_slowdown - correction)
        right_pwm = int(base - turn_slowdown + correction)

        return left_pwm, right_pwm, target_point

    def analyze_lidar_sectors(self, lidar_scan):
        """
        Lidar verisini Sol, Merkez, Sağ olarak analiz eder.
        Returns: (center_blocked, left_dist, center_dist, right_dist)
        """
        if not lidar_scan:
            return False, float('inf'), float('inf'), float('inf')

        min_left = float('inf')
        min_center = float('inf')
        min_right = float('inf')
        center_blocked = False

        for _, angle, dist_mm in lidar_scan:
            dist_m = dist_mm / 1000.0

            # Gürültü ve Menzil Filtresi
            if dist_m < 0.1 or dist_m > cfg.LIDAR_MAX_DIST:
                continue

            # Açıyı normalize et (-180, 180)
            if angle > 180: angle -= 360

            # Sektörlere Ayır
            # Merkez: -15 ile +15 derece
            if -15 <= angle <= 15:
                if dist_m < min_center: min_center = dist_m
                if dist_m < cfg.LIDAR_EMERGENCY_STOP_M:
                    center_blocked = True

            # Sağ: +15 ile +60 (ZED/Lidar montajına göre değişebilir)
            elif 15 < angle <= 60:
                if dist_m < min_right: min_right = dist_m

            # Sol: -60 ile -15
            elif -60 <= angle < -15:
                if dist_m < min_left: min_left = dist_m

        return center_blocked, min_left, min_center, min_right

    def calculate_lidar_corridor_drive(self, left_dist, right_dist):
        """
        İki duvar arasında ortalayarak gitmek için PWM hesaplar (Task 5).
        """
        # Mesafe sonsuzsa (duvar yoksa) sanal bir duvar varsay
        l_val = left_dist if not math.isinf(left_dist) else 2.0
        r_val = right_dist if not math.isinf(right_dist) else 2.0

        # Hata: (Sağ - Sol). Pozitifse sağda çok boşluk var -> Sağa dön
        error = r_val - l_val

        # P Kontrolcü
        correction = error * cfg.LIDAR_KORIDOR_KP
        correction = np.clip(correction, -100, 100)

        # Yavaş ve Dikkatli Hız
        SAFE_PWM = 60  # Base üzerine eklenir

        left_pwm = int(cfg.BASE_PWM + SAFE_PWM + correction)
        right_pwm = int(cfg.BASE_PWM + SAFE_PWM - correction)

        return left_pwm, right_pwm

    def get_escape_maneuver(self, left_dist, right_dist):
        """
        Acil durumda (Ön dolu) kaçış için PWM değerleri döndürür.
        Boş olan tarafa doğru olduğu yerde (tank) döner.
        """
        turn_power = 200  # Dönüş sertliği

        # Hangi taraf daha boş?
        if left_dist > right_dist:
            # Sola Dön (Sol Geri, Sağ İleri)
            return (1500 - turn_power), (1500 + turn_power)
        else:
            # Sağa Dön (Sol İleri, Sağ Geri)
            return (1500 + turn_power), (1500 - turn_power)
    
    def get_failsafe_command(self, lost_time_start):
        """
        A* yol bulamadığında devreye giren 'B Planı'.
        Tüm görevler (Task 1, 2, 3) bu fonksiyonu ortak kullanır.
        """
        import time # Garanti olsun diye buraya da ekledim
        if lost_time_start is None:
            return 1500, 1500
            
        elapsed = time.time() - lost_time_start

        # FAZ 1: NEFES AL (0 - 2.0 Saniye)
        # Hemen panikleme. Belki lidar anlık hata yaptı, harita düzelir.
        if elapsed < 2.0:
            return 1500, 1500 

        # FAZ 2: MİNİK KAÇIŞ (MICRO ESCAPE) (2.0 - 5.0 Saniye)
        # 2 saniyedir yol yok. Sıkıştık. Olduğun yerde yavaşça dönerek kör noktayı aç.
        elif elapsed < 5.0:
            # Sola yavaş dönüş (Kendi ekseninde)
            return 1400, 1600 

        # FAZ 3: GERİ ÇEKİL (5.0+ Saniye)
        # Dönmek işe yaramadı, önümüz tamamen duvar. Geri git.
        else:
            return 1400, 1400 # Yavaşça geri git