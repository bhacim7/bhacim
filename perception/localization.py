import math
import time
from config import cfg


class KalmanFilter:
    """
    Manyetik pusula (Heading) verisi için Kalman Filtresi.
    Gürültülü sensör verisini pürüzsüzleştirir.
    """

    def __init__(self, process_variance=1e-3, measurement_variance=1e-1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.x_estimate = None  # Cos bileşeni
        self.y_estimate = None  # Sin bileşeni
        self.error_covariance = 1.0

    def update(self, angle_deg):
        # Açıyı birim çembere çevir (0-360 sorunu yaşamamak için)
        angle_rad = math.radians(angle_deg)
        x_meas = math.cos(angle_rad)
        y_meas = math.sin(angle_rad)

        if self.x_estimate is None:
            self.x_estimate = x_meas
            self.y_estimate = y_meas
            return angle_deg

        # Tahmin (Prediction) - Basit modelde değişim yok varsayıyoruz
        pred_x = self.x_estimate
        pred_y = self.y_estimate
        pred_cov = self.error_covariance + self.process_variance

        # Güncelleme (Update)
        kalman_gain = pred_cov / (pred_cov + self.measurement_variance)

        self.x_estimate = pred_x + kalman_gain * (x_meas - pred_x)
        self.y_estimate = pred_y + kalman_gain * (y_meas - pred_y)
        self.error_covariance = (1 - kalman_gain) * pred_cov

        # Normalize et
        norm = math.sqrt(self.x_estimate ** 2 + self.y_estimate ** 2)
        self.x_estimate /= norm
        self.y_estimate /= norm

        # Tekrar açıya çevir
        filtered_rad = math.atan2(self.y_estimate, self.x_estimate)
        filtered_deg = math.degrees(filtered_rad)
        return filtered_deg % 360


class LocalizationManager:
    def __init__(self):
        # Filtre
        self.heading_filter = KalmanFilter()

        # Durum Değişkenleri (Lokal)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # Radyan
        self.heading = 0.0  # Derece (Kuzey=0, Saat Yönü)

        # Durum Değişkenleri (Global)
        self.lat = 0.0
        self.lon = 0.0
        self.gps_fix = 0  # 0: No Fix, 3: 3D Fix

    def update(self, zed_pose, mav_heading, gps_data):
        """
        Sensör verilerini birleştirir.

        Args:
            zed_pose: ZED sl.Pose nesnesi (veya None)
            mav_heading: Pixhawk pusula verisi (derece)
            gps_data: (lat, lon, fix_type) tuple
        """
        # 1. Lokal Konum (ZED Odometry)
        if zed_pose:
            # Orijinal koddaki eksen düzeltmesi:
            # Robot ileri gittiğinde (ZED Z ekseni) -> Haritada X artmalı
            # Robot sağa gittiğinde (ZED X ekseni) -> Haritada Y azalmalı
            t = zed_pose.get_translation().get()
            self.x = t[1]  # ZED Forward (Z) -> Map X
            self.y = -t[0]  # ZED Right (X)   -> Map -Y (Left)

            # ZED'den gelen Yaw (Radyan) - Yedek olarak tutuyoruz
            o = zed_pose.get_orientation().get()  # [ox, oy, oz, ow]
            # Quaternion -> Yaw hesaplama
            siny_cosp = 2 * (o[3] * o[2] + o[0] * o[1])
            cosy_cosp = 1 - 2 * (o[1] * o[1] + o[2] * o[2])
            self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. Heading (Pusula) Filtreleme
        if mav_heading is not None:
            # Kalman filtresinden geçir
            filtered = self.heading_filter.update(mav_heading)
            # Manyetik sapma düzeltmesi (+6 derece, orijinal koddan)
            self.heading = (filtered + 6) % 360

        # 3. GPS Güncelleme
        if gps_data and gps_data[0] is not None:
            self.lat = gps_data[0]
            self.lon = gps_data[1]
            self.gps_fix = gps_data[2]

    def get_pose(self):
        """Lokal haritalama için (x, y, yaw_rad) döndürür."""
        return self.x, self.y, self.yaw

    def get_gps(self):
        """Global navigasyon için (lat, lon) döndürür."""
        return self.lat, self.lon