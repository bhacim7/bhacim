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
        self.yaw_offset = None # ZED ve Pusula arasındaki fark

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
            raw_yaw = math.atan2(siny_cosp, cosy_cosp)

            # Map-Aligned ZED Yaw (Forward=0)
            # ZED Frame: Yaw=0 at Right, Yaw=90 at Forward.
            # Map Frame: Yaw=0 at Forward.
            # So Map = ZED - 90 deg (pi/2)
            self.yaw = raw_yaw - (math.pi / 2)

        # 2. Heading (Pusula) Filtreleme
        if mav_heading is not None:
            # Kalman filtresinden geçir
            filtered = self.heading_filter.update(mav_heading)
            # Manyetik sapma düzeltmesi (+6 derece, orijinal koddan)
            self.heading = (filtered + 6) % 360

            # [SENIOR UPDATE] SENSOR FUSION (ZED + COMPASS)
            # ZED Yaw (Kısa vadeli stabil) ile Pusula (Uzun vadeli doğru) birleştirilir.
            if zed_pose is not None:
                # Compass (CW from North) -> Map ENU (CCW from East)
                # Map 0 = East, 90 = North.
                # Compass 0 = North -> Map 90.
                # Compass 90 = East -> Map 0.
                compass_enu_deg = (90 - self.heading) % 360
                compass_enu_rad = math.radians(compass_enu_deg)

                # İlk hizalama (Offset belirle)
                if self.yaw_offset is None:
                    self.yaw_offset = compass_enu_rad - self.yaw

                # Complementary Filter: %98 ZED, %2 Pusula
                # Amaç: ZED'in driftini pusula ile yavaşça düzeltmek.
                aligned_zed_yaw = self.yaw + self.yaw_offset

                diff = compass_enu_rad - aligned_zed_yaw
                diff = (diff + math.pi) % (2 * math.pi) - math.pi # Normalize (-PI, PI)

                # Offseti güncelle (Öğrenme Modu)
                # Hatayı zamanla offset'e yedirerek kalıcı düzeltme sağla.
                self.yaw_offset += 0.02 * diff

        # 3. Apply Offset and Coordinate Rotation (Local -> Global ENU)
        if self.yaw_offset is not None and zed_pose is not None:
            # Apply offset to Yaw
            self.yaw = self.yaw + self.yaw_offset

            # Rotate Position (ZED Frame -> Global ENU)
            # self.x, self.y are currently in ZED-Map aligned frame (X=Forward).
            # yaw_offset is rotation from ZED-Map to Global ENU.

            raw_x = self.x
            raw_y = self.y

            cos_off = math.cos(self.yaw_offset)
            sin_off = math.sin(self.yaw_offset)

            self.x = raw_x * cos_off - raw_y * sin_off
            self.y = raw_x * sin_off + raw_y * cos_off

        # 4. GPS Güncelleme
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
