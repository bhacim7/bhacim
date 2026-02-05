import pyzed.sl as sl
import cv2
import numpy as np
import sys
from config import cfg


class TimestampHandler:
    """
    IMU ve Görüntü zaman damgalarını senkronize etmek için yardımcı sınıf.
    """

    def __init__(self):
        self.t_imu = sl.Timestamp()

    def is_new(self, sensor):
        if isinstance(sensor, sl.IMUData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        return False


class ZedCameraDriver:
    def __init__(self):

        #kullanılacak nesneleri oluştur
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.runtime_params = sl.RuntimeParameters(enable_fill_mode=True)

        # ZED Ayarlarını Config'den Al
        self._configure_init_params()

        # Veri tutucular (sl.Mat) - Bellek yönetimi için init'te oluşturuyoruz
        self.mat_image = sl.Mat()
        self.mat_depth = sl.Mat()
        self.sensors_data = sl.SensorsData()
        self.zed_pose = sl.Pose()

        self.ts_handler = TimestampHandler()
        self.is_open = False

    def _configure_init_params(self):
        """ZED Başlangıç parametrelerini ayarlar."""
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 30
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Hız için
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        self.init_params.depth_minimum_distance = 0.20
        self.init_params.depth_maximum_distance = 20.0
        self.init_params.camera_disable_self_calib = False
        self.init_params.sensors_required = False  # IMU bozuksa bile kamera açılsın

    def start(self):
        """Kamerayı ve Pozisyon Takibini (Positional Tracking) başlatır."""
        print("[ZED] Kamera açılıyor...")
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"[ZED HATA] Açılamadı: {err}", file=sys.stderr)
            return False

        # --- Positional Tracking (Konum Takibi) Ayarları ---
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True
        tracking_params.set_floor_as_origin = False  # Su üstünde zemin oynaktır, kapalı.
        tracking_params.enable_area_memory = False  # Su yüzeyi değiştiği için hafıza kapalı.
        tracking_params.enable_pose_smoothing = True

        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"[ZED UYARI] Konum takibi başlatılamadı: {err}")
        else:
            print("[ZED] Konum takibi (IMU Fusion) AKTİF.")

        self.is_open = True
        return True

    def get_data(self):
        """
        Kameradan anlık veriyi çeker.
        Return: (rgb_image_numpy, depth_map_numpy, pose_data, sensors_data)
        """
        if not self.is_open:
            return None, None, None, None

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            # 1. Görüntüyü Al (RGBA -> RGB dönüşümü dahil)
            self.zed.retrieve_image(self.mat_image, sl.VIEW.LEFT)
            frame_bgra = self.mat_image.get_data()
            frame_bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)

            # 2. Derinlik Haritasını Al
            self.zed.retrieve_measure(self.mat_depth, sl.MEASURE.DEPTH)
            depth_map = self.mat_depth.get_data()  # Numpy array döner

            # 3. Robotun Konumunu Al (Odometry)
            # REFERENCE_FRAME.WORLD: Başlangıç noktasına göre global konum
            state = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)

            # 4. Sensör Verilerini (IMU/Pusula) Al
            self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT)

            return frame_bgr, depth_map, self.zed_pose, self.sensors_data

        return None, None, None, None

    def get_fps(self):
        return self.zed.get_current_fps()

    def close(self):
        """Kamerayı güvenli şekilde kapatır."""
        if self.is_open:
            self.zed.disable_positional_tracking()
            self.zed.close()
            self.is_open = False
            print("[ZED] Kapatıldı.")