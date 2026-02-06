import os
from dataclasses import dataclass, field
from typing import Dict, Tuple

#ZED'in parametreleri zed_camera.py içerisinde

@dataclass
class RobotConfig:
    """
    Robotun tüm donanım ve yazılım ayarlarını tutan merkezi sınıf.
    """

    # --- GENEL OPERASYON ---
    # Yarışma modu: "GPS" (Açık Alan) veya "VISION" (Kapalı Alan)
    NAV_MODE: str = "GPS"
    START_TASK: str = "TASK2_AVOIDANCE"  # Başlangıç görevi

    # --- İLETİŞİM VE PORTLAR ---
    # Çevresel değişkenlerden (Env Var) okur, yoksa varsayılanı kullanır
    # 1. Pixhawk (Mavlink) - Motor Sürücü
    MAVLINK_PORT: str = os.getenv("MAV_PORT", "/dev/ttyACM0")
    MAVLINK_BAUD: int = 57600
    # 2. Telemetri Radyosu (JSON Veri/Komut) - Yer kontrol Bağlantısı
    TELEM_PORT: str = os.getenv("TELEM_PORT", "/dev/ttyUSB1")
    TELEM_BAUD: int = 57600

    # 3. Lidar
    LIDAR_PORT: str = "/dev/ttyUSB0"
    LIDAR_BAUDRATE: int = 1000000

    # --- MOTOR VE GÜÇ ---
    SOL_MOTOR_PIN: int = 1
    SAG_MOTOR_PIN: int = 3
    MOTOR_RELAY_PIN: int = 15
    GPIO_MODE: str = "BOARD"  # 'BOARD' veya 'BCM'

    # PWM Limitleri
    BASE_PWM: int = 1500
    MIN_PWM_LIMIT: int = 1100
    MAX_PWM_LIMIT: int = 1900
    ESC_INIT_DELAY: float = 3.0  # ESC'lerin açılması için bekleme süresi

    # Hız Ayarları
    CRUISE_PWM: int = 80  # Normal seyir hızı (Base PWM üzerine eklenir)
    ESCAPE_PWM: int = 300  # Engelden kaçış manevrası gücü
    T3_SPEED_PWM: int = 100  # Speed Challenge (Task 3) ekstra hızı
    MAX_PWM_CHANGE: int = 60  # Ani hızlanmayı önlemek için yumuşatma limiti

    # --- GÖRÜNTÜ İŞLEME VE VISION ---
    STREAM_ENABLED: bool = True # yer kontrole canlı kamera görüntüsü
    RECORD_VIDEO: bool = True  # SD Karta kayıt
    SHOW_LOCAL_WINDOW: bool = False  # Ekrana pencere açma (Headless modda(orine monitör takılı değilse) False olmalı)

    YOLO_MODEL_PATH: str = "/home/yarkin/roboboatIDA/roboboat/weights/best.engine"
    YOLO_CONFIDENCE: float = 0.35
    CAM_HFOV: float = 110.0  # Kamera yatay görüş açısı

    # Vision PID Kontrolcü
    Kp_PIXEL: float = 0.3
    Kd_PIXEL: float = 0.1

    # --- NAVİGASYON VE LIDAR ---
    LIDAR_MAX_DIST: float = 10.0
    LIDAR_EMERGENCY_STOP_M: float = 1.2  # Bu mesafede engel varsa dur/kaç
    LIDAR_KORIDOR_KP: float = 30.0  # Lidar ile ortalama katsayısı

    Kp_HEADING: float = 2.5  # GPS Modunda rota düzeltme katsayısı (Eski KpACI)

    # Navigasyon Fiziksel Parametreleri
    ROBOT_RADIUS_M: float = 0.45
    INFLATION_MARGIN_M: float = 0.10

    # --- TASK 6 (SESLİ KOMUT SİSTEMİ) ---
    T6_AUDIO_CHUNK: int = 2048
    T6_AUDIO_RATE: int = 44100
    T6_AUDIO_CHANNELS: int = 1

    # Frekans Aralıkları: {Ad: (Min, Max, Eşik)}
    # Mutable default argument hatası almamak için field kullanıyoruz
    T6_TARGET_FREQS: Dict[str, Tuple[int, int, int]] = field(default_factory=lambda: {
        '600Hz': (570, 630, 6000000),
        '800Hz': (760, 840, 2000000),
        '1000Hz': (950, 1050, 6000000),
        '1200Hz': (1140, 1260, 6000000),
    })


# Tek bir instance oluşturup dışarı açıyoruz

cfg = RobotConfig()
