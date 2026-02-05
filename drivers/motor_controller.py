import time
import threading
import math
import atexit
from pymavlink import mavutil
from config import cfg

# GPIO Kütüphanesi (Jetson/Rpi uyumlu, PC'de hata vermemesi için try-except)
try:
    import Jetson.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO

        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False
        print("[UYARI] GPIO kütüphanesi bulunamadı (PC Modu?). Röle çalışmayacak.")


class MotorDriver:
    """
    Hem Pixhawk (Mavlink) hem de Güç Rölesini (GPIO) yöneten birleşik sürücü.
    """

    def __init__(self):
        self.master = None
        self.msg_dict = {}  # Gelen son mesajları burada saklarız (GPS, HUD vb.)
        self.running = False
        self.thread = None

        # Röle Kurulumu
        self._setup_relay()

        # Pixhawk Bağlantısı
        self._connect_mavlink()

        # Çıkışta temizlik garantisi
        atexit.register(self.close)

    def _setup_relay(self):
        """Güç rölesi için GPIO ayarlarını yapar."""
        if not GPIO_AVAILABLE: return

        try:
            mode = GPIO.getmode()
            if mode is None:
                if cfg.GPIO_MODE == 'BCM':
                    GPIO.setmode(GPIO.BCM)
                else:
                    GPIO.setmode(GPIO.BOARD)

            # Başlangıçta KAPALI (LOW)
            GPIO.setup(cfg.MOTOR_RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)
            print(f"[GUC] Röle pini ({cfg.MOTOR_RELAY_PIN}) ayarlandı.")
        except Exception as e:
            print(f"[HATA] Röle kurulum hatası: {e}")

    def power_on(self):
        """Motorlara güç verir (Röleyi açar)."""
        if GPIO_AVAILABLE:
            GPIO.output(cfg.MOTOR_RELAY_PIN, GPIO.HIGH)
            print("[GUC] Motorlara güç verildi.")
            # ESC'lerin açılış sesleri için bekleme (Config'den)
            time.sleep(cfg.ESC_INIT_DELAY)

    def power_off(self):
        """Motorların gücünü keser (Röleyi kapatır)."""
        if GPIO_AVAILABLE:
            GPIO.output(cfg.MOTOR_RELAY_PIN, GPIO.LOW)
            print("[GUC] Motor gücü kesildi.")

    def _connect_mavlink(self):
        """Pixhawk'a (Mavlink) bağlanır."""
        # [DÜZELTME] Eski: cfg.SERIAL_PORT -> Yeni: cfg.MAVLINK_PORT
        print(f"[MAV] Bağlanılıyor: {cfg.MAVLINK_PORT} @ {cfg.MAVLINK_BAUD}")
        
        try:
            self.master = mavutil.mavlink_connection(cfg.MAVLINK_PORT, baud=cfg.MAVLINK_BAUD)
            print("[MAV] Bağlantı isteği gönderildi, Heartbeat bekleniyor...")
            
            # Heartbeat bekle (Timeout ekleyelim ki sonsuza kadar donmasın)
            if self.master.wait_heartbeat(timeout=5.0):
                print("[MAV] HEARTBEAT ALINDI! Bağlantı Başarılı.")
                
                # Veri akışını başlat (GPS, HUD vb. için)
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    5,  # 5 Hz
                    1
                )
                
                # Dinleyici thread'i başlat
                self.running = True
                self.thread = threading.Thread(target=self._listener_loop, daemon=True)
                self.thread.start()
                
            else:
                print("[MAV HATA] Heartbeat alınamadı! (Kabloyu kontrol et)")
                
        except Exception as e:
            print(f"[MAV KRİTİK HATA] Bağlantı kurulamadı: {e}")

    def _listener_loop(self):
        """Arka planda Mavlink mesajlarını okur ve sözlüğe yazar."""
        while self.running:
            try:
                # Bloklamadan oku
                msg = self.master.recv_match(blocking=False)
                if msg:
                    # Mesaj tipine göre sözlüğe kaydet (En güncel hali tutulur)
                    self.msg_dict[msg.get_type()] = msg
                else:
                    time.sleep(0.01)  # CPU'yu yormamak için minik bekleme
            except Exception:
                pass

    def arm(self):
        """Aracı ARM eder (Motorları aktifleştirir)."""
        if not self.master: return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        print("[MAV] ARM isteği gönderildi.")
        # self.master.motors_armed_wait() # Bloklamamak için beklemiyoruz

    def disarm(self):
        """Aracı DISARM eder (Güvenli mod)."""
        if not self.master: return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("[MAV] DISARM isteği gönderildi.")

    def set_mode(self, mode_name="MANUAL"):
        """Uçuş modunu değiştirir (MANUAL, GUIDED, vb.)."""
        if not self.master: return
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"[MAV HATA] Bilinmeyen mod: {mode_name}")
            return

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"[MAV] Mod değiştirildi: {mode_name}")

    def set_pwm(self, left_pwm, right_pwm):
        """
        Motorlara PWM sinyali gönderir.
        Config dosyasındaki pin numaralarını kullanır.
        """
        if not self.master: return

        # Limitleri kontrol et (Güvenlik)
        left_pwm = max(cfg.MIN_PWM_LIMIT, min(cfg.MAX_PWM_LIMIT, int(left_pwm)))
        right_pwm = max(cfg.MIN_PWM_LIMIT, min(cfg.MAX_PWM_LIMIT, int(right_pwm)))

        # MAV_CMD_DO_SET_SERVO komutu ile servo sürme
        # Sol Motor
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            cfg.SOL_MOTOR_PIN, left_pwm, 0, 0, 0, 0, 0
        )
        # Sağ Motor
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            cfg.SAG_MOTOR_PIN, right_pwm, 0, 0, 0, 0, 0
        )

    def get_gps_data(self):
        """
        Mevcut GPS verisini döndürür.
        Return: (lat, lon, fix_type) veya (None, None, None)
        """
        msg = self.msg_dict.get('GPS_RAW_INT')
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            fix = msg.fix_type
            return lat, lon, fix
        return None, None, None

    def get_heading(self):
        """Pusula verisini (Heading) döndürür (0-360 derece)."""
        msg = self.msg_dict.get('VFR_HUD')  # GLOBAL_POSITION_INT alternatifi
        if msg:
            return msg.heading
        return 0.0

    def get_speed(self):
        """Yatay hızı (m/s) döndürür."""
        msg = self.msg_dict.get('VFR_HUD')
        if msg:
            return msg.groundspeed
        return 0.0

    def close(self):
        """Sistemi güvenli şekilde kapatır."""
        self.running = False
        self.power_off()  # Önce gücü kes

        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup(cfg.MOTOR_RELAY_PIN)
            except:
                pass

        if self.master:
            self.master.close()
