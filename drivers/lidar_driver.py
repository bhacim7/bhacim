import time
import threading
import sys
from rplidar import RPLidar
from config import cfg


class LidarDriver:
    """
    RPLidar için dayanıklı (Robust) sürücü sınıfı.
    Bağlantı kopsa bile arka planda sürekli yeniden bağlanmayı dener.
    """

    def __init__(self):
        self.port = cfg.LIDAR_PORT
        self.baudrate = cfg.LIDAR_BAUDRATE
        self.lidar = None

        self.running = False
        self.thread = None

        # Veri paylaşımı için kilit ve tampon
        self.lock = threading.Lock()
        self.latest_scan = []
        self.last_scan_time = 0

    def start(self):
        """Okuma işlemini arka plan thread'inde başlatır."""
        if self.running:
            return

        print(f"[LIDAR] Başlatılıyor... ({self.port})")
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Lidar'ı ve thread'i güvenli şekilde durdurur."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        self._disconnect()
        print("[LIDAR] Durduruldu.")

    def get_latest_scan(self):
        """
        En son okunan tarama verisini döndürür.
        Return: [(quality, angle, distance), ...]
        """
        with self.lock:
            # Veri çok eskiyse (örn: bağlantı koptuysa) boş liste dönebiliriz
            # veya son veriyi koruyabiliriz. Şimdilik son veriyi dönüyoruz.
            return list(self.latest_scan)

    def is_fresh(self, timeout=0.5):
        """Verinin güncel olup olmadığını kontrol eder."""
        return (time.time() - self.last_scan_time) < timeout

    def _connect(self):
        """Lidar'a bağlanmayı dener."""
        try:
            # print(f"[LIDAR] Bağlanılıyor: {self.port}...")
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            self.lidar.start_motor()
            time.sleep(0.5)  # Motorun hızlanması için bekle
            # print("[LIDAR] Bağlantı Başarılı.")
            return True
        except Exception as e:
            # print(f"[LIDAR] Bağlantı Hatası: {e}")
            return False

    def _disconnect(self):
        """Güvenli çıkış ve motor durdurma."""
        if self.lidar:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
            self.lidar = None

    def _run_loop(self):
        """Ana okuma döngüsü (Thread içinde çalışır)."""
        while self.running:
            # 1. Bağlantı Yoksa Bağlan
            if self.lidar is None:
                if not self._connect():
                    time.sleep(2.0)  # Tekrar denemeden önce bekle
                    continue

            # 2. Veri Okuma
            try:
                # iter_scans: max_buf_meas tamponu sınırlar, min_len gürültüyü azaltır
                iterator = self.lidar.iter_scans(max_buf_meas=5000, min_len=5)

                for scan in iterator:
                    if not self.running:
                        break

                    # Veriyi işle: (quality, angle, distance)
                    valid_points = []
                    for quality, angle, distance in scan:
                        if distance > 0:
                            valid_points.append((quality, angle, distance))

                    # Global değişkeni güncelle
                    if valid_points:
                        with self.lock:
                            self.latest_scan = valid_points
                            self.last_scan_time = time.time()

            except Exception as e:
                print(f"[LIDAR] Okuma Hatası (Resetleniyor): {e}", file=sys.stderr)
                self._disconnect()
                time.sleep(1.0)  # Portu dinlendir