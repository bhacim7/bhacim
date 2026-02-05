import serial
import json
import threading
import queue
import time
from config import cfg


class TelemetryDriver:
    """
    Yer İstasyonu (GCS) ile JSON formatında haberleşmeyi sağlar.
    - Robot verilerini gönderir (TX).
    - Komutları dinler (RX) ve kuyruğa atar.
    """

    def __init__(self):
        self.ser = None
        self.running = False
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()  # Gelen komutlar burada birikir
        self.thread = None

    def start(self):
        print(f"[TELEM] Bağlanılıyor: {cfg.TELEM_PORT} @ {cfg.TELEM_BAUD}")
        try:
            self.ser = serial.Serial(cfg.TELEM_PORT, cfg.TELEM_BAUD, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            print("[TELEM] Bağlantı başarılı.")
        except Exception as e:
            print(f"[TELEM HATA] Bağlanamadı: {e}")

    def send_status(self, payload):
        """Gönderilecek veriyi kuyruğa ekler."""
        if self.running:
            self.tx_queue.put(payload)

    def get_command(self):
        """Kuyruktan bir komut çeker (varsa)."""
        try:
            return self.rx_queue.get_nowait()
        except queue.Empty:
            return None

    def _worker_loop(self):
        """Okuma ve Yazma işlemlerini yapan ana döngü."""
        while self.running:
            # 1. OKUMA (RX)
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        try:
                            cmd = json.loads(line)
                            print(f"[TELEM CMD] Alındı: {cmd}")
                            self.rx_queue.put(cmd)
                        except json.JSONDecodeError:
                            pass
            except Exception as e:
                print(f"[TELEM RX HATA] {e}")

            # 2. YAZMA (TX)
            try:
                while not self.tx_queue.empty():
                    payload = self.tx_queue.get_nowait()
                    msg = json.dumps(payload) + "\n"
                    self.ser.write(msg.encode('utf-8'))
            except Exception as e:
                print(f"[TELEM TX HATA] {e}")

            time.sleep(0.05)  # CPU dinlendirme

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()