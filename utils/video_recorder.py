import cv2
import threading
import queue
import time
import datetime
import os
from config import cfg


class VideoRecorder:
    def __init__(self, fps=20.0):
        self.recording = False
        self.queue = queue.Queue(maxsize=120)  # Bellek şişmesin diye limit
        self.thread = None
        self.writer = None
        self.fps = fps
        self.filename = ""

        if not os.path.exists("idaKayit"):
            os.makedirs("idaKayit")

    def start(self, width, height):
        if not cfg.RECORD_VIDEO: return

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"idaKayit/kayit_{ts}.mp4"

        # Gstreamer yerine standart mp4v codec (Daha uyumlu)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.filename, fourcc, self.fps, (width, height))

        self.recording = True
        self.thread = threading.Thread(target=self._record_loop, daemon=True)
        self.thread.start()
        print(f"[REC] Video kaydı başladı: {self.filename}")

    def frame(self, frame):
        """Kuyruğa yeni kare ekler."""
        if self.recording and not self.queue.full():
            self.queue.put(frame)

    def _record_loop(self):
        while self.recording or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=1.0)
                if self.writer:
                    self.writer.write(frame)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[REC HATA] {e}")

    def stop(self):
        if not self.recording: return

        self.recording = False
        if self.thread:
            self.thread.join()
        if self.writer:
            self.writer.release()
        print("[REC] Video kaydedildi ve kapatıldı.")