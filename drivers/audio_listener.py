import time
import threading
import numpy as np
import pyaudio
from config import cfg


class AudioListener:
    """
    Task 6 için Arka Plan Ses Dinleme ve Analiz Sürücüsü.
    PyAudio kullanarak mikrofonu dinler, FFT ile frekans analizi yapar
    ve belirli frekanslardaki (Düdük vb.) ses kalıplarını tespit eder.
    """

    def __init__(self):
        self.p = None
        self.stream = None
        self.running = False
        self.thread = None

        # Tespit Edilen Komutlar (Thread-Safe Erişim İçin)
        self.lock = threading.Lock()
        self.last_command = None  # Örn: 3 (Tek ses), 5 (Çift ses)
        self.last_freq_name = None
        self.last_timestamp = 0

    def start(self):
        """Mikrofonu açar ve dinleme thread'ini başlatır."""
        if self.running: return

        print("[AUDIO] Mikrofon başlatılıyor...")
        try:
            self.p = pyaudio.PyAudio()
            # Config'den ayarları alarak stream aç
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=cfg.T6_AUDIO_CHANNELS,
                rate=cfg.T6_AUDIO_RATE,
                input=True,
                frames_per_buffer=cfg.T6_AUDIO_CHUNK
            )

            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print("[AUDIO] Dinleme aktif.")

        except Exception as e:
            print(f"[AUDIO HATA] Mikrofon açılamadı: {e}")
            self.running = False

    def stop(self):
        """Dinlemeyi durdurur ve kaynakları serbest bırakır."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass

        if self.p:
            try:
                self.p.terminate()
            except:
                pass
        print("[AUDIO] Kapatıldı.")

    def get_command(self):
        """
        Son algılanan komutu döndürür ve temizler.
        Return: (command_id, freq_name) veya (None, None)
        command_id: 3 (Task 3'e git) veya 5 (Task 5'e git/Eve dön)
        """
        with self.lock:
            if self.last_command:
                # Veriyi al
                cmd = self.last_command
                freq = self.last_freq_name

                # Okunduğu için temizle (Bir kez işlensin)
                self.last_command = None
                self.last_freq_name = None

                return cmd, freq
            return None, None

    def _listen_loop(self):
        """Arka planda çalışan ana döngü (FFT ve State Machine)."""

        # Durum Makinesi Değişkenleri
        state = "LISTENING"
        first_blast_end_time = 0
        cooldown_start_time = 0
        BLAST_TIMEOUT = 2.0  # İki düdük arası maksimum bekleme süresi
        COOLDOWN_PERIOD = 3.0  # Algılamadan sonraki kör süre

        temp_detected_freq = None

        while self.running:
            try:
                # 1. Ses Verisini Oku (Bloklamadan okumaya çalış veya overflow yoksay)
                raw_data = self.stream.read(cfg.T6_AUDIO_CHUNK, exception_on_overflow=False)
                data_np = np.frombuffer(raw_data, dtype=np.int16)

                # 2. FFT Analizi (Frekans Bulma)
                # Hanning penceresi ile sinyali yumuşat
                fft_data = np.fft.fft(data_np * np.hanning(len(data_np)))
                magnitudes = np.abs(fft_data)[:len(fft_data) // 2]

                # Frekans eksenini oluştur
                frequencies = np.fft.fftfreq(len(data_np), 1.0 / cfg.T6_AUDIO_RATE)[:len(data_np) // 2]

                sound_present = None

                # 3. Hedef Frekansları Tara
                for freq_name, (min_f, max_f, threshold) in cfg.T6_TARGET_FREQS.items():
                    # İlgili frekans aralığındaki indeksleri bul
                    indices = np.where((frequencies >= min_f) & (frequencies <= max_f))
                    if len(indices[0]) > 0:
                        max_mag = np.max(magnitudes[indices[0]])
                        # Eşik değerini kontrol et
                        if max_mag > threshold:
                            sound_present = freq_name
                            break

                current_time = time.time()

                # 4. Durum Makinesi (State Machine)
                if state == "LISTENING":
                    if sound_present is not None:
                        print(f"[AUDIO] Sinyal 1 Algılandı: {sound_present}")
                        state = "SOUND_DETECTED"
                        temp_detected_freq = sound_present

                elif state == "SOUND_DETECTED":
                    # Ses kesilene kadar bekle
                    if sound_present is None:
                        state = "WAITING_FOR_NEXT"
                        first_blast_end_time = current_time
                        # print("[AUDIO] Sessizlik... 2. sinyal bekleniyor.")

                elif state == "WAITING_FOR_NEXT":
                    if sound_present is not None:
                        # 2. SES GELDİ -> TASK 5 (ÇİFT SİNYAL)
                        self._set_command(5, temp_detected_freq)
                        state = "COOLDOWN"
                        cooldown_start_time = current_time
                        print(f">>> [AUDIO] ÇİFT SİNYAL! ({temp_detected_freq}) -> TASK 5")

                    elif (current_time - first_blast_end_time) > BLAST_TIMEOUT:
                        # ZAMAN DOLDU -> TASK 3 (TEK SİNYAL)
                        self._set_command(3, temp_detected_freq)
                        state = "COOLDOWN"
                        cooldown_start_time = current_time
                        print(f">>> [AUDIO] TEK SİNYAL! ({temp_detected_freq}) -> TASK 3")

                elif state == "COOLDOWN":
                    if (current_time - cooldown_start_time) > COOLDOWN_PERIOD:
                        state = "LISTENING"
                        temp_detected_freq = None

            except Exception:
                # Hata durumunda (örn. buffer hatası) döngüyü kırma, devam et
                time.sleep(0.01)

    def _set_command(self, cmd_id, freq_name):
        """Thread-safe komut kaydetme."""
        with self.lock:
            self.last_command = cmd_id
            self.last_freq_name = freq_name
            self.last_timestamp = time.time()