import socket
import struct
import pickle
import cv2
import numpy as np
from config import cfg


class VideoStreamer:
    """
    Kamera görüntüsünü küçültür, Harita ve Telemetri verilerini üzerine yazar
    ve Yer İstasyonuna gönderir.
    """

    def __init__(self):
        self.client_socket = None
        self.ip = "192.168.1.25"
        self.port = 5000

        # GPS Fix Tipini Yazıya Çeviren Sözlük
        self.fix_map = {
            0: "No GPS", 1: "No FIX", 2: "2D Fix", 3: "3D Fix",
            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed", 7: "STATIC", 8: "PPP"
        }

        if cfg.STREAM_ENABLED:
            self._connect()

    def _connect(self):
        print(f"[STREAM] {self.ip}:{self.port} bağlanılıyor...")
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(1.0)
            self.client_socket.connect((self.ip, self.port))
            print(f"[STREAM] Bağlantı BAŞARILI.")
        except Exception as e:
            print(f"[STREAM UYARI] Bağlantı kurulamadı: {e}")
            self.client_socket = None

    def send_frame(self, frame, nav_map=None, telem_data=None):
        """
        frame: Kamera görüntüsü (Renkli)
        nav_map: (Opsiyonel) Harita verisi
        telem_data: (Opsiyonel) {
            'lidar_count': int, 'lidar_min': float,
            'dist_to_goal': float, 'target_course': float,
            'heading': float, 'gps_fix': int,
            'lat': float, 'lon': float,
            'mode': str
        }
        """
        if not cfg.STREAM_ENABLED or self.client_socket is None:
            return

        try:
            # 1. Ana görüntüyü küçült (640x360)
            stream_frame = cv2.resize(frame, (640, 360))

            # 2. Harita Overlay (Sağ Alt Köşe)
            if nav_map is not None:
                self._draw_map_overlay(stream_frame, nav_map)

            # 3. Telemetri Overlay (Yazılar)
            if telem_data:
                self._draw_telemetry_overlay(stream_frame, telem_data)

            # 4. Sıkıştır ve Gönder
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            ret, buffer = cv2.imencode('.jpg', stream_frame, encode_param)

            if ret:
                data = pickle.dumps(buffer)
                message = struct.pack("!Q", len(data)) + data
                self.client_socket.sendall(message)

        except Exception as e:
            self.close()

    def _draw_map_overlay(self, frame, nav_map):
        """Haritayı sağ alta çizer."""
        try:
            map_small = cv2.resize(nav_map, (150, 150), interpolation=cv2.INTER_NEAREST)
            map_color = cv2.cvtColor(map_small, cv2.COLOR_GRAY2BGR)

            # Engelleri Kırmızı Yap
            mask_obstacle = (map_small < 100)
            map_color[mask_obstacle] = [0, 0, 255]

            h_f, w_f = frame.shape[:2]
            h_m, w_m = map_color.shape[:2]
            y_off = h_f - h_m - 10
            x_off = w_f - w_m - 10

            frame[y_off:y_off + h_m, x_off:x_off + w_m] = map_color
            cv2.rectangle(frame, (x_off, y_off), (x_off + w_m, y_off + h_m), (255, 255, 0), 2)
        except:
            pass

    def _draw_telemetry_overlay(self, frame, data):
        """İstenilen bilgileri ekrana yazar."""
        # --- SOL TARAF (LIDAR) ---
        lidar_count = data.get('lidar_count', 0)
        lidar_min = data.get('lidar_min', 0)

        cv2.putText(frame, f"LIDAR Nokta(On): {lidar_count}", (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"LIDAR Min(On): {lidar_min:.0f}", (10, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- SAĞ TARAF (GPS & NAV) ---
        # Sadece GPS verisi varsa veya manuel modda değilsek göster
        x_pos = 350
        y_start = 20
        gap = 20

        # Mesafe
        dist = data.get('dist_to_goal')
        dist_str = f"H.Msf: {dist:.1f}m" if dist is not None else "H.Msf: -"
        cv2.putText(frame, dist_str, (x_pos, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Rota (Advised Course)
        crs = data.get('target_course')
        crs_str = f"Rota: {crs:.0f}" if crs is not None else "Rota: -"
        cv2.putText(frame, crs_str, (x_pos, y_start + gap), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)

        # Heading
        head = data.get('heading')
        head_str = f"Head: {head:.0f}" if head is not None else "Head: -"
        cv2.putText(frame, head_str, (x_pos, y_start + 2 * gap), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        # GPS Fix Tipi
        fix_val = data.get('gps_fix')
        fix_str = f"GPS: {self.fix_map.get(fix_val, 'Unknown')}"
        cv2.putText(frame, fix_str, (x_pos, y_start + 3 * gap), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Lat/Lon
        lat = data.get('lat')
        lon = data.get('lon')
        lat_str = f"La:{lat:.5f}" if lat else "La:-"
        lon_str = f"Lo:{lon:.5f}" if lon else "Lo:-"

        cv2.putText(frame, lat_str, (x_pos, y_start + 4 * gap), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, lon_str, (x_pos, y_start + 5 * gap), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Mod Bilgisi (Sol Üst)
        mode = data.get('mode', 'UNKNOWN')
        cv2.putText(frame, f"MOD: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    def close(self):
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None