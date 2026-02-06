import cv2
import math
import numpy as np
from ultralytics import YOLO
from config import cfg


class ObjectDetector:
    """
    YOLO kullanarak nesne tespiti yapar ve ZED derinlik haritası ile
    bu nesnelerin robot merkezli ve global koordinatlarını hesaplar.
    """

    def __init__(self):
        print(f"[YOLO] Model yükleniyor: {cfg.YOLO_MODEL_PATH}")
        try:
            self.model = YOLO(cfg.YOLO_MODEL_PATH)
            # Model TensorRT (.engine) ise, FP16 kullanımı otomatik olabilir
            # PyTorch (.pt) ise manuel GPU'ya atabiliriz:
            # self.model.to('cuda')
        except Exception as e:
            print(f"[YOLO HATA] Model yüklenemedi: {e}")
            self.model = None

        # Sınıf Eşleşmeleri (Modelinin çıktısına göre burayı güncelle!)
        # dalgaDeneme.py referans alındı:
        self.CLASS_MAP = {
            'RED': [0, 3, 5],
            'GREEN': [1, 4, 12],
            'YELLOW': [9],
            'BLACK': [10],
            'BLUE': [2]  # Task 5 için gerekebilir
        }

        # Kamera Parametreleri (Radyan cinsinden HFOV)
        self.hfov_rad = math.radians(cfg.CAM_HFOV)

    def detect(self, frame, depth_map, robot_pose):
        """
        Görüntüdeki nesneleri tespit eder ve 3D konumlarını hesaplar.

        Args:
            frame: RGB görüntü (Numpy array)
            depth_map: ZED derinlik haritası (Numpy array)
            robot_pose: (robot_x, robot_y, robot_yaw_radians)

        Returns:
            detected_objects: List of dict [{'label': 'RED', 'x': 5.0, 'y': 2.0, 'dist': 3.5}, ...]
            annotated_frame: Üzerine çizim yapılmış görüntü
        """
        detected_objects = []
        annotated_frame = frame.copy()

        if self.model is None or frame is None:
            return [], frame

        # 1. YOLO Tahmini
        results = self.model(frame, conf=cfg.YOLO_CONFIDENCE, verbose=False)[0]

        # 2. Sonuçları İşle
        # Ultralytics sonuçlarını döngüye al
        for box in results.boxes:
            # Sınıf ID ve Güven
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            # Etiket Belirleme (RED, GREEN, vb.)
            label = self._get_label_from_id(cls_id)
            if label == "UNKNOWN":
                continue

            # Kutu Koordinatları
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # 3. Derinlik Okuma (ZED Depth Map'ten)
            dist_m = self._get_depth_safe(depth_map, cx, cy, x1, y1, x2, y2)

            # Eğer mesafe geçerliyse koordinat hesapla
            if dist_m and dist_m < cfg.LIDAR_MAX_DIST:
                # Global Koordinat Hesabı
                global_x, global_y = self._calculate_global_pos(
                    (x1, y1, x2, y2), depth_map, robot_pose
                )

                # Listeye Ekle
                obj_data = {
                    'label': label,
                    'x': global_x,
                    'y': global_y,
                    'dist': dist_m,
                    'pixel': (cx, cy),
                    'conf': conf
                }
                detected_objects.append(obj_data)

                # Ekrana Çizim (Opsiyonel - Debug için)
                self._draw_debug(annotated_frame, x1, y1, x2, y2, label, dist_m)

        return detected_objects, annotated_frame

    def _get_label_from_id(self, cls_id):
        """Class ID'yi (int) string etikete çevirir."""
        for label, ids in self.CLASS_MAP.items():
            if cls_id in ids:
                return label
        return "UNKNOWN"

    def _get_depth_safe(self, depth_map, cx, cy, x1, y1, x2, y2):
        """
        Merkez pikselden derinlik okur. Hata varsa (NaN/Inf)
        kutunun ortalamasına bakmaya çalışır veya alan hesabı yapar.
        """
        h, w = depth_map.shape
        # Koordinatları sınırla
        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        dist = depth_map[cy, cx]

        # Geçerli bir değer mi? (Nan, Inf veya çok yakın değilse)
        if np.isnan(dist) or np.isinf(dist) or dist <= 0.1:
            # Alternatif: Eğer çok yakındaysa (derinlik okunamıyor) ve kutu büyükse
            area = (x2 - x1) * (y2 - y1)
            frame_area = w * h
            if area > (frame_area * 0.10):  # Ekranın %10'undan büyükse
                return 0.5  # Çok yakın varsay
            return None

        return float(dist)

    def _calculate_global_pos(self, bbox, depth_map, robot_pose):
        """
        [SENIOR UPDATE]
        Kutunun alt %15'ine bakarak daha doğru derinlik ölçer.
        """
        x1, y1, x2, y2 = map(int, bbox)
        cx = int((x1 + x2) / 2)

        # Hata Yönetimi: Koordinatlar harita dışındaysa None dön
        h, w = depth_map.shape[:2]
        if not (0 <= x1 < w and 0 <= y1 < h):
            return None, None

        # --- DÜZELTME BAŞLANGIÇ ---
        # Kutunun yüksekliği
        box_h = y2 - y1
        # Hedef nokta: Kutunun altından %15 yukarı
        target_cy = int(y2 - (box_h * 0.15))
        
        # Görüntü dışına taşmasın
        cy = max(0, min(target_cy, h - 1))
        # --------------------------

        # Derinlik Oku
        dist = self._get_depth_safe(depth_map, cx, cy)
        if dist is None or dist <= 0:
            return None, None

        # Global Konum Hesabı (Mevcut kodunun aynısı)
        rx, ry, ryaw = robot_pose
        pixel_offset = (cx - (w / 2)) / w
        angle_offset = -pixel_offset * self.hfov_rad
        obj_global_angle = ryaw + angle_offset

        obj_x = rx + (dist * math.cos(obj_global_angle))
        obj_y = ry + (dist * math.sin(obj_global_angle))

        return obj_x, obj_y

    def _draw_debug(self, frame, x1, y1, x2, y2, label, dist):
        """Tespit edilen nesneyi kare içine alır."""
        color = (0, 255, 0)
        if label == 'RED':
            color = (0, 0, 255)
        elif label == 'YELLOW':
            color = (0, 255, 255)
        elif label == 'BLACK':
            color = (0, 0, 0)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        text = f"{label} {dist:.1f}m"
        cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)