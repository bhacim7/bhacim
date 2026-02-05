import math
import heapq
import cv2
import numpy as np


class PathPlanner:
    """
    A* (A-Star) Algoritması ile en kısa yolu bulan sınıf.
    [PERFORMANS GÜNCELLEMESİ]: Harita 1/8 oranında küçültülerek (Downsampling) aranır.
    """

    def __init__(self):
        self.SCALE = 0.125  # 800px -> 100px (1/8 Küçültme)

    def _heuristic(self, a, b, weight=1.5):
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * weight

    def _clamp_target_to_boundary(self, start_px, raw_goal_px, w, h):
        """
        Eğer hedef harita dışındaysa, robot ile hedef arasındaki açıyı bozmadan
        hedefi haritanın kenarına (sınırına) çeker.
        """
        sx, sy = start_px
        gx, gy = raw_goal_px

        # Eğer zaten içerideyse dokunma
        if 0 <= gx < w and 0 <= gy < h:
            return (int(gx), int(gy))

        # Yön vektörü (dx, dy)
        dx = gx - sx
        dy = gy - sy

        if dx == 0 and dy == 0: return start_px

        # Kesişim sürelerini (t) hesapla: start + t * vector = boundary
        t_min = float('inf')

        # Sağ kenar (x = w - 1) veya Sol kenar (x = 0)
        if dx > 0:
            t_min = min(t_min, (w - 1 - sx) / dx)
        elif dx < 0:
            t_min = min(t_min, (0 - sx) / dx)

        # Alt kenar (y = h - 1) veya Üst kenar (y = 0)
        if dy > 0:
            t_min = min(t_min, (h - 1 - sy) / dy)
        elif dy < 0:
            t_min = min(t_min, (0 - sy) / dy)

        # Yeni hedefi hesapla
        new_gx = sx + t_min * dx
        new_gy = sy + t_min * dy

        return (int(max(0, min(w - 1, new_gx))), int(max(0, min(h - 1, new_gy))))

    def _is_line_clear(self, start_px, goal_px, grid):
        """
        Bresenham algoritması veya benzeri ile iki nokta arasındaki çizgi
        boyunca engel var mı kontrol eder.
        """
        x0, y0 = start_px
        x1, y1 = goal_px

        # Basit örnekleme (Sampling) metodu
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = int(dist)  # Her pikselde bir kontrol
        if steps == 0: return True

        for i in range(steps + 1):
            t = i / steps
            x = int(x0 + (x1 - x0) * t)
            y = int(y0 + (y1 - y0) * t)

            # Harita sınır kontrolü ve engel kontrolü
            if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                if grid[y, x] == 0:  # 0 = Engel
                    return False  # Yol kapalı
        return True  # Yol açık

    def plan_path(self, start_world, target_world, nav_map, map_info):
        """
        GELİŞTİRİLMİŞ A* PLANLAYICI
        - Hedef harita dışındaysa açıyı koruyarak sınıra çeker (Clamping).
        - A* yol bulamazsa ve önü boşsa direkt çizgi çeker (Fallback).
        """
        if nav_map is None: return []

        # 1. DOWN-SAMPLING
        low_res_grid = cv2.resize(nav_map, None, fx=self.SCALE, fy=self.SCALE, interpolation=cv2.INTER_NEAREST)
        h_grid, w_grid = low_res_grid.shape

        # 2. Koordinat Dönüşümleri
        center_m = map_info['center']
        res = map_info['res']
        orig_w, orig_h = map_info['size']
        cw, ch = orig_w // 2, orig_h // 2

        def to_low_res_pixel(wx, wy):
            dx = wx - center_m[0]
            dy = wy - center_m[1]
            px_hi = int(cw + (dx / res))
            py_hi = int(ch - (dy / res))
            return int(px_hi * self.SCALE), int(py_hi * self.SCALE)

        start_px = to_low_res_pixel(start_world[0], start_world[1])
        raw_goal_px = to_low_res_pixel(target_world[0], target_world[1])

        # Başlangıç noktasını güvenli sınıra al
        start_px = (max(0, min(w_grid - 1, start_px[0])), max(0, min(h_grid - 1, start_px[1])))

        # [YENİ] Hedef Sınırlama (Clamping)
        # Hedefi haritanın köşesine değil, açıyı koruyarak kenarına yapıştırır.
        goal_px = self._clamp_target_to_boundary(start_px, raw_goal_px, w_grid, h_grid)

        # 3. Engel Kontrolü
        # Robot duvarın içindeyse yapacak bir şey yok, çıkmaya çalışmalı (Burada boş dönüyoruz)
        if low_res_grid[start_px[1], start_px[0]] == 0:
            return []

        # Hedef nokta engelse (duvarın içindeyse), hedefe en yakın boş noktayı ara (Opsiyonel)
        # Şimdilik direkt A* deniyoruz, A* zaten duvara giremez.

        # 4. A* Algoritması
        frontier = []
        heapq.heappush(frontier, (0, start_px))
        came_from = {start_px: None}
        cost_so_far = {start_px: 0}
        path_found = False
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal_px:
                path_found = True
                break

            # [PERFORMANS] Çok uzun süre ararsa kes (Opsiyonel)
            # if len(came_from) > 2000: break

            for dx, dy in neighbors:
                next_node = (current[0] + dx, current[1] + dy)

                if 0 <= next_node[0] < w_grid and 0 <= next_node[1] < h_grid:
                    if low_res_grid[next_node[1], next_node[0]] != 0:  # Engel değilse
                        move_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                        new_cost = cost_so_far[current] + move_cost

                        if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                            cost_so_far[next_node] = new_cost
                            priority = new_cost + self._heuristic(goal_px, next_node)
                            heapq.heappush(frontier, (priority, next_node))
                            came_from[next_node] = current

        # 5. [YENİ] FALLBACK MEKANİZMASI (B Planı)
        if not path_found:
            # A* yol bulamadı (Belki hedef çok karmaşık bir yerde veya harita yetmedi).
            # Ancak önümüz boşsa durmak yerine hedefe doğru (Bodoslama) gidelim.
            if self._is_line_clear(start_px, goal_px, low_res_grid):
                # Direkt kuş uçuşu yol döndür
                print("[PLAN] A* Başarısız -> Line-of-Sight Aktif (Direkt Gidiliyor)")
                return [start_world, target_world]
            else:
                return []  # Önümüzde engel var ve yol yok -> Dur.

        # 6. Yolu Geri Oluştur (Reconstruct)
        path_pixels = []
        curr = goal_px
        while curr is not None:
            path_pixels.append(curr)
            curr = came_from.get(curr)  # .get kullanarak hata riskini azalt
        path_pixels.reverse()

        # 7. Piksel -> Dünya Dönüşümü
        world_path = []
        for p in path_pixels:
            hx = p[0] / self.SCALE
            hy = p[1] / self.SCALE

            # Matematiksel dönüşüm (Ters işlem)
            wx = (hx - cw) * res + center_m[0]
            wy = center_m[1] + ((ch - hy) * res)
            world_path.append((wx, wy))

        # [YENİ] Path Smoothing (Yol Yumuşatma)
        world_path = self._smooth_path(world_path)

        return world_path

    def _smooth_path(self, path, iterations=1):
        """
        Zikzaklı grid yolunu yumuşatır.
        """
        if len(path) < 3: return path
        smoothed = list(path)

        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                prev_p = smoothed[i - 1]
                next_p = smoothed[i + 1]
                curr_p = smoothed[i]

                # Ağırlıklı Ortalama: %50 Mevcut, %25 Önceki, %25 Sonraki
                new_x = (prev_p[0] + next_p[0] + 2 * curr_p[0]) / 4.0
                new_y = (prev_p[1] + next_p[1] + 2 * curr_p[1]) / 4.0
                smoothed[i] = (new_x, new_y)

        return smoothed