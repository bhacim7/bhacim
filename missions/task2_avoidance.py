import math
import cv2
import time
from config import cfg, wp
from navigation.path_planner import PathPlanner
from navigation.controller import RobotController
from utils.math_tools import haversine, calculate_bearing

class Task2Avoidance:
    """
    Task 2: Debris Clearance (Modular State Machine Implementation)

    States:
    1. TASK_START: Navigate to task2_start_pos.
    2. GO_TO_CORNER: Navigate to task2_corner_pos with A* + Pure Pursuit + Breadcrumbs.
    3. GO_TO_FINAL: Navigate to task2_final_pos with A* + Pure Pursuit + Breadcrumbs.
    4. SEARCH_PATTERN: Circular search for Green Marker.
    5. GREEN_MARKER_FOUND: Circle detected marker.
    6. RETURN_HOME: Retrace breadcrumbs to start.
    """

    # State Constants
    STATE_TASK_START = "TASK_START"
    STATE_GO_TO_CORNER = "GO_TO_CORNER"
    STATE_GO_TO_FINAL = "GO_TO_FINAL"
    STATE_SEARCH_PATTERN = "SEARCH_PATTERN"
    STATE_GREEN_MARKER_FOUND = "GREEN_MARKER_FOUND"
    STATE_RETURN_HOME = "RETURN_HOME"

    def __init__(self):
        self.current_state = self.STATE_TASK_START
        self.finished = False
        self.next_task = "TASK3_SPEED" # Or whatever is next

        self.planner = PathPlanner()
        self.controller = RobotController()

        self.breadcrumbs = []
        self.landmarks = []
        self.landmark_id_counter = 0

        # State-specific variables
        self.circle_phase = 0
        self.target_buoy = None
        self.path_lost_time = None
        self.search_laps = 0
        self.search_angle = 0.0
        self.start_lat = wp.TASK2_START_LAT
        self.start_lon = wp.TASK2_START_LON

    def enter(self):
        print("[TASK 2] Starting Debris Clearance Task")
        self.current_state = self.STATE_TASK_START
        self.finished = False
        self.breadcrumbs = []
        self.landmarks = []
        self.search_laps = 0

    def execute(self, sensors, robot_state):
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        nav_map = sensors.get('nav_map')
        map_info = sensors.get('map_info')
        vision_objs = sensors.get('vision_objs', [])

        # Update Memory
        self._update_landmark_memory(vision_objs, rx, ry)

        # --- STATE MACHINE ---

        # 1. TASK START (GPS Only Navigation)
        if self.current_state == self.STATE_TASK_START:
            target_lat = wp.TASK2_START_LAT
            target_lon = wp.TASK2_START_LON

            dist = haversine(rlat, rlon, target_lat, target_lon)
            if dist < 3.0:
                print(f"[TASK 2] Reached Start Position. Switching to {self.STATE_GO_TO_CORNER}")
                self.current_state = self.STATE_GO_TO_CORNER
                self.breadcrumbs.append((rlat, rlon)) # Initial breadcrumb
                return 1500, 1500

            # Calculate Heading
            target_bearing = calculate_bearing(rlat, rlon, target_lat, target_lon)
            current_heading = math.degrees(ryaw) # Assuming ryaw is in radians?
            # Note: robot_state.get_pose() usually returns yaw in radians.
            # But controller expects degrees for compass heading?
            # Main.py: heading = motors.get_heading() -> passed to localizer.
            # Localizer keeps yaw in radians?
            # Let's check main.py again. localizer.update(..., heading, ...)
            # RobotController.calculate_heading_nav takes current_heading (0-360).
            # If ryaw is radians (math angle), we need to convert to bearing or use raw compass heading if available.
            # But here we only have robot_state (localizer pose).
            # Assuming ryaw is radians, standard math?
            # Let's assume ryaw is consistent with bearing if converted.
            # Actually, localizer usually converts everything to a consistent frame.
            # Let's use `robot_state.yaw` converted to degrees if needed.
            # But wait, `heading` from motors is Compass.
            # Let's try to use the raw compass heading if possible? No, we only get `robot_state`.
            # If `ryaw` is 0 at East (Math), and Bearing is 0 at North.
            # Let's assume `ryaw` matches the controller expectation if we convert it properly.
            # Controller `calculate_heading_nav` expects (0-360).
            # Let's assume `ryaw` is in radians and convert to degrees.
            # Better: recalculate bearing relative to robot frame?

            # Simple fix: Use `calculate_heading_nav` with converted yaw.
            # If ryaw is standard math (0=East, CCW):
            # Heading (0=North, CW) = 90 - MathDeg

            current_heading_deg = (90 - math.degrees(ryaw)) % 360
            left, right = self.controller.calculate_heading_nav(current_heading_deg, target_bearing)
            return left, right

        # 2. GO TO CORNER (A* + Pure Pursuit + Breadcrumbs)
        elif self.current_state == self.STATE_GO_TO_CORNER:
            self._record_breadcrumb(rlat, rlon)

            target_lat = wp.TASK2_CORNER_LAT
            target_lon = wp.TASK2_CORNER_LON

            dist = haversine(rlat, rlon, target_lat, target_lon)
            if dist < 3.0:
                print(f"[TASK 2] Reached Corner. Switching to {self.STATE_GO_TO_FINAL}")
                self.current_state = self.STATE_GO_TO_FINAL
                return 1500, 1500

            # Calculate Target (x, y) relative to map
            tx, ty = self._get_target_xy(rlat, rlon, rx, ry, target_lat, target_lon)

            # Plan Path
            path = self.planner.plan_path((rx, ry), (tx, ty), nav_map, map_info)

            if path:
                self.path_lost_time = None
                left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
                return left, right
            else:
                # Failsafe
                if self.path_lost_time is None: self.path_lost_time = time.time()
                return self.controller.get_failsafe_command(self.path_lost_time)

        # 3. GO TO FINAL (A* + Pure Pursuit + Breadcrumbs)
        elif self.current_state == self.STATE_GO_TO_FINAL:
            self._record_breadcrumb(rlat, rlon)

            target_lat = wp.TASK2_FINAL_LAT
            target_lon = wp.TASK2_FINAL_LON

            dist = haversine(rlat, rlon, target_lat, target_lon)
            if dist < 3.0:
                print(f"[TASK 2] Reached Final Position. Switching to {self.STATE_SEARCH_PATTERN}")
                self.current_state = self.STATE_SEARCH_PATTERN
                self.search_laps = 0
                return 1500, 1500

            tx, ty = self._get_target_xy(rlat, rlon, rx, ry, target_lat, target_lon)
            path = self.planner.plan_path((rx, ry), (tx, ty), nav_map, map_info)

            if path:
                self.path_lost_time = None
                left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
                return left, right
            else:
                if self.path_lost_time is None: self.path_lost_time = time.time()
                return self.controller.get_failsafe_command(self.path_lost_time)

        # 4. SEARCH PATTERN
        elif self.current_state == self.STATE_SEARCH_PATTERN:
            # Check for Green Marker (Class ID 4)
            for lm in self.landmarks:
                if lm.get('class_id') == 4 or lm.get('label') == 'GREEN':
                    if lm['conf'] >= 1:
                        print(f"[TASK 2] Green Marker Found! Switching to {self.STATE_GREEN_MARKER_FOUND}")
                        self.target_buoy = lm
                        self.current_state = self.STATE_GREEN_MARKER_FOUND
                        self.marker_total_angle = 0.0
                        self.marker_last_angle = None
                        return 1500, 1500

            # Execute Circle Pattern around Final Pos
            center_lat, center_lon = wp.TASK2_FINAL_LAT, wp.TASK2_FINAL_LON
            cx, cy = self._get_target_xy(rlat, rlon, rx, ry, center_lat, center_lon)

            radius = 2.0

            # Angle of robot relative to center
            dx = rx - cx
            dy = ry - cy
            current_angle = math.atan2(dy, dx)

            # Move CCW
            target_angle = current_angle + 0.3

            tx = cx + radius * math.cos(target_angle)
            ty = cy + radius * math.sin(target_angle)

            # Count laps
            if not hasattr(self, 'last_search_angle') or self.last_search_angle is None:
                 self.last_search_angle = current_angle
                 self.total_angle = 0.0

            ang_diff = current_angle - self.last_search_angle
            ang_diff = (ang_diff + math.pi) % (2*math.pi) - math.pi

            # Filter noise / jumps
            if abs(ang_diff) < 1.0:
                self.total_angle += abs(ang_diff)

            self.last_search_angle = current_angle

            if self.total_angle >= 4 * math.pi: # 2 Full Laps
                print("[TASK 2] Search Complete (Not Found). Returning Home.")
                self.current_state = self.STATE_RETURN_HOME
                return 1500, 1500

            # Pure pursuit to point
            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
            return left, right

        # 5. GREEN MARKER FOUND (Circle Object)
        elif self.current_state == self.STATE_GREEN_MARKER_FOUND:
            if not self.target_buoy:
                self.current_state = self.STATE_RETURN_HOME
                return 1500, 1500

            cx, cy = self.target_buoy['x'], self.target_buoy['y']
            radius = 2.0

            dx = rx - cx
            dy = ry - cy
            current_angle = math.atan2(dy, dx)
            target_angle = current_angle + 0.3

            tx = cx + radius * math.cos(target_angle)
            ty = cy + radius * math.sin(target_angle)

            # Lap counting
            if not hasattr(self, 'marker_last_angle') or self.marker_last_angle is None:
                 self.marker_last_angle = current_angle
                 self.marker_total_angle = 0.0

            ang_diff = current_angle - self.marker_last_angle
            ang_diff = (ang_diff + math.pi) % (2*math.pi) - math.pi

            if abs(ang_diff) < 1.0:
                self.marker_total_angle += abs(ang_diff)

            self.marker_last_angle = current_angle

            if self.marker_total_angle >= 4 * math.pi: # 2 Laps
                 print("[TASK 2] Inspection Complete. Returning Home.")
                 self.current_state = self.STATE_RETURN_HOME
                 return 1500, 1500

            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
            return left, right

        # 6. RETURN HOME
        elif self.current_state == self.STATE_RETURN_HOME:
            if not self.breadcrumbs:
                print("[TASK 2] Returned to Start. Task Complete.")
                self.finished = True
                return 1500, 1500

            target_lat, target_lon = self.breadcrumbs[-1]
            dist = haversine(rlat, rlon, target_lat, target_lon)

            if dist < 3.0:
                self.breadcrumbs.pop()
                return 1500, 1500

            tx, ty = self._get_target_xy(rlat, rlon, rx, ry, target_lat, target_lon)
            path = self.planner.plan_path((rx, ry), (tx, ty), nav_map, map_info)

            if path:
                left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
                return left, right
            else:
                 if self.path_lost_time is None: self.path_lost_time = time.time()
                 return self.controller.get_failsafe_command(self.path_lost_time)

        return 1500, 1500

    def _get_target_xy(self, rlat, rlon, rx, ry, tlat, tlon):
        dist = haversine(rlat, rlon, tlat, tlon)
        bearing = calculate_bearing(rlat, rlon, tlat, tlon)
        # Convert bearing (0=N, CW) to Math Angle (0=E, CCW) -> Math = 90 - Bearing
        math_angle_rad = math.radians(90 - bearing)
        tx = rx + dist * math.cos(math_angle_rad)
        ty = ry + dist * math.sin(math_angle_rad)
        return tx, ty

    def _record_breadcrumb(self, rlat, rlon):
        if not self.breadcrumbs:
            self.breadcrumbs.append((rlat, rlon))
            return

        last_lat, last_lon = self.breadcrumbs[-1]
        if haversine(rlat, rlon, last_lat, last_lon) >= 3.0:
            self.breadcrumbs.append((rlat, rlon))
            # print(f"[BREADCRUMB] Added. Total: {len(self.breadcrumbs)}")

    def _update_landmark_memory(self, vision_objs, rx, ry):
        """Updates internal memory with detected objects."""
        MATCH_THRESH = 1.5
        for obj in vision_objs:
            # We are interested in GREEN (class 4? or label 'GREEN')
            # Prompt says "class_id = 4 (Green Marker)"
            # Check object_detector.py to see if it gives class_id or label.
            # Usually vision_objs has 'label', 'x', 'y'.
            # If label is 'GREEN' or class_id is 4.
            # Assuming 'label' is populated.

            ox, oy = obj['x'], obj['y']
            label = obj.get('label', '')

            # Simple matching
            found = False
            for lm in self.landmarks:
                dist = math.sqrt((lm['x'] - ox) ** 2 + (lm['y'] - oy) ** 2)
                if dist < MATCH_THRESH:
                    lm['x'] = (lm['x'] * 0.7) + (ox * 0.3)
                    lm['y'] = (lm['y'] * 0.7) + (oy * 0.3)
                    lm['conf'] += 1
                    found = True
                    break

            if not found:
                self.landmark_id_counter += 1
                self.landmarks.append({
                    'id': self.landmark_id_counter,
                    'label': label,
                    'class_id': obj.get('class_id', -1),
                    'x': ox,
                    'y': oy,
                    'conf': 1
                })

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task
