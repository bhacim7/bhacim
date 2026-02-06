import math
import time
from config import cfg, wp
from navigation.path_planner import PathPlanner
from navigation.controller import RobotController
from utils.math_tools import haversine, calculate_bearing

class Task3Speed:
    """
    Task 3: Speed Challenge (Modular State Machine with Retry)

    States:
    1. GATE_SEARCH: Navigate to T3_GATE_SEARCH.
    2. YELLOW_APPROACH: Navigate to T3_YELLOW_APPROACH, detect gate & direction.
    3. YELLOW_CIRCLE: Search pattern for Yellow Buoy (Class 9).
    4. YELLOW_FOUND: Circle the Yellow Buoy.
    5. RETURN_HOME: Retrace breadcrumbs.
    """

    # State Constants
    STATE_GATE_SEARCH = "GATE_SEARCH"
    STATE_YELLOW_APPROACH = "YELLOW_APPROACH"
    STATE_YELLOW_CIRCLE = "YELLOW_CIRCLE"
    STATE_YELLOW_FOUND = "YELLOW_FOUND"
    STATE_RETURN_HOME = "RETURN_HOME"
    STATE_FAILED = "TASK_FAILED"

    def __init__(self):
        # Tools
        self.planner = PathPlanner()
        self.controller = RobotController()

        # Variables
        self.retry_count = 0
        self.max_retries = 3
        self.turn_direction = 'RIGHT' # Default
        self.breadcrumbs = []
        self.gate_detected = False

        # State Tracking
        self.current_state = self.STATE_GATE_SEARCH
        self.finished = False
        self.next_task = "TASK5_DOCKING"

        # Internal Logic Vars
        self.path_lost_time = None
        self.search_angle = 0.0
        self.last_search_angle = None
        self.total_angle = 0.0
        self.target_buoy = None
        self.gate_midpoint = None

    def enter(self):
        print("[TASK 3] Starting Speed Challenge")
        self.current_state = self.STATE_GATE_SEARCH
        self.finished = False
        self.retry_count = 0
        self.breadcrumbs = []
        self.turn_direction = 'RIGHT'
        self.gate_detected = False
        self.gate_midpoint = None

    def execute(self, sensors, robot_state):
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()

        nav_map = sensors.get('nav_map')
        map_info = sensors.get('map_info')
        vision_objs = sensors.get('vision_objs', [])

        # --- STATE MACHINE ---

        # 1. GATE SEARCH
        if self.current_state == self.STATE_GATE_SEARCH:
            target_lat = wp.T3_GATE_SEARCH_LAT
            target_lon = wp.T3_GATE_SEARCH_LON

            dist = haversine(rlat, rlon, target_lat, target_lon)
            if dist < 3.0:
                print(f"[TASK 3] Reached Gate Search Pos. Switching to {self.STATE_YELLOW_APPROACH}")
                self.current_state = self.STATE_YELLOW_APPROACH
                self.gate_detected = False # Reset for this attempt
                self.gate_midpoint = None
                return 1500, 1500

            # Navigate using Heading
            target_bearing = calculate_bearing(rlat, rlon, target_lat, target_lon)
            # Convert robot yaw (radians, 0=East) to Compass Heading (0=North)
            current_heading_deg = (90 - math.degrees(ryaw)) % 360
            left, right = self.controller.calculate_heading_nav(current_heading_deg, target_bearing)
            return left, right

        # 2. YELLOW APPROACH
        elif self.current_state == self.STATE_YELLOW_APPROACH:
            self._record_breadcrumb(rlat, rlon)

            # A) Visual Task: Direction (Class 3=Left, 4=Right)
            for obj in vision_objs:
                cid = obj.get('class_id')
                if cid == 3:
                    self.turn_direction = 'LEFT'
                elif cid == 4:
                    self.turn_direction = 'RIGHT'

            # B) Visual Task: Gate (Red + Green Pair)
            # Assuming 'RED' and 'GREEN' labels are available or class ids.
            # Let's try to find a Red and Green obj close to each other
            red_obj = None
            green_obj = None
            for obj in vision_objs:
                if obj.get('label') == 'RED': red_obj = obj
                if obj.get('label') == 'GREEN': green_obj = obj

            if red_obj and green_obj:
                # Check distance between them
                d = math.sqrt((red_obj['x'] - green_obj['x'])**2 + (red_obj['y'] - green_obj['y'])**2)
                if d < 5.0: # Reasonable gate width
                    self.gate_detected = True
                    # Calculate Midpoint in Global Frame
                    mx = (red_obj['x'] + green_obj['x']) / 2.0
                    my = (red_obj['y'] + green_obj['y']) / 2.0
                    self.gate_midpoint = (mx, my)

            # C) Navigation
            target_lat = wp.T3_YELLOW_APPROACH_LAT
            target_lon = wp.T3_YELLOW_APPROACH_LON

            dist = haversine(rlat, rlon, target_lat, target_lon)

            # Arrival Check
            if dist < 3.0:
                if self.gate_detected:
                    print(f"[TASK 3] Gate Passed. Switching to {self.STATE_YELLOW_CIRCLE}")
                    self.current_state = self.STATE_YELLOW_CIRCLE
                    self.total_angle = 0.0
                    self.last_search_angle = None
                    return 1500, 1500
                else:
                    # Retry Logic
                    self.retry_count += 1
                    print(f"[TASK 3] Gate NOT detected. Retry {self.retry_count}/{self.max_retries}")
                    if self.retry_count >= self.max_retries:
                         print("[TASK 3] Max retries reached. FAILED.")
                         self.current_state = self.STATE_FAILED
                         self.finished = True
                    else:
                         self.current_state = self.STATE_GATE_SEARCH
                    return 1500, 1500

            # Determine Target (Gate Midpoint OR Waypoint)
            if self.gate_midpoint:
                # Use Gate Midpoint as temporary local target
                # But ensure we are still moving towards the global waypoint generally?
                # Actually, prompt says "adjust path to pass through midpoint while continuing towards T3_YELLOW_APPROACH"
                # If we use midpoint as A* target, we go there. Once passed, we resume.
                # Let's check if we passed midpoint.
                d_mid = math.sqrt((rx - self.gate_midpoint[0])**2 + (ry - self.gate_midpoint[1])**2)
                if d_mid < 1.0:
                    self.gate_midpoint = None # Passed, resume normal nav
                    tx, ty = self._get_target_xy(rlat, rlon, rx, ry, target_lat, target_lon)
                else:
                    tx, ty = self.gate_midpoint
            else:
                tx, ty = self._get_target_xy(rlat, rlon, rx, ry, target_lat, target_lon)

            # Plan Path
            path = self.planner.plan_path((rx, ry), (tx, ty), nav_map, map_info)

            if path:
                self.path_lost_time = None
                # Slow down if approaching gate to ensure detection? Prompt says "move forward slowly"
                left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path, current_speed_pwm=1550)
                return left, right
            else:
                 if self.path_lost_time is None: self.path_lost_time = time.time()
                 return self.controller.get_failsafe_command(self.path_lost_time)

        # 3. YELLOW CIRCLE
        elif self.current_state == self.STATE_YELLOW_CIRCLE:
            # Search for Class ID 9 (Yellow)
            for obj in vision_objs:
                if obj.get('class_id') == 9 or obj.get('label') == 'YELLOW':
                     print(f"[TASK 3] Yellow Object Found! Switching to {self.STATE_YELLOW_FOUND}")
                     self.target_buoy = obj
                     self.current_state = self.STATE_YELLOW_FOUND
                     self.total_angle = 0.0
                     self.last_search_angle = None
                     return 1500, 1500

            # Execute Circle Pattern (2m radius, 2 laps) around T3_YELLOW_APPROACH
            center_lat, center_lon = wp.T3_YELLOW_APPROACH_LAT, wp.T3_YELLOW_APPROACH_LON
            cx, cy = self._get_target_xy(rlat, rlon, rx, ry, center_lat, center_lon)
            radius = 2.0

            dx = rx - cx
            dy = ry - cy
            current_angle = math.atan2(dy, dx)
            target_angle = current_angle + 0.3 # CCW

            tx = cx + radius * math.cos(target_angle)
            ty = cy + radius * math.sin(target_angle)

            # Lap Counting
            if self.last_search_angle is None:
                self.last_search_angle = current_angle

            ang_diff = current_angle - self.last_search_angle
            ang_diff = (ang_diff + math.pi) % (2*math.pi) - math.pi
            if abs(ang_diff) < 1.0: self.total_angle += abs(ang_diff)
            self.last_search_angle = current_angle

            if self.total_angle >= 4 * math.pi: # 2 Laps
                print(f"[TASK 3] Object Not Found after 2 Laps. Retry {self.retry_count + 1}")
                self.retry_count += 1
                if self.retry_count >= self.max_retries:
                    self.current_state = self.STATE_FAILED
                    self.finished = True
                else:
                    self.current_state = self.STATE_GATE_SEARCH
                return 1500, 1500

            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
            return left, right

        # 4. YELLOW FOUND
        elif self.current_state == self.STATE_YELLOW_FOUND:
            if not self.target_buoy:
                self.current_state = self.STATE_RETURN_HOME
                return 1500, 1500

            cx, cy = self.target_buoy['x'], self.target_buoy['y']
            radius = 2.0

            dx = rx - cx
            dy = ry - cy
            current_angle = math.atan2(dy, dx)

            # Direction Logic
            step = 0.3 if self.turn_direction == 'LEFT' else -0.3 # Left=CCW(Pos), Right=CW(Neg)?
            # Standard Math: CCW is positive.
            # Prompt says: "If class_id == 4 found: Set turn_direction = RIGHT."
            # "Circle ... in the specific turn_direction".
            # Usually Right turn means Clockwise.

            target_angle = current_angle + (-0.3 if self.turn_direction == 'RIGHT' else 0.3)

            tx = cx + radius * math.cos(target_angle)
            ty = cy + radius * math.sin(target_angle)

             # Lap Counting (1 Lap)
            if self.last_search_angle is None:
                self.last_search_angle = current_angle
                self.total_angle = 0.0

            ang_diff = current_angle - self.last_search_angle
            ang_diff = (ang_diff + math.pi) % (2*math.pi) - math.pi
            if abs(ang_diff) < 1.0: self.total_angle += abs(ang_diff)
            self.last_search_angle = current_angle

            if self.total_angle >= 2 * math.pi: # 1 Lap
                print("[TASK 3] Circle Complete. Returning Home.")
                self.current_state = self.STATE_RETURN_HOME
                return 1500, 1500

            path = [(tx, ty)]
            left, right, _ = self.controller.calculate_pure_pursuit((rx, ry, ryaw), path)
            return left, right

        # 5. RETURN HOME
        elif self.current_state == self.STATE_RETURN_HOME:
            if not self.breadcrumbs:
                print("[TASK 3] Returned to Start. Task Complete.")
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

    def _record_breadcrumb(self, rlat, rlon):
        if not self.breadcrumbs:
            self.breadcrumbs.append((rlat, rlon))
            return

        last_lat, last_lon = self.breadcrumbs[-1]
        if haversine(rlat, rlon, last_lat, last_lon) >= 3.0:
            self.breadcrumbs.append((rlat, rlon))

    def _get_target_xy(self, rlat, rlon, rx, ry, tlat, tlon):
        dist = haversine(rlat, rlon, tlat, tlon)
        bearing = calculate_bearing(rlat, rlon, tlat, tlon)
        math_angle_rad = math.radians(90 - bearing)
        tx = rx + dist * math.cos(math_angle_rad)
        ty = ry + dist * math.sin(math_angle_rad)
        return tx, ty

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task
