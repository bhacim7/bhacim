import math
from config import cfg, wp
from navigation.controller import RobotController
from utils.math_tools import haversine, calculate_bearing

class Task1Channel:
    """
    Refactored Task 1: Channel Navigation with Modular State Machine
    States:
        1. STATE_TO_START: Navigate to TASK1_START
        2. STATE_TO_MID: Navigate to TASK1_MID
        3. STATE_TO_FINAL: Navigate to TASK1_FINAL

    Logic:
        - Transition when < 2m from target.
        - Controller: Rotate-then-Move based on heading error.
        - Visual Override: Priority to Red/Green gate midpoint.
    """

    def __init__(self):
        # States
        self.STATE_TO_START = "STATE_TO_START"
        self.STATE_TO_MID = "STATE_TO_MID"
        self.STATE_TO_FINAL = "STATE_TO_FINAL"

        self.current_state = self.STATE_TO_START
        self.finished = False
        self.next_task = "TASK2_APPROACH"

        # Controller
        self.controller = RobotController()

        # Waypoints
        self.wp_start = (wp.TASK1_START_LAT, wp.TASK1_START_LON)
        self.wp_mid = (wp.TASK1_MID_LAT, wp.TASK1_MID_LON)
        self.wp_final = (wp.TASK1_FINAL_LAT, wp.TASK1_FINAL_LON)

    def enter(self):
        print("[TASK 1] Started: Modular State Machine Refactor")
        self.current_state = self.STATE_TO_START
        self.finished = False

    def get_current_target(self):
        if self.current_state == self.STATE_TO_START:
            return self.wp_start
        elif self.current_state == self.STATE_TO_MID:
            return self.wp_mid
        elif self.current_state == self.STATE_TO_FINAL:
            return self.wp_final
        return None

    def detect_gate_midpoint_map(self, vision_objs, robot_pose):
        """
        Finds the nearest Red/Green gate and returns its midpoint in MAP coordinates (x, y).
        """
        rx, ry, ryaw = robot_pose

        reds = [o for o in vision_objs if o['label'] == 'RED']
        greens = [o for o in vision_objs if o['label'] == 'GREEN']

        best_gate_dist = 999.0
        best_gate_mid = None

        for r in reds:
            for g in greens:
                # Distance between buoys (Map coordinates)
                # vision_objs contains 'x', 'y' in Global/Map frame (as per ObjectDetector code)
                d_buoys = math.sqrt((r['x'] - g['x'])**2 + (r['y'] - g['y'])**2)

                # Valid gate width (2m - 15m)
                if 2.0 < d_buoys < 15.0:
                    # Midpoint in Map frame
                    mx = (r['x'] + g['x']) / 2.0
                    my = (r['y'] + g['y']) / 2.0

                    # Distance to robot
                    d_robot = math.sqrt((mx - rx)**2 + (my - ry)**2)

                    if d_robot < best_gate_dist:
                        best_gate_dist = d_robot
                        best_gate_mid = (mx, my)

        return best_gate_mid

    def execute(self, sensors, robot_state):
        # Localizer'den gelen pose (Map coordinates, Yaw in Rad aligned to Compass)
        rx, ry, ryaw = robot_state.get_pose()
        rlat, rlon = robot_state.get_gps()
        vision_objs = sensors.get('vision_objs', [])

        if rlat == 0.0 or rlon == 0.0:
            print("[TASK 1] Waiting for GPS...")
            return 1500, 1500

        # 1. Determine Target Waypoint
        target_lat, target_lon = self.get_current_target()

        # 2. Check Distance for State Transition
        dist_to_target = haversine(rlat, rlon, target_lat, target_lon)

        # Debug Info
        # print(f"[TASK 1] State: {self.current_state}, Dist: {dist_to_target:.1f}m")

        if dist_to_target < 2.0:
            print(f"[TASK 1] Reached {self.current_state} (Dist: {dist_to_target:.2f}m)")

            if self.current_state == self.STATE_TO_START:
                self.current_state = self.STATE_TO_MID
            elif self.current_state == self.STATE_TO_MID:
                self.current_state = self.STATE_TO_FINAL
            elif self.current_state == self.STATE_TO_FINAL:
                print("[TASK 1] Mission Completed.")
                self.finished = True
                return 1500, 1500

            # Update target after transition
            target_lat, target_lon = self.get_current_target()

        # 3. Calculate Navigation Target (Bearing)

        # A. Default: Bearing to GPS Waypoint
        target_bearing = calculate_bearing(rlat, rlon, target_lat, target_lon)

        # B. Visual Override: Check for Gate
        # Note: vision_objs are in Map Frame (see ObjectDetector)
        gate_mid_map = self.detect_gate_midpoint_map(vision_objs, (rx, ry, ryaw))

        if gate_mid_map:
            mx, my = gate_mid_map
            # Calculate bearing to gate midpoint in Map Frame
            # Map Frame is ENU (East-North-Up). math.atan2(dy, dx) gives angle from East (CCW).
            angle_to_gate_rad = math.atan2(my - ry, mx - rx)

            # Convert ENU Angle to Compass Bearing (0=N, CW)
            # Compass = (90 - ENU_Deg) % 360
            target_bearing = (90 - math.degrees(angle_to_gate_rad)) % 360
            # print(f"[TASK 1] Gate Detected! Override Bearing: {target_bearing:.1f}")

        # 4. Heading Controller
        # Robot Heading (Compass)
        # Convert Robot Yaw (ENU, 0=East, CCW) to Compass (0=North, CW)
        current_heading = (90 - math.degrees(ryaw)) % 360

        left, right = self.controller.calculate_heading_nav(current_heading, target_bearing)

        return left, right

    def is_finished(self):
        return self.finished

    def get_next_task(self):
        return self.next_task
