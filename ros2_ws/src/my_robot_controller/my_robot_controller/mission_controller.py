#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, String, Int32
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap # [NEW] Import for Map Clearing
from action_msgs.msg import GoalStatus
import time

# --- STATES ---
STATE_IDLE = 0        
STATE_NAV_A = 1       
STATE_SEARCH_A = 2    
STATE_APPROACH_A = 3  

# Pick States
STATE_PICK_ACTION = 4    
STATE_PICK_WAIT = 41     
STATE_PICK_VERIFY = 42   
STATE_BACKING_UP = 43    

STATE_NAV_B = 5       
STATE_SEARCH_B = 6    
STATE_APPROACH_B = 7  
STATE_PLACE = 8       

# [NEW] Recovery State
STATE_RECOVERY_WAIT = 99 

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- CONFIGURATION ---
        self.pos_a_x = -0.0355; self.pos_a_y = 5.8193 
        self.pos_b_x = 1.3364;  self.pos_b_y = 5.5942 
        
        # --- LOOP CONFIGURATION ---
        self.max_mission_loops = 3 
        self.current_loop_count = 0

        # Retry Configuration (Pick)
        self.pick_attempts = 0
        self.max_pick_attempts = 5
        
        # [NEW] Recovery Configuration (Navigation)
        self.nav_recovery_attempts = 0
        self.max_nav_recoveries = 3
        self.last_nav_target = None # To remember where we were going (A or B)

        self.state_start_time = 0.0 # Timer for non-blocking waits

        # --- SPEED & DISTANCE SETTINGS ---
        self.max_approach_speed = 0.5  
        self.min_approach_speed = 0.08  
        self.max_turn_kp = 0.003
        self.min_turn_kp = 0.001
        self.slowdown_radius = 600.0    
        
        self.stop_distance_cube = 200.0     
        self.stop_distance_platform = 110.0 
        self.stable_stop_count = 0
        self.required_stop_counts = 3

        # --- PUBS/SUBS ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vision_mode_pub = self.create_publisher(Int32, '/vision_mode', 10)
        self.gripper_pub = self.create_publisher(Int32, '/gripper_cmd', 10)
        
        self.create_subscription(Float32MultiArray, '/target_info', self.vision_callback, 10)
        self.create_subscription(Float32MultiArray, '/robot_status', self.sensor_callback, 10)
        self.create_subscription(String, '/mission_trigger', self.trigger_callback, 10)

        # --- CLIENTS ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # [NEW] Service Clients for Clearing Maps
        self.clear_local_map_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_map_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        # --- VARIABLES ---
        self.current_state = STATE_IDLE
        self.target_found = False
        self.error_x = 0.0
        self.tof_dist = 9999.0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Mission Ready. Will run {self.max_mission_loops} times.")

    def trigger_callback(self, msg):
        if msg.data == "start" and self.current_state == STATE_IDLE:
            self.get_logger().info("STARTING MISSION...")
            self.current_loop_count = 0 
            self.start_sequence()

    def start_sequence(self):
        self.send_gripper_cmd(3) # RESET
        time.sleep(1.0) 
        self.send_gripper_cmd(0)
        time.sleep(1.0)

        self.start_nav_to_a()

    def send_gripper_cmd(self, val):
        msg = Int32(); msg.data = val
        self.gripper_pub.publish(msg)

    def set_vision_mode(self, mode):
        msg = Int32(); msg.data = mode
        self.vision_mode_pub.publish(msg)

    def vision_callback(self, msg):
        if len(msg.data) >= 2:
            self.target_found = bool(msg.data[0])
            self.error_x = msg.data[1]

    def sensor_callback(self, msg):
        if len(msg.data) >= 2: self.tof_dist = msg.data[1]

    # --- [NEW] RECOVERY HELPER ---
    def perform_map_reset(self):
        self.get_logger().warn("⚠️ TRIGGERING MAP RESET RECOVERY...")
        
        # Clear Local Costmap
        if self.clear_local_map_client.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_local_map_client.call_async(req)
        
        # Clear Global Costmap
        if self.clear_global_map_client.wait_for_service(timeout_sec=1.0):
            req = ClearEntireCostmap.Request()
            self.clear_global_map_client.call_async(req)

    # --- NAVIGATION HELPER ---
    def send_nav_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w
        
        self.get_logger().info(f"Navigating to {x}, {y}...")
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal Rejected! Retrying immediately...")
            # If rejected immediately, maybe try reset? For now just retry same state.
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("DESTINATION REACHED!")
            self.nav_recovery_attempts = 0 # Reset recovery counter
            
            if self.current_state == STATE_NAV_A:
                self.get_logger().info("Searching for CUBE...")
                self.set_vision_mode(1) 
                self.current_state = STATE_SEARCH_A
                self.pick_attempts = 0 
            
            elif self.current_state == STATE_NAV_B:
                self.get_logger().info("Searching for PLATFORM...")
                self.set_vision_mode(2) 
                self.current_state = STATE_SEARCH_B

        else:
            # === [NEW] FAILURE HANDLING ===
            self.get_logger().warn(f"Nav Failed (Status: {status}). Entering Recovery...")
            
            if self.nav_recovery_attempts < self.max_nav_recoveries:
                self.nav_recovery_attempts += 1
                
                # Remember where we were going so we can retry later
                if self.current_state == STATE_NAV_A:
                    self.last_nav_target = 'A'
                elif self.current_state == STATE_NAV_B:
                    self.last_nav_target = 'B'
                
                # Trigger Reset
                self.perform_map_reset()
                
                # Enter Wait State
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
                self.current_state = STATE_RECOVERY_WAIT
                
            else:
                self.get_logger().error("CRITICAL: Recovery failed 3 times. Aborting Mission.")
                self.current_state = STATE_IDLE

    # --- HELPER: CALCULATE SPEED ---
    def get_approach_speed(self, distance, stop_dist):
        if distance > self.slowdown_radius:
            return self.max_approach_speed
        else:
            ratio = (distance - stop_dist) / (self.slowdown_radius - stop_dist)
            ratio = max(0.0, min(1.0, ratio)) 
            return self.min_approach_speed + (self.max_approach_speed - self.min_approach_speed) * ratio
        
    def get_turn_kp(self, distance, stop_dist):
        if distance > self.slowdown_radius:
            return self.max_turn_kp 
        else:
            ratio = (distance - stop_dist) / (self.slowdown_radius - stop_dist)
            ratio = max(0.0, min(1.0, ratio))
            return self.min_turn_kp + (self.max_turn_kp - self.min_turn_kp) * ratio

    # --- MAIN LOOP ---
# --- MAIN LOOP ---
    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9 

        # === [MODIFIED] RECOVERY BACKUP LOGIC ===
        if self.current_state == STATE_RECOVERY_WAIT:
            elapsed = now - self.state_start_time
            
            # Phase 1: Move Backward blindly for 1.0 second
            if elapsed < 1.0:
                self.get_logger().info("RECOVERY: Backing up...", throttle_duration_sec=1.0)
                cmd.linear.x = -0.20 # Move back slowly
            
            # Phase 2: Stop and Wait (Allow map to clear/settle)
            elif elapsed < 2.5:
                cmd.linear.x = 0.0 # Stop
                
            # Phase 3: Retry
            else:
                self.get_logger().info("Recovery Done. Retrying Navigation...")
                if self.last_nav_target == 'A':
                    self.start_nav_to_a()
                elif self.last_nav_target == 'B':
                    self.start_nav_to_b()
                else:
                    self.get_logger().error("Lost track of target. Going Idle.")
                    self.current_state = STATE_IDLE
                return # Exit to avoid publishing velocity this cycle

        # === PHASE 1: SEARCH & APPROACH CUBE ===
        elif self.current_state == STATE_SEARCH_A:
            self.stable_stop_count = 0
            if self.target_found:
                self.current_state = STATE_APPROACH_A
            else:
                cmd.angular.z = 0.2 

        elif self.current_state == STATE_APPROACH_A:
            if not self.target_found:
                self.current_state = STATE_SEARCH_A
                return
            
            current_turn_kp = self.get_turn_kp(self.tof_dist, self.stop_distance_cube)
            cmd.angular.z = self.error_x * current_turn_kp

            if abs(self.error_x) < 20: 
                if self.tof_dist <= self.stop_distance_cube:
                    self.stable_stop_count += 1
                else:
                    self.stable_stop_count = 0
                    cmd.linear.x = self.get_approach_speed(self.tof_dist, self.stop_distance_cube)

                if self.stable_stop_count >= self.required_stop_counts:
                    self.get_logger().info("STABLE TARGET REACHED (Pick)")
                    self.cmd_pub.publish(Twist()) 
                    self.current_state = STATE_PICK_ACTION 
                    self.stable_stop_count = 0 

        # === SMART PICK LOGIC ===
        elif self.current_state == STATE_PICK_ACTION:
            self.get_logger().info(f"ACTION: PICK ATTEMPT {self.pick_attempts + 1}")
            self.send_gripper_cmd(1) # PICK
            self.state_start_time = now 
            self.current_state = STATE_PICK_WAIT

        elif self.current_state == STATE_PICK_WAIT:
            if (now - self.state_start_time) > 2.0:
                self.send_gripper_cmd(0) 
                self.current_state = STATE_PICK_VERIFY

        elif self.current_state == STATE_PICK_VERIFY:
            if self.target_found:
                self.get_logger().warn("PICK FAILED: Cube still detected!")
                self.pick_attempts += 1
                
                if self.pick_attempts < self.max_pick_attempts:
                    self.get_logger().info("RETRYING: Opening Gripper & Moving Backward...")
                    self.send_gripper_cmd(3) # RESET/OPEN
                    self.state_start_time = now
                    self.current_state = STATE_BACKING_UP
                else:
                    self.get_logger().error("MAX RETRIES REACHED. Moving on.")
                    self.set_vision_mode(0) 
                    self.start_nav_to_b() 
            else:
                self.get_logger().info("PICK SUCCESS.")
                self.set_vision_mode(0) 
                self.start_nav_to_b()

        elif self.current_state == STATE_BACKING_UP:
            if (now - self.state_start_time) < 1.5:
                cmd.linear.x = -0.15 
            else:
                self.cmd_pub.publish(Twist()) 
                self.send_gripper_cmd(0) 
                self.get_logger().info("Restarting Search...")
                self.current_state = STATE_SEARCH_A

        # === PHASE 2: PLACE ON PLATFORM ===
        elif self.current_state == STATE_NAV_B:
            pass 

        elif self.current_state == STATE_SEARCH_B:
            if self.target_found:
                self.current_state = STATE_APPROACH_B
            else:
                cmd.angular.z = -0.2 

        elif self.current_state == STATE_APPROACH_B:
            if not self.target_found:
                self.current_state = STATE_SEARCH_B
                return
            
            current_turn_kp = self.get_turn_kp(self.tof_dist, self.stop_distance_platform)
            cmd.angular.z = self.error_x * current_turn_kp

            if abs(self.error_x) < 20:
                if self.tof_dist <= self.stop_distance_platform:
                    self.stable_stop_count += 1
                else:
                    self.stable_stop_count = 0 
                    cmd.linear.x = self.get_approach_speed(self.tof_dist, self.stop_distance_platform)

                if self.stable_stop_count >= self.required_stop_counts:
                    self.get_logger().info("STABLE TARGET REACHED (Place)")
                    self.cmd_pub.publish(Twist()) 
                    self.current_state = STATE_PLACE
                    self.stable_stop_count = 0

        elif self.current_state == STATE_PLACE:
            self.get_logger().info("ACTION: PLACING CUBE")
            self.set_vision_mode(0) 
            self.send_gripper_cmd(2) # PLACE
            time.sleep(0.1)
            self.send_gripper_cmd(0)

            time.sleep(3.0) 
            
            self.current_loop_count += 1
            self.get_logger().info(f"Mission Loop {self.current_loop_count}/{self.max_mission_loops} Complete.")

            if self.current_loop_count < self.max_mission_loops:
                self.get_logger().info("Restarting Sequence...")
                self.start_sequence() 
            else:
                self.get_logger().info("ALL MISSIONS COMPLETE. STOPPING.")
                self.current_state = STATE_IDLE

        # --- IMPORTANT: Publish Velocity ---
        # Added STATE_RECOVERY_WAIT to this list so it actually moves!
        if self.current_state in [STATE_SEARCH_A, STATE_APPROACH_A, STATE_BACKING_UP, STATE_SEARCH_B, STATE_APPROACH_B, STATE_RECOVERY_WAIT]:
            self.cmd_pub.publish(cmd)

    # Cleaned up Navigation Starters
    def start_nav_to_a(self):
        self.send_nav_goal(self.pos_a_x, self.pos_a_y, 0.9225, 0.3860)
        self.current_state = STATE_NAV_A

    def start_nav_to_b(self):
        self.get_logger().info("Moving to Point B...")
        self.send_nav_goal(self.pos_b_x, self.pos_b_y, 0.6208, 0.7840)
        self.current_state = STATE_NAV_B

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()