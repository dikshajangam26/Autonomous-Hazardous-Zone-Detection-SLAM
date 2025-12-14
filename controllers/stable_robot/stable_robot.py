# Copyright 1996-2023 Cyberbotics Ltd.
# Final Controller: Locked Heading + Robust CSV Logging (Crash Fix)

from controller import Supervisor
import math
import numpy as np
import csv
import os 
import subprocess 

# PART 1: OPTIMIZED MAPPER
class OccupancyGridMapper:
    def __init__(self, map_size_m, resolution_m, max_range_m):
        self.RESOLUTION = resolution_m
        self.SIZE_M = map_size_m
        self.SIZE_PIXELS = int(self.SIZE_M / self.RESOLUTION)
        self.CENTER_PIXEL = self.SIZE_PIXELS // 2
        self.MAX_RANGE = max_range_m 
        
        self.grid = np.full((self.SIZE_PIXELS, self.SIZE_PIXELS), 0.5, dtype=np.float32)

    def world_to_map(self, x_w, y_w):
        x_p = int(x_w / self.RESOLUTION)
        y_p = int(y_w / self.RESOLUTION)
        map_x = x_p + self.CENTER_PIXEL
        map_y = self.SIZE_PIXELS - (y_p + self.CENTER_PIXEL)
        
        if 0 <= map_x < self.SIZE_PIXELS and 0 <= map_y < self.SIZE_PIXELS:
            return map_x, map_y
        return None, None

    def log_odds_update(self, map_x, map_y, is_occupied):
        if map_x is None: return
        current_p = self.grid[map_y, map_x]
        
        if is_occupied: 
            self.grid[map_y, map_x] = min(1.0, current_p + 0.3) 
        else:           
            if current_p > 0.9: return 
            self.grid[map_y, map_x] = max(0.0, current_p - 0.05)

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return points

    def update_map(self, x_robot, y_robot, theta_robot, lidar_data):
        num_rays = len(lidar_data)
        if num_rays == 0: return
        
        step = 3 
        lidar_angle_min = -math.pi / 2 
        angle_increment = math.pi / num_rays 
        
        rob_x, rob_y = self.world_to_map(x_robot, y_robot)

        for i in range(0, num_rays, step):
            distance = lidar_data[i]
            if distance > self.MAX_RANGE: 
                real_distance = self.MAX_RANGE; is_hit = False
            else:
                real_distance = distance; is_hit = True

            ray_angle = theta_robot + (lidar_angle_min + i * angle_increment)
            
            x_wall = x_robot + real_distance * math.cos(ray_angle)
            y_wall = y_robot + real_distance * math.sin(ray_angle)
            wall_x, wall_y = self.world_to_map(x_wall, y_wall)
            
            free_dist = min(real_distance, 4.0) 
            x_free = x_robot + free_dist * math.cos(ray_angle)
            y_free = y_robot + free_dist * math.sin(ray_angle)
            free_end_x, free_end_y = self.world_to_map(x_free, y_free)

            if rob_x is not None and free_end_x is not None:
                line_points = self.bresenham_line(rob_x, rob_y, free_end_x, free_end_y)
                for (px, py) in line_points:
                    self.log_odds_update(px, py, is_occupied=False)

            if is_hit and wall_x is not None:
                self.log_odds_update(wall_x, wall_y, is_occupied=True)

    def visualize_map(self, display_device):
        output = np.full((self.SIZE_PIXELS, self.SIZE_PIXELS, 4), 128, dtype=np.uint8)
        output[:, :, 3] = 255 
        output[self.grid > 0.65, 0:3] = 0   
        output[self.grid < 0.35, 0:3] = 255
        ir = display_device.imageNew(output.tobytes(), display_device.RGBA, self.SIZE_PIXELS, self.SIZE_PIXELS)
        display_device.imagePaste(ir, 0, 0, False)
        display_device.imageDelete(ir)


# ==============================================================================
# PART 2: ROBOT CONTROLLER
# ==============================================================================
class KinematicRobot(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.robot_node = self.getSelf()
        self.SPEED = 0.3 
        
        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")
        
        start_pos = self.trans_field.getSFVec3f()
        start_rot = self.rot_field.getSFRotation()
        self.current_x = start_pos[0]
        self.current_y = start_pos[1]
        self.current_theta = start_rot[3] 
        self.LOCKED_Z = 0.11 
        
        self.lidar = self.getDevice("lidar")
        self.lidar.enable(self.time_step)
        self.lidar.enablePointCloud()
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        
        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(self.time_step)
        
        self.map_display = self.getDevice("map_display") 
        
        self.mapper = OccupancyGridMapper(
            map_size_m=50.0, 
            resolution_m=0.1, 
            max_range_m=self.lidar.getMaxRange()
        )

        self.state = "FORWARD"
        self.timer = 0
        self.state_timer = 0 
        self.turn_direction = 1
        self.step_count = 0 

        # --- CSV LOGGING SETUP (CRASH PROOF) ---
        self.csv_writer = None # 1. Initialize to None first!
        self.csv_file = None

        try:
            print(f"CSV Logging to: {os.getcwd()}/robot_data.csv")
            self.csv_file = open('robot_data.csv', 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            
            self.csv_writer.writerow([
                'Time_s', 'Pos_X', 'Pos_Y', 'Theta_rad', 
                'State', 'Worker_Det', 
                'Lidar_Front', 'Lidar_Left', 'Lidar_Right'
            ])
            self.csv_file.flush()
            print("CSV Header Written Successfully.")
            
            # Auto-launch Plotter
            print("Attempting to launch plotter...")
            subprocess.Popen(['python', 'plot_results.py'])
            
        except PermissionError:
             print("❌ ERROR: 'robot_data.csv' is open in Excel! Please close it.")
        except Exception as e:
            print(f"❌ CSV Setup Error: {e}")

    def detect_worker(self):
        image = self.camera.getImage()
        if image is None: return False
        
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        img_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        center_y = height // 2; center_x = width // 2
        crop = img_array[center_y-10:center_y+10, center_x-10:center_x+10]
        
        avg_r = np.mean(crop[:,:,2])
        avg_g = np.mean(crop[:,:,1])
        avg_b = np.mean(crop[:,:,0])
        
        if avg_r > 150 and avg_g > 150 and avg_b < 100:
            return True
        return False

    def run(self):
        print("Final Controller: Timed Emergency Fix")
        self.imu_offset = None  
        self.is_worker_detected = False
        
        # Force alignment at start
        self.current_theta = 0.0 
        
        while self.step(self.time_step) != -1:
            dt = self.time_step / 1000.0
            self.timer += dt
            self.state_timer += dt 
            self.step_count += 1
            
            # --- 1. SENSOR FUSION ---
            imu_rpy = self.imu.getRollPitchYaw()
            raw_theta = imu_rpy[2] 
            if self.imu_offset is None: self.imu_offset = raw_theta
            fused_theta = raw_theta - self.imu_offset

            # --- 2. VISION ---
            if self.step_count % 10 == 0:
                 self.is_worker_detected = self.detect_worker()

            # --- 3. LIDAR CLEANING ---
            raw_lidar_data = self.lidar.getRangeImage()
            cleaned_data = []
            for dist in raw_lidar_data:
                if float(dist) == float('inf') or dist > 20.0:
                    cleaned_data.append(20.0)
                elif dist < 0.15: 
                    cleaned_data.append(20.0) 
                else:
                    cleaned_data.append(dist)
            
            if len(cleaned_data) > 0:
                third = len(cleaned_data) // 3
                min_right = min(cleaned_data[0 : third])
                min_front = min(cleaned_data[third : 2*third])
                min_left  = min(cleaned_data[2*third : len(cleaned_data)])
            else:
                min_right, min_front, min_left = 20.0, 20.0, 20.0

            # --- 4. LOGIC ---
            linear_v = 0
            angular_v = 0
            
            # A. WORKER SAFETY
            if self.state == "FORWARD" and self.is_worker_detected and min_front < 3.0:
                self.state = "WORKER_WAIT"
                self.state_timer = 0
                print("⚠️ WORKER DETECTED")
            
            elif self.state == "WORKER_WAIT":
                linear_v = 0.0; angular_v = 0.0
                if self.state_timer > 5.0:
                    if self.is_worker_detected and min_front < 3.5:
                        self.state_timer = 0 
                    else:
                        self.state = "FORWARD"
            
            # B. EMERGENCY TRIGGER (Only if NOT already in Emergency)
            elif self.state != "EMERGENCY" and (min_front < 0.20 or min_left < 0.20 or min_right < 0.20):
                 self.state = "EMERGENCY"
                 self.state_timer = 0 # Reset timer on entry
                 print("⚠️ EMERGENCY BACKUP TRIGGERED")

            # C. EMERGENCY EXECUTION (The Fix)
            elif self.state == "EMERGENCY":
                 linear_v = -0.15 # Back up slowly
                 angular_v = 0.0
                 
                 # Exit Condition: Back up for 1.5 seconds OR until plenty of space
                 if self.state_timer > 1.5 or min_front > 0.6:
                     print("Clear of emergency. Turning...")
                     self.state = "U_TURN" 
                     self.state_timer = 0

            # D. NAVIGATION
            elif self.state == "FORWARD":
                if min_front < 0.4:
                    self.state = "U_TURN"
                    self.state_timer = 0
                    if min_left < min_right: self.turn_direction = -1 
                    else:                    self.turn_direction = 1  
                else:
                    linear_v = self.SPEED
                    angular_v = 0.0 

            # E. U-TURN
            # D. TURN (90 Degrees)
            elif self.state == "U_TURN":
                linear_v = 0.0
                angular_v = self.turn_direction * 1.5 
           
                # 1.05 = 90 degrees
                # 0.5  = 45 degrees
                if self.state_timer > 1.05: 
                    self.state = "FORWARD"
            
            # --- 5. UPDATES ---
            self.current_theta += angular_v * dt
            if self.current_theta > math.pi: self.current_theta -= 2*math.pi
            if self.current_theta < -math.pi: self.current_theta += 2*math.pi
            
            self.current_x += linear_v * math.cos(self.current_theta) * dt
            self.current_y += linear_v * math.sin(self.current_theta) * dt

            self.mapper.update_map(self.current_x, self.current_y, self.current_theta, raw_lidar_data)
            
            if self.step_count % 20 == 0: 
                 self.mapper.visualize_map(self.map_display)

            self.trans_field.setSFVec3f([self.current_x, self.current_y, self.LOCKED_Z])
            self.rot_field.setSFRotation([0, 0, 1, self.current_theta])

            # CSV LOGGING
            if self.step_count % 10 == 0 and self.csv_writer is not None:
                try:
                    self.csv_writer.writerow([
                        f"{self.timer:.2f}",
                        f"{self.current_x:.3f}",
                        f"{self.current_y:.3f}",
                        f"{self.current_theta:.3f}",
                        self.state,
                        self.is_worker_detected,
                        f"{min_front:.2f}",
                        f"{min_left:.2f}",
                        f"{min_right:.2f}"
                    ])
                    self.csv_file.flush()
                except Exception: pass
# --- EXECUTION ---
controller = KinematicRobot()
controller.run()