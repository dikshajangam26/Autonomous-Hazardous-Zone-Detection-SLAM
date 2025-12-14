# Copyright 1996-2020 Cyberbotics Ltd.
# Licensed under the Apache License, Version 2.0
from controller import Supervisor
import math
import random

class ForkliftDriver(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        
        # --- SETTINGS ---
        self.SPEED = 1.0
        self.PATROL_TIME = 5.0 
        
        # STOP DISTANCE (Higher = Stop further away)
        # With our table (0m=1000, 5m=0), 500 is about 2.5 meters away.
        self.SAFETY_THRESHOLD = 500 
        
        self.robot_node = self.getSelf()
        if self.robot_node is None:
            print("Error: Could not find self node.")
            return

        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")
        
        # 1. CAPTURE STARTING POSITION
        start_pos = self.trans_field.getSFVec3f()
        self.current_x = start_pos[0]
        self.current_y = start_pos[1]
        self.fixed_z = start_pos[2]
        
        # 2. CAPTURE STARTING ROTATION
        start_rot = self.rot_field.getSFRotation()
        self.current_angle = start_rot[3] # Start facing the way you placed it
        
        self.timer = 0
        self.PATROL_TIME += random.uniform(-1.0, 1.0)
        
        # --- 3. SETUP SENSOR ---
        self.ds = self.getDevice("forklift_ds")
        if self.ds:
            self.ds.enable(self.time_step)
        else:
            print("Warning: 'forklift_ds' not found on this robot.")

    def run(self):
        if self.robot_node is None: return

        # Random Start Delay
        start_delay = random.uniform(0, 2.0)
        wait_timer = 0
        while self.step(self.time_step) != -1:
            wait_timer += self.time_step / 1000.0
            if wait_timer > start_delay:
                break

        while self.step(self.time_step) != -1:
            dt = self.time_step / 1000.0
            
            # --- 1. SAFETY CHECK ---
            # If sensor sees something close, PAUSE everything.
            sensor_value = 0
            if self.ds:
                sensor_value = self.ds.getValue()
            
            if sensor_value > self.SAFETY_THRESHOLD:
                # Just skip the rest of the loop. The robot effectively freezes.
                # It will check again next frame.
                continue 

            # --- 2. NORMAL MOVEMENT ---
            self.timer += dt
            
            # Turn Around Logic
            if self.timer > self.PATROL_TIME:
                self.current_angle += 3.14159 
                self.current_angle += random.uniform(-0.05, 0.05)
                self.timer = 0
            
            # Apply Rotation
            self.rot_field.setSFRotation([0, 0, 1, self.current_angle])
            
            # Apply Position
            move_dist = self.SPEED * dt
            dx = move_dist * math.cos(self.current_angle)
            dy = move_dist * math.sin(self.current_angle)
            
            self.current_x += dx
            self.current_y += dy
            
            self.trans_field.setSFVec3f([self.current_x, self.current_y, self.fixed_z])

controller = ForkliftDriver()
controller.run()