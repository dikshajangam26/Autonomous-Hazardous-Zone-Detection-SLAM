# Copyright 1996-2020 Cyberbotics Ltd.
# Licensed under the Apache License, Version 2.0
from controller import Supervisor
import math
import random

class Pedestrian(Supervisor):
    def __init__(self):
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        
        # --- SETTINGS ---
        self.ROOT_HEIGHT = 1.27
        self.speed = 0.8
        self.CYCLE_TO_DISTANCE_RATIO = 0.05
        self.TIME_TO_WALK = 4.0 
        self.OBSTACLE_THRESHOLD = 600 
        
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]
        ]
        Supervisor.__init__(self)
        self.animation_timer = 0.0
        self.walk_timer = 0.0
        
        # Counters
        self.turns_taken = 0
        self.target_turns_before_back = random.randint(4, 5)

    def run(self):
        self.time_step = int(self.getBasicTimeStep())
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        
        # SETUP SENSORS
        self.ds_left = self.getDevice("ds_left")
        self.ds_right = self.getDevice("ds_right")
        
        if self.ds_left: self.ds_left.enable(self.time_step)
        if self.ds_right: self.ds_right.enable(self.time_step)
        
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # --- NEW: RANDOM START DELAY ---
        # This ensures copied robots don't move in perfect sync
        start_delay = random.uniform(0, 2.0) # 0 to 2 seconds delay
        start_counter = 0
        while self.step(self.time_step) != -1:
            start_counter += (self.time_step / 1000.0)
            if start_counter > start_delay:
                break

        current_angle = 0.0

        while self.step(self.time_step) != -1:
            self.root_node_ref.resetPhysics()
            dt = self.time_step / 1000.0
            self.walk_timer += dt
            
            # 1. SENSOR READINGS
            val_left = self.ds_left.getValue() if self.ds_left else 0
            val_right = self.ds_right.getValue() if self.ds_right else 0
            
            hit_left = val_left > self.OBSTACLE_THRESHOLD
            hit_right = val_right > self.OBSTACLE_THRESHOLD
            is_time_up = self.walk_timer > self.TIME_TO_WALK
            
            # 2. COLLISION LOGIC
            if hit_left or hit_right:
                if hit_left and hit_right:
                    # Blocked front -> Backwards
                    current_angle += 3.14 + random.uniform(-0.2, 0.2)
                elif hit_left:
                    # Blocked Left -> Turn Right
                    current_angle -= 1.57 + random.uniform(0, 0.5)
                elif hit_right:
                    # Blocked Right -> Turn Left
                    current_angle += 1.57 + random.uniform(0, 0.5)
                self.walk_timer = 0 
                
            # 3. EXPLORATION LOGIC
            elif is_time_up:
                self.turns_taken += 1
                if self.turns_taken >= self.target_turns_before_back:
                    # Turn Backwards
                    current_angle += 3.14 + random.uniform(-0.3, 0.3)
                    self.turns_taken = 0
                    self.target_turns_before_back = random.randint(4, 5)
                else:
                    # Random Turn
                    current_angle = random.uniform(-3.14, 3.14)
                self.walk_timer = 0 

            current_angle = current_angle % 6.28
            self.root_rotation_field.setSFRotation([0, 0, 1, current_angle])
            
            # 4. MOVE
            move_dist = self.speed * dt
            dx = move_dist * math.cos(current_angle)
            dy = move_dist * math.sin(current_angle)
            
            current_pos = self.root_translation_field.getSFVec3f()
            new_x = current_pos[0] + dx
            new_y = current_pos[1] + dy

            # 5. ANIMATE
            self.animation_timer += dt
            ratio = (self.animation_timer * self.speed) / self.CYCLE_TO_DISTANCE_RATIO
            current_sequence = int(ratio % self.WALK_SEQUENCES_NUMBER)
            ratio = ratio - int(ratio) 

            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
            
            self.root_translation_field.setSFVec3f([new_x, new_y, self.ROOT_HEIGHT + self.current_height_offset])

            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle_limb = self.angles[i][current_sequence] * (1 - ratio) + \
                    self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle_limb)

controller = Pedestrian()
controller.run()