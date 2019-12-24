#! /usr/bin/env python
import math
import time

from utils import *

class RobomasterEnv:
    def __init__(self):
        #Set up state parameters
        self._odom_info = [[None]] * 4 
        self._gimbal_angle_info = [[None]] * 4
        self._robot_hp = [[2000], [2000], [2000], [2000]]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]
        self._launch_velocity_max = 25
        self._shoot = [0, 0, 0, 0]
        
        #Amount of time running per timestep
        self._running_step = 0.1
        #Max number of timesteps
        self._timestep = 0
        self._max_timesteps = 1000 
        #Conversion from timestep to real time
        self._real_time_conversion = 1

        #Robot parameters
        def diag_stats_helper(l, w):
            angle = math.atan(w / l)
            return angle, w / math.sin(angle) / 2
        
        self.robot_length, self.robot_width = 0.550, 0.420
        self.robot_diag_angle, self.robot_diag_length = diag_stats_helper(self.robot_length, self.robot_width)

        self.armor_size = 0.131
        self.armor_thickness = 0.015
        self.forward_frame_length = self.robot_length + self.armor_thickness * 2
        self.forward_frame_diag_angle, self.forward_frame_diag_length = diag_stats_helper(self.forward_frame_length, self.armor_size)
        
        self.sideway_frame_width = self.robot_width + self.armor_thickness * 2
        self.sideway_frame_diag_angle, self.sideway_frame_diag_length = diag_stats_helper(self.armor_size, self.sideway_frame_width)

        self._armor_plate_damage = [20, 40, 40, 60]

        ################################### Exportables to Pymunk ########################################
        self.gimbal_angle_range = 82.5 / 180 * math.pi
        self.gimbal_shoot_range = float('inf') # more on this later

        self.robot_coords = [None] * 4
        self.robot_plates_coords = [None] * 4     

        #Gives coordinates for the obstacles:
        #x-start, x-end, y-start, y-end
        self.parallel_obstacles = [
            (1.500, 1.750, 0, 1.000),
            (3.600, 4.600, 1.000, 1.250),
            (7.100, 8.100, 1.000, 1.250),
            (1.500, 2.300, 2.425, 2.675),
            (5.800, 6.600, 2.425, 2.675),
            (0.000, 1.000, 3.850, 4.100),
            (3.500, 4.500, 3.850, 4.100),
            (6.350, 6.600, 4.100, 5.100),
        ]
        #points [top, vmid, bot, left, hmid, right]
        diagonal_len = .300 / math.sqrt(2)
        self.center_obstacle = (4.050 - diagonal_len, 4.050, 4.050 + diagonal_len, 2.550 - diagonal_len, 2.550, 2.550 + diagonal_len)

        # initialize segments of each obstacle for blocking calculation
        # buffer is added
        segments = []
        buffer = 0.015
        for xl, xr, yb, yt in self.parallel_obstacles:
            xl, xr, yb, yt = xl - buffer, xr + buffer, yb - buffer, yt + buffer
            segments.extend([(xl, yb, xl, yt), (xr, yb, xl, yb), (xr, yt, xr, yb), (xl, yt, xr, yt)])
        left, vmid, right, bot, hmid, top = self.center_obstacle
        left, right, top, bot = left-buffer, right+buffer, top+buffer, bot-buffer
        segments.extend([(left, hmid, vmid, bot), (vmid, bot, right, hmid), (right, hmid, vmid, top), (vmid, top, left, hmid)])
        self.segments = segments

        self._zones = [
        [3.830, 4.370, 0.270, 0.750],
        [1.630, 2.170, 1.695, 2.175],
        [0.230, 0.770, 3.120, 3.600],
        [4.350, 4.830, 4.500, 5.040], 
        [5.330, 7.870, 1.500, 1.980],
        [5.930, 6.470, 2.925, 3.405]]
        #####################################################################################################
        self._zones_active = [False] * 6

        #Zone Types:
        #0: refill
        #1: hp recovery
        #2: no movement debuff
        #3: no shoot debuff
        self._zone_types = [0, 3, 2]

        #Effects per robot
        self._robot_effects = [dict(), dict(), dict(), dict()]
        

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z 

    def check_collision_with_obstacle(robot1, robot2):
        pass

    def robot_coords_from_odom(self, odom_info):
        if not odom_info:
            return
        x, y, z, yaw = odom_info
        coords = []
        for angle in (yaw + self.robot_diag_angle, yaw + math.pi - self.robot_diag_angle, yaw + math.pi + self.robot_diag_angle, yaw - self.robot_diag_angle):
            xoff, yoff = math.cos(angle) * self.robot_diag_length, math.sin(angle) * self.robot_diag_length
            coords.extend([x + xoff, y + yoff])
        return coords

    def get_enemy_team(self, robot_index):
        if robot_index < 2:
            return (2, 3)
        return (0, 1)

    def plate_coords_from_odom(self, odom_info):
        if not odom_info:
            return
        x, y, z, yaw = odom_info
        frontx1off, fronty1off = math.cos(yaw + self.forward_frame_diag_angle) * self.forward_frame_diag_length, math.sin(yaw + self.forward_frame_diag_angle) * self.forward_frame_diag_length
        frontx2off, fronty2off = math.cos(yaw - self.forward_frame_diag_angle) * self.forward_frame_diag_length, math.sin(yaw - self.forward_frame_diag_angle) * self.forward_frame_diag_length
        backx1off, backy1off = math.cos(yaw + math.pi + self.forward_frame_diag_angle) * self.forward_frame_diag_length, math.sin(yaw + math.pi + self.forward_frame_diag_angle) * self.forward_frame_diag_length
        backx2off, backy2off = math.cos(yaw + math.pi - self.forward_frame_diag_angle) * self.forward_frame_diag_length, math.sin(yaw + math.pi - self.forward_frame_diag_angle) * self.forward_frame_diag_length
        leftx1off, lefty1off = math.cos(yaw + self.sideway_frame_diag_angle) * self.sideway_frame_diag_length, math.sin(yaw + math.pi - self.sideway_frame_diag_angle) * self.sideway_frame_diag_length
        rightx2off, righty2off  = math.cos(yaw - self.sideway_frame_diag_angle) * self.sideway_frame_diag_length, math.sin(yaw - self.sideway_frame_diag_angle) * self.sideway_frame_diag_length
        rightx1off, righty1off = math.cos(yaw + math.pi + self.sideway_frame_diag_angle) * self.sideway_frame_diag_length, math.sin(yaw + math.pi + self.sideway_frame_diag_angle) * self.sideway_frame_diag_length
        leftx2off, lefty2off = math.cos(yaw + math.pi - self.sideway_frame_diag_angle) * self.sideway_frame_diag_length, math.sin(yaw + math.pi - self.sideway_frame_diag_angle) * self.sideway_frame_diag_length
        return [(x + frontx1off, y + fronty1off, x + frontx2off, y + fronty2off),
                (x + leftx1off, y + lefty1off, x + leftx2off, y + lefty2off),
                (x + rightx1off, y + righty1off, x + rightx2off, y + righty2off),
                (x + backx1off, y + backy1off, x + backx2off, y + backy2off)]

    def update_robot_coords(self):
        self.robot_coords = [self.robot_coords_from_odom(self._odom_info[0]), self.robot_coords_from_odom(self._odom_info[1]), \
            self.robot_coords_from_odom(self._odom_info[2]), self.robot_coords_from_odom(self._odom_info[3])]
        self.robot_plates_coords = [self.plate_coords_from_odom(self._odom_info[0]), self.plate_coords_from_odom(self._odom_info[1]), \
            self.plate_coords_from_odom(self._odom_info[2]), self.plate_coords_from_odom(self._odom_info[3])]

    # pass in from_robot_index so it doesn't check for the robot itself!
    # otherwise, each robot blocks its own line-of-sight
    def is_line_of_sight_blocked(self, x1, y1, x2, y2, from_robot_index=None):
        # uninitiated
        if not self.robot_coords:
            return True
        for robot_index in range(4):
            if robot_index != from_robot_index:
                _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 = self.robot_coords[robot_index]
                if lines_cross(x1, y1, x2, y2, _x1, _y1, _x3, _y3) or lines_cross(x1, y1, x2, y2, _x2, _y2, _x4, _y4):
                    return True
        for xl, yl, xh, yh in self.segments:
            if lines_cross(x1, y1, x2, y2, xl, yl, xh, yh):
                return True
        return False

    def get_visible_enemy_plates(self, robot_index):
        enemy1, enemy2 = self.get_enemy_team(robot_index)
        visible = []
        x, y, z, yaw = self._odom_info[robot_index]
        for index in range(4):
            x1, y1, x2, y2 = self.robot_plates_coords[enemy1][index]
            if distance(x, y, x1, y1) < self.gimbal_shoot_range and distance(x, y, x2, y2) < self.gimbal_shoot_range and angleDiff(angleTo(x, y, x1, y1), yaw) <= self.gimbal_angle_range and angleDiff(angleTo(x, y, x2, y2), yaw) <= self.gimbal_angle_range:
                if not self.is_line_of_sight_blocked(x, y, x1, y1, robot_index) and not self.is_line_of_sight_blocked(x, y, x2, y2, robot_index):
                    visible.extend([(enemy1, self._armor_plate_damage[index], angleBetween(x, y, x1, y1, x2, y2))])
        for index in range(4):
            x1, y1, x2, y2 = self.robot_plates_coords[enemy2][index]
            if distance(x, y, x1, y1) < self.gimbal_shoot_range and distance(x, y, x2, y2) < self.gimbal_shoot_range and angleDiff(angleTo(x, y, x1, y1), yaw) <= self.gimbal_angle_range and angleDiff(angleTo(x, y, x2, y2), yaw) <= self.gimbal_angle_range:
                if not self.is_line_of_sight_blocked(x, y, x1, y1, robot_index) and not self.is_line_of_sight_blocked(x, y, x2, y2, robot_index):
                    visible.extend([(enemy2, self._armor_plate_damage[index], angleBetween(x, y, x1, y1, x2, y2))])
        return visible

    def _timestep_to_real_time(self):
        #converts timestep to real time
        return self._timestep * self._running_step * self._real_time_conversion 

    def _check_in_zone(self, position):
        #Input: x y z position
        #Checks if robot are in a buff/debuff zone, returns the zone
        for i in range(len(self._zones)):
            edge_buffer = 0.05 if self._zone_types[i%3] == 0 or self._zone_types[i%3] == 1 else -0.05
            if position[0] >= self._zones[i][0] + edge_buffer and position[0] <= self._zones[i][1] - edge_buffer and position[1] >= self._zones[i][2] + edge_buffer and position[1] <= self._zones[i][3] - edge_buffer:
                return i
        return None

    def step(self, action1, action2): 
        #x velocity, y velocity, twist angular, shoot

        if len(action1) != 8 or len(action2) != 8:
            print("Invalid action!")
            return None, None, None, {} 

        #Check for/update debuffs to robots if in zone
        for position in range(len(self._odom_info)):
            if self._odom_info[position][0] is not None:
                zone = self._check_in_zone(self._odom_info[position])
                if zone is None:
                    continue
                if not self._zones_active[zone]:
                     self._zones_active[zone] = True
                else:
                    continue
                if position == 0:
                    print(zone)
                    #print(zone, self._odom_info[position])
                if zone is None:
                    continue
                elif self._zone_types[zone%3] == 0 or self._zone_types[zone%3] == 1:
                    if zone < 3:
                        self._robot_effects[0][self._zone_types[zone%3]] = True
                        self._robot_effects[1][self._zone_types[zone%3]] = True
                    else:
                        self._robot_effects[2][self._zone_types[zone%3]] = True
                        self._robot_effects[3][self._zone_types[zone%3]] = True
                else:
                    self._robot_effects[position][self._zone_types[zone%3]] = 10
         
        #Apply buffs/debuffs
        for robot_number in range(len(self._robot_effects)):
            if self._robot_effects[robot_number].get(0, False):
                self._num_projectiles[robot_number] += 100
                self._robot_effects[robot_number][0] = False
            if self._robot_effects[robot_number].get(1, False):
                self._robot_hp[robot_number] = min(2000, self.robot_hp[robot_number] + 200)
                self._robot_effects[robot_number][1] = False
            if  self._robot_effects[robot_number].get(2, 0) > 0:
                if robot_number == 0:
                    action1[0] = 0
                    action1[1] = 0
                    action1[2] = 0
                elif robot_number == 1:
                    action1[4] = 0
                    action1[5] = 0
                    action1[6] = 0 
                elif robot_number == 2:
                    action2[0] = 0
                    action2[1] = 0
                    action2[2] = 0
                else:
                    action2[4] = 0
                    action2[5] = 0
                    action2[6] = 0
                self._robot_effects[robot_number][2] -= self._running_step
            if self._robot_effects[robot_number].get(3, 0) > 0:
                if robot_number == 0:
                    action1[3] = 0
                elif robot_number == 1:
                    action1[7] = 0
                elif robot_number == 2:
                    action2[3] = 0
                elif robot_number == 3:
                    action2[7] = 0
                self._robot_effects[robot_number] -= self._running_step

        #calculate state and reward
        state = self.get_state()
        reward = self.get_reward(state, action1, action2)
        done = self._timestep >= self._max_timesteps

        return state, reward, done, {}
       

    def reset(self):
        self._zones_active = [False] * 6
        self._zone_types = [0, 2, 3]
        self._robot_debuffs = [[], [], [], []]
        self._timestep = 0
        self.real_time = 0
        self._robot_hp = [[2000], [2000], [2000], [2000]]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]

        state = self.get_state()

        return state

    def get_state(self):
        #xyz position, robot angle, gimbal angle, number of projectiles, barrel heat of shooter, robot hp
        #only use barrel heat for your robots

        #Sizes of parameters
        #xyz: 3
        #robot angle: 1
        #gimbal_angle: 2
        #number_of_projectiles: 1
        #barrel heat: 1 
        #robot hp: 1

        robot_state = [list(self._odom_info[i]) + self._num_projectiles[i] + self._barrel_heat[i] + self._robot_hp[i] for i in range(4)]
        return [robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3], robot_state[0] + robot_state[1] + robot_state[2] + robot_state[3]]

    def get_reward(self, state, action1, action2):
        return 0

if __name__ == '__main__':  
    env = RobomasterEnv()
    for i in range(1000):
        state, reward, done, info = env.step([0, 1, 0, 0, 0, 0, -1, 0], [0, 0, -1, 0, 0, 0, 1, 0])
        time.sleep(0.01)
        if done:
            env.reset()