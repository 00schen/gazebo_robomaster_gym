"""Port of the Chipmunk tank demo. Showcase a topdown tank driving towards the
mouse, and hitting obstacles on the way.
"""

import random
from math import *

import pygame
from pygame.locals import *

import pymunk
from pymunk.vec2d import Vec2d
import pymunk.pygame_util

from gym_env import *

RATIO = .10
SCREEN = (int(8100*RATIO), int(5100*RATIO))

class PymunkEnv(RobomasterEnv):
    def __init__(self):
        super().__init__()
        space = pymunk.Space()
        space.iterations = 10
        space.sleep_time_threshold = 0.5

        static_body = space.static_body
        
        # Create segments around the edge of the screen.
        shape = pymunk.Segment(static_body, (1,1), (1,SCREEN[1]), 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10

        shape = pymunk.Segment(static_body, (SCREEN[0],1), SCREEN, 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10
        
        shape = pymunk.Segment(static_body, (1,1), (SCREEN[0],1), 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10
        
        shape = pymunk.Segment(static_body, (1,SCREEN[0]), SCREEN, 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10

        for i in range(0,len(self.segments),4):
            vertices = [(int(s[0]*1000*RATIO), int(s[1]*1000*RATIO)) for s in self.segments[i:i+4]]
            shape = pymunk.Poly(static_body, vertices)
            shape.friction = 10
            shape.color = (0,255,255,255)
            space.add(shape)

        self._tank_bodies = [generate_tank(space,(500,500)), generate_tank(space,(500,4600)), generate_tank(space,(7600,500)), generate_tank(space,(7600,4600))]
        self.space = space

    def generate_tank(space, center):
        # We joint the tank to the control body and control the tank indirectly by modifying the control body.
        tank_control_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        tank_control_body.position = 320, 240
        space.add(tank_control_body)
        tank_body = pymunk.Body()
        space.add(tank_body)
        shape = pymunk.Poly.create_box(tank_body, (550*RATIO, 420*RATIO), 0.0)
        shape.mass = 1
        shape.friction = 10
        space.add(shape)
        tank_body.position = int(center[0]*RATIO),int(center[1]*RATIO)
        for s in tank_body.shapes:
            s.color = (0,255,100,255)

        pivot = pymunk.PivotJoint(tank_control_body, tank_body, (0,0), (0,0))
        space.add(pivot)
        pivot.max_bias = 0 # disable joint correction
        pivot.max_force = 10000 # emulate linear friction
        
        gear = pymunk.GearJoint(tank_control_body, tank_body, 0.0, 1.0)
        space.add(gear)    
        gear.error_bias = 0 # attempt to fully correct the joint each step
        gear.max_bias = 1.2  # but limit it's angular correction rate
        gear.max_force = 50000 # emulate angular friction
        return (tank_body, tank_control_body)

    def step(self, action1, action2):
        super.step(action1, action2)
        


    def update(self, dt, surface):
        tank_body = self.tank_body
        tank_control_body = self.tank_control_body
        self._odom_info = [tuple(list(tank_body.position)+[0,tank_body.angle]) for _ in range(4)]
        self.update_robot_coords()
        space = self.space
        
        mouse_delta = Vec2d(0,0) # mouse_delta exact length does not matter
        pressed = pygame.key.get_pressed()
        if pressed[pygame.K_a]:
            mouse_delta = Vec2d(-1,0)
        if pressed[pygame.K_q]:
            mouse_delta = Vec2d(-1,1)
        if pressed[pygame.K_w]:
            mouse_delta = Vec2d(0,1)
        if pressed[pygame.K_e]:
            mouse_delta = Vec2d(1,1)
        if pressed[pygame.K_d]:
            mouse_delta = Vec2d(1,0)
        if pressed[pygame.K_x]:
            mouse_delta = Vec2d(1,-1)
        if pressed[pygame.K_s]:
            mouse_delta = Vec2d(0,-1)
        if pressed[pygame.K_z]:
            mouse_delta = Vec2d(-1,-1)

        if mouse_delta.get_length_sqrd() > 0:
            if abs((tank_body.angle-mouse_delta.angle)%pi) < abs((tank_body.angle-(mouse_delta.angle+pi)%pi)):
                tank_control_body.angle = mouse_delta.angle
                active_rotation_vector = tank_body.rotation_vector
            else:
                tank_control_body.angle = (mouse_delta.angle+pi)%pi
                active_rotation_vector = tank_body.rotation_vector.cpvrotate(Vec2d(-1,0))

            if mouse_delta.dot(active_rotation_vector) > 0.0:
                direction = 1.0 
            else:
                direction = -1.0
            dv = Vec2d(30.0*direction, 0.0)
            tank_control_body.velocity = active_rotation_vector.cpvrotate(dv)
        else:
            tank_control_body.angle = tank_body.angle
            tank_control_body.velocity = 0,0
        
        space.step(dt)


env = PymunkEnv()
pygame.init()
screen = pygame.display.set_mode(SCREEN) 
clock = pygame.time.Clock()
draw_options = pymunk.pygame_util.DrawOptions(screen)


font = pygame.font.Font(None, 24)
text = "Use the mouse to drive the tank, it will follow the cursor."
text = font.render(text, 1, pygame.color.THECOLORS["white"])

while True:
    for event in pygame.event.get():
        if event.type == QUIT or pygame.key.get_pressed()[K_ESCAPE]: 
            exit()
    
    screen.fill(pygame.color.THECOLORS["black"])
    env.space.debug_draw(draw_options)
    screen.blit(text, (15,15))
    fps = 60.0
    env.update(1/fps, screen)
    pygame.display.flip()
    
    clock.tick(fps)