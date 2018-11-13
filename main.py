#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
An attempt at some simple, self-contained pygame-based examples.

Example 01

In short:
One static body: a big polygon to represent the ground
One dynamic body: a rotated big polygon
And some drawing code to get you going.

kne
"""
import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE)

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)
from Box2D import (b2BodyDef, b2CircleShape, b2Color, b2EdgeShape,
                   b2FixtureDef, b2PolygonShape, b2RayCastCallback, b2Vec2,
                   b2_dynamicBody, b2_pi)

from top_down import (TDCar, TDTire)
from goal import (Goal)

# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 1.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 720

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, 0), doSleep=True)

goals = [Goal(world, "goal 1", 300, 300)]

goalsBodies = []
for goal in goals:
    goalsBodies.append(goal.body)

horizontal_ground_body_fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(1280, 5)),
                userData = "wall",
            )

vertical_ground_body_fixDef = b2FixtureDef(
                shape=b2PolygonShape(box=(5, 720)),
                userData = "wall",
            )
# And a static body to hold the ground shape
h1_ground_body = world.CreateStaticBody(
    position=(0, 0),
    fixtures=horizontal_ground_body_fixDef,
)
h2_ground_body = world.CreateStaticBody(
    position=(0, 715),
    fixtures=horizontal_ground_body_fixDef,
)
v1_ground_body = world.CreateStaticBody(
    position=(0, 0),
    fixtures=vertical_ground_body_fixDef,
)
v2_ground_body = world.CreateStaticBody(
    position=(1275, 0),
    fixtures=vertical_ground_body_fixDef,
)

# car = Car(world, 15, 15, 15)
car = TDCar(world)
laserPoints = []
laserImpactPoints = []

rightOrLeft = 0
backwardOrForward = 0

# And add a box fixture onto it (with a nonzero density, so it will move)
# box = dynamic_body.CreatePolygonFixture(box=(2, 1), density=1, friction=0.3, userData="wall")

colors = {
    staticBody: (0, 0, 255, 255),
    dynamicBody: (127, 127, 127, 255),
}

# --- main game loop ---
running = True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if (event.type == KEYDOWN):
            print("key down", event.key)
            if (event.key == 97): #left
                rightOrLeft = 1
            if (event.key == 100): #right
                rightOrLeft = -1
            if (event.key == 119): #forward
                backwardOrForward = 1
            if (event.key == 115): #backward
                backwardOrForward = -1
        if (event.type == KEYUP):
            print("key up", event.key)
            if (event.key == 97): #left
                rightOrLeft = 0
            if (event.key == 100): #right
                rightOrLeft = 0
            if (event.key == 119): #forward
                backwardOrForward = 0
            if (event.key == 115): #backward
                backwardOrForward = 0
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    screen.fill((0, 0, 0, 0))
    # Draw the world
    for body in ([h1_ground_body, h2_ground_body, v1_ground_body, v2_ground_body] + car.getBodies() + goalsBodies):  # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            # The fixture holds information like density and friction,
            # and also the shape.
            shape = fixture.shape

            # Naively assume that this is a polygon shape. (not good normally!)
            # We take the body's transform and multiply it with each
            # vertex, and then convert from meters to pixels with the scale
            # factor.
            vertices = [(body.transform * v) * PPM for v in shape.vertices]

            # But wait! It's upside-down! Pygame and Box2D orient their
            # axes in different ways. Box2D is just like how you learned
            # in high school, with positive x and y directions going
            # right and up. Pygame, on the other hand, increases in the
            # right and downward directions. This means we must flip
            # the y components.
            # vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]

            color = colors[body.type]

            if (fixture.userData == "wheel front"):
                color = (255, 0, 0, 255)
            if (fixture.userData == "wheel rear"):
                color = (0, 0, 255, 255)
            if (type(fixture.userData) is str and len(fixture.userData) >= 4 and fixture.userData[:4] == "goal"):
                color = (0, 150, 150, 255)

            pygame.draw.polygon(screen, color, vertices)

    for point in laserPoints:
        color = (0, 255, 0, 255)
        carPosScaled = b2Vec2(car.body.worldCenter.x * PPM, car.body.worldCenter.y * PPM)
        pointScaled = b2Vec2(point.x * PPM, point.y * PPM)
        pygame.draw.line(screen, color, carPosScaled, pointScaled)

    for point in laserImpactPoints:
        color = (0, 255, 0, 255)
        pointScaled = b2Vec2(point.x * PPM, point.y * PPM)
        # print("laserImpactPoints point", point, pointScaled)
        pygame.draw.circle(screen, color, (int(pointScaled.x), int(pointScaled.y)), 2, 0)

    # Make Box2D simulate the physics of our world for one step.
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    # See the manual (Section "Simulating the World") for further discussion
    # on these parameters and their implications.
    # car.update(-1,1)
    car.update(backwardOrForward, rightOrLeft)
    laserPoints = car.sensors.laserPoints
    laserImpactPoints = car.sensors.laserImpactPoints
    world.Step(TIME_STEP, 10, 10)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
