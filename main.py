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
import time
from threading import Thread
from random import uniform
import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE)

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from world import (World)

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

numPopulation = 3000
population = []

for i in range(numPopulation):
    population.append(World(PPM, TIME_STEP))

rightOrLeft = 0
backwardOrForward = 0


def myfunc(world, screen, backwardOrForward, rightOrLeft):
    world.update(screen, backwardOrForward, rightOrLeft)

# --- main game loop ---
running = True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if (event.type == KEYDOWN):
            # print("key down", event.key)
            if (event.key == 97): #left
                rightOrLeft = 1
            if (event.key == 100): #right
                rightOrLeft = -1
            if (event.key == 119): #forward
                backwardOrForward = 1
            if (event.key == 115): #backward
                backwardOrForward = -1
        if (event.type == KEYUP):
            # print("key up", event.key)
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
    
    time1 = time.time()
    
    for person in population:
        backwardOrForward = uniform(-1,1)
        rightOrLeft = uniform(-1,1)
        # t = Thread(target=myfunc, args=(person,screen, backwardOrForward, rightOrLeft))
        # t.start()
        person.update(screen, backwardOrForward, rightOrLeft)

    
    time2 = time.time()
    print('{:s} function took {:.3f} ms'.format("tick", (time2-time1)*1000.0))

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
