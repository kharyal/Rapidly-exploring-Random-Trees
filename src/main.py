import pygame
import numpy
from os import system
from world import *

clock = pygame.time.Clock()

h=600
w=600
gameDisplay=pygame.display.set_mode((h,w))
pygame.display.set_caption("Title")

crashed = False

sim = World(h,w)

obstacles = [(0,0,100,100),
             (100,150,10,100)]

sim.add_obstacles(obstacles)

while(not crashed):

    # crashing the simulation manually
    for i in pygame.event.get():
      if i.type == pygame.KEYDOWN:
        if i.unicode == "q":
          crashed = True
    

    gameDisplay.fill((255,255,255))
    sim.print_obstacles(gameDisplay)
    pygame.display.update()
    clock.tick(60)