import pygame
import numpy
from os import system
from world import *
from robot import*
import time

clock = pygame.time.Clock()

h=600
w=600
gameDisplay=pygame.display.set_mode((h,w))
pygame.display.set_caption("Title")

crashed = False; done = False

sim = World(h,w)

robo = RobotHolonomic(sim.spawn, sim.goal)

obstacles = [(0,0,100,100),
             (100,150,10,100),
             (250,250,10,300)]

sim.add_obstacles(obstacles)


num_steps = 1000
step = 0
while not (done or crashed):
    print(step)
    step = step+1
    time.sleep(0.5)
    # crashing the simulation manually
    for i in pygame.event.get():
      if i.type == pygame.KEYDOWN:
        if i.unicode == "q":
          crashed = True
    
    if step%2 == 0:
      goalx = np.random.uniform(0,h)
      goaly = np.random.uniform(0,w)
      goal = (goalx, goaly)
    else:
      goal = sim.goal
    
    done = robo.RRT_step(goal, sim)

    gameDisplay.fill((255,255,255))
    sim.print_obstacles(gameDisplay)
    robo.print_paths(gameDisplay)
    pygame.display.update()
    clock.tick(60)

    if step>num_steps:
      done = True