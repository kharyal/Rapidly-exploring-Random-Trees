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
gameDisplay.blit(pygame.transform.flip(gameDisplay, False, True), dest=(0, 0))

crashed = False; done = False

sim = World(h,w)

robo = RobotNonHolonomic_tricycle_drive(sim.spawn, sim.goal)
robot_size = robo.robot_size + 3

obstacles = [[0,0,100,100],
             [100,150,10,100],
             [250,250,10,300],
             [100, 245, 150,10],
             [350, 0, 10, 300],
             [350, 295, 170, 10]]

sim.add_obstacles(obstacles)
sim.grow_obstacles(robot_size)


num_steps = 100000
step = 0
while not (done or crashed):
    print(step)
    step = step+1
    # time.sleep(1)

    # crashing the simulation manually
    for i in pygame.event.get():
      if i.type == pygame.KEYDOWN:
        if i.unicode == "q":
          crashed = True
    
    if step%2 == 0:
      goal = sim.goal      
    else:
      goalx = np.random.uniform(0,h)
      goaly = np.random.uniform(0,w)
      goal = (goalx, goaly)
    
    done = robo.RRT_step(goal, sim)

    gameDisplay.fill((255,255,255))
    sim.print_obstacles(gameDisplay, robot_size)
    robo.print_paths(gameDisplay, type='center')
    # robo.print_paths(gameDisplay, type='wheels')
    pygame.display.update()
    clock.tick(60)

    if step>num_steps:
      done = True