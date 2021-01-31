import numpy as np
import pygame

class RobotHolonomic():
    def __init__(self, spawn, goal, num_children = 3):
        '''
        spawn: initial position or World.spawn
        goal: target position
        num_children: number of children to protrude from each node of the RRT
        '''
        self.pos = spawn
        self.goal = goal
        self.unexplored_points = [spawn]
        self.num_children = num_children
        self.explored_lines = []

    def print_robot(self, gameDisplay):
        '''
        prints robot on to <gameDisplay>
        '''
        pygame.draw.circle(gameDisplay,(0,0,255),self.pos, (3,3))
    
    def RRT_step(self, target):
        
        min_index = 0
        min_distance = np.inf
        for i, point in enumerate(self.unexplored_points):
            distance = np.sqrt((point[0] - goa;[0])**2 + (point[1] - goal[1])**2)
            if distance<min_distance:
                min_distance = distance
                min_index = i
        
        node = self.unexplored_points.pop(min_index)

