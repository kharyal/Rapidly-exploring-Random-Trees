import numpy as np
import pygame

class RobotHolonomic():
    def __init__(self, spawn, goal, num_children = 5):
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
        self.reach_threshold = 5

    def print_robot(self, gameDisplay):
        '''
        prints robot on to <gameDisplay>
        '''
        pygame.draw.circle(gameDisplay,(0,0,255),self.pos, (3,3))

    def print_paths(self, gameDisplay):
        for line in self.explored_lines:
            pygame.draw.line(gameDisplay, (147, 205, 218), line[0], line[1], 2)
    
    def RRT_step(self, target, world):
        
        min_index = 0
        min_distance = np.inf
        for i, point in enumerate(self.unexplored_points):
            distance = np.sqrt((point[0] - target[0])**2 + (point[1] - target[1])**2)
            if distance<min_distance:
                min_distance = distance
                min_index = i
            if np.sqrt((point[0] - self.goal[0])**2 + (point[1] - self.goal[1])**2) < self.reach_threshold:
                return True
        
        # node = self.unexplored_points.pop(min_index)
        node = self.unexplored_points[min_index]

        Vs = np.random.uniform(-20,20, (2,self.num_children))
        
        for i in range(self.num_children):
            if not world.check_collision_holonomic(node, (node[0]+Vs[0,i], node[1]+Vs[1,i])):
                self.unexplored_points.append((node[0]+Vs[0,i], node[1]+Vs[1,i]))
                self.explored_lines.append([node,(node[0]+Vs[0,i], node[1]+Vs[1,i])])
        
        return False