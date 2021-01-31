import numpy as np
import pygame
from copy import deepcopy

# def drawCircleArc(screen,color,center,radius,startDeg,endDeg,thickness):
#     (x,y) = center
#     rect = (x-radius,y-radius,radius*2,radius*2)
#     startRad = degreesToRadians(startDeg)
#     endRad = degreesToRadians(endDeg)
   
#     pygame.draw.arc(screen,color,rect,startRad,endRad,thickness)

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


class RobotNonHolonomic():
    def __init__(self, spawn, goal, heading = (1,0), num_children = 7, d = 2, B = 5):
        '''
        Non-holonomic tricycle drive robot

        spawn: initial position or World.spawn
        goal: target position
        num_children: number of children to protrude from each node of the RRT
        d: axle length/2
        B = robot length
        '''
        self.pos = spawn
        self.goal = goal
        self.unexplored_points = [spawn+heading]
        self.num_children = num_children
        self.explored_paths = []
        self.reach_threshold = 5
        self.d = d
        self.B = B

    def print_paths(self, gameDisplay):
        for path in self.explored_paths:
            pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)

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

        point = (node[0], node[1])
        heading = (node[2], node[3])
        

        Vfs = np.random.uniform(-25,25, (1,self.num_children))
        alphas = np.random.uniform(-np.pi/4,np.pi/4, (1,self.num_children))
        
        # kinematics
        Ws = Vfs*np.sin(alphas)/self.B
        R = self.B/(np.tan(alphas)+0.001)
        Vs = Ws*R


        for i in range(self.num_children):
            heading_ = deepcopy(heading)
            point_ = deepcopy(point)
            samples = []
            num_samples = 20

            R = np.array([[np.cos(Ws[0,i]/num_samples), -np.sin(Ws[0,i]/num_samples)],
                          [np.sin(Ws[0,i]/num_samples), np.cos(Ws[0,i]/num_samples)]])
            prev_point = deepcopy(point)
            
            for j in range(num_samples):
                velocity = np.array(heading_)*Vs[0,i]/num_samples
                point_ = deepcopy((point_[0]+velocity[0], point_[1]+velocity[1]))
                samples.append(deepcopy(point_))
                heading_ = R@(np.array(heading_).T)
                heading_ = (heading_[0], heading_[1])         
                if j%5 == 4:
                    if world.check_collision_holonomic(point_, prev_point):
                        break
                    elif j == num_samples-1:
                        self.explored_paths.append(samples)
                        self.unexplored_points.append((point_[0], point_[1], heading_[0], heading_[1]))        
        # print(len(self.explored_paths))
        return False