# This unit is aimed to give particle's movement simulation tools
# Later some of the classes can be used to describe a real field
# Creation date 17-7-21


import math
from math import sqrt
from abc import ABCMeta, abstractproperty, abstractmethod
import cv2
import numpy as np
import time


class CommonParticleClass():
# """Parent abstract class for particles"""

    __metaclass__ = ABCMeta
        
    @abstractmethod
    def move():
        '''Moves particle'''
        
    @abstractmethod
    def set_state():
        '''defines new state'''
   
    @abstractmethod
    def get_state():
        '''get the state'''
        
    @abstractmethod
    def get_size():
        '''get the particle size'''
        
    @abstractmethod
    def get_coord():
        '''get the particle coord'''
        



class SimulatedParticleClass(CommonParticleClass):
# """Class defines properties of particles for the purpose of !simulation! only"""    
    
    def __init__(self, coord_tuple, reaction_f, size):
        self.__x, self.__y = coord_tuple
        self.__reaction_f = reaction_f
        self.__size = size
        self.__ro = 1
        self.__mass = 3.14 * (self.__size**3)*self.__ro / 6
        self.__state = 'Active'
    
    # moves the particle according to external action and particle's reaction
    def move(self, action,time):
        if self.__state == 'Active':
            reaction_x, reaction_y = self.__reaction_f(action)
            new_x = reaction_x * time**2 / self.__mass
            new_y = reaction_y * time**2 / self.__mass
            self.__x = self.__x + new_x
            self.__y = self.__y + new_y
            #print('Real position')
            #print((self.__x,self.__y))
            return 0
        else:
            return 1
    
    def set_state(self,new_state = 'Active'):
        self.__state = new_state 
    
    def get_state(self):
        return self.__state
        
    def get_size(self):
        return self.__size
    
    def get_coord(self):
        return (self.__x, self.__y)
    


class SimulatedTargetsClass(CommonParticleClass):
# """Class defines properties of targets and not active particles for the purpose of !simulation! only"""   
    def __init__(self, coord_tuple, reaction_f, size):
        self.__x, self.__y = coord_tuple
        self.__size = size
        self.__reaction = reaction_f
        self.__state = 'Free'
        
    def get_size(self):
        return self.__size
    
    def get_coord(self):
        return (self.__x, self.__y) 
    
    def move():
        pass
    
    def set_state():
        pass
    
    def get_state():
        pass



class SimulatedFieldClass():
#"""describes the "real" work field object for simulation"""

    def __init__(self, particles_dict, targets_dict, borders):
#    """borders - tuple (left,bottom,right,top)"""
        
        self.__particles = particles_dict
        self.__targets = targets_dict
        self.__left, self.__top, self.__bottom, self.__right = borders 
        
        print((self.__left, self.__top, self.__right, self.__bottom))
    
    def add_particle(self,new_id_key, new_particle):
        if new_id_key not in self.__particles.keys():
            self.__particles[new_id_key] = new_particle
    
    def remove_particle(self, id_key):
        if id_key in self.__particles.keys():
            self.__particles.pop(id_key, None)
    
    def apply_action(self, action_tuple, time):
        really_moved = 0
        for p_key in self.__particles.keys():
            if self.__particles[p_key].get_state() == 'Active':
                self.__particles[p_key].move(action_tuple, time)
                really_moved = really_moved + 1
        return really_moved
    
    # check if particles are at targets and mark those particles
    def mark_particle_target_stick(self):
        sticked = 0
        for p_key in self.__particles.keys():
            for t_key in self.__targets.keys():
                p_x, p_y = self.__particles[p_key].get_coord()
                t_x, t_y = self.__targets[t_key].get_coord()
                
                p_size = self.__particles[p_key].get_size()
                t_size = self.__targets[t_key].get_size()
                
                dist = sqrt((t_x - p_x)**2 + (t_y - p_y)**2) - p_size - t_size
                
                if dist <= 0:
                    self.__particles[p_key].set_state(new_state = 'AtTarget')
                    sticked = sticked + 1
                    print('Sticked to target')
        return sticked
    
    # check if particles interact with other particles and mark them
    def mark_particle_particle_stick(self):
        sticked = 0
        for p_key in self.__particles.keys():
                for t_key in self.__particles.keys():
                    if (self.__particles[p_key].get_state() == 'Active') and (p_key != t_key):
                            p_x, p_y = self.__particles[p_key].get_coord()
                            t_x, t_y = self.__particles[t_key].get_coord()
                
                            p_size = self.__particles[p_key].get_size()
                            t_size = self.__particles[t_key].get_size()
                
                            dist = sqrt((t_x - p_x)**2 + (t_y - p_y)**2) - p_size - t_size
                
                            if dist <= 0:
                                self.__particles[p_key].set_state(new_state = 'AtParticle')
                                self.__particles[t_key].set_state(new_state = 'AtParticle')
                                sticked = sticked + 1
                                print('Sticked to other particle')
                        

        return sticked
    
    
    # mark particles who crossed the field border
    def mark_particle_ran_away(self):
        ran_away = 0
        for p_key in self.__particles.keys():
            if self.__particles[p_key].get_state() == 'Active':
                    
                    p_x, p_y = self.__particles[p_key].get_coord()
                    
                    if (p_x < self.__left)  or (p_x > self.__right) or (p_y > self.__bottom) or (p_y < self.__top):
                        self.__particles[p_key].set_state('AtBorder')
                        ran_away = ran_away + 1
                        print('Ran away the border')
                        
        return ran_away
            
    
    def next_time_step(self, action_tuple, time):
        # has to be rewritten
        self.apply_action(action_tuple, time)
        p_t_s = self.mark_particle_target_stick()   
        
        p_p_s = self.mark_particle_particle_stick()
        
        p_r_a = self.mark_particle_ran_away()
               
        return p_t_s + p_p_s + p_r_a
    
    def generate_image(self, width = 640, height = 480):
        
        image = np.zeros((height, width))
        image = image + 255
        
        
        
        for p_key in self.__particles.keys():
            x,y = self.__particles[p_key].get_coord()
            center = (int(x),int(y))
            r = self.__particles[p_key].get_size()
            cv2.circle(image,center, r, (0,0,0), 3)
        
        for p_key in self.__targets.keys():
            x,y = self.__targets[p_key].get_coord()
            center = (int(x),int(y))
            r = self.__targets[p_key].get_size()
            cv2.circle(image,center, r, (0,0,0), -1)
        
        image = image.astype("uint8")
        return image
    

# def reaction_f_example(self,action):
#     some example reaction
#    x_a,y_a,f_a = action
    
#    x_re = (x_a + 0.1*y_a) * f_a
#    y_re = (y_a + 0.5*x_a) * f_a

#    return x_re,y_re
    
#def reaction_0_example(self,action):
#    """ zero reaction for non-active elements(particles)"""
#    return 0,0

#def generate_field_example(self, active_p_action, target_p_action):
#    """ example field generation"""
#    particle1 = self.SimulatedParticleClass((30,30),active_p_action,25)

    
#    target1 = self.SimulatedTargetsClass((125,75), target_p_action,20)
#    target2 = self.SimulatedTargetsClass((180,250),target_p_action, 30)
#    target3 = self.SimulatedTargetsClass((330,310), target_p_action,25)
    
#    particles_dict = {'p1':particle1}
#    targets_dist = {'t1':target1, 't2':target2, 't3':target3}
#    borders = (0,0,480,640)
    
#    RealField = self.SimulatedFieldClass(particles_dict,targets_dist,borders)
#    return RealField"""


