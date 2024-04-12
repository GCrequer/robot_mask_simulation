# inspired by Mirado Rajaomarosata

import numpy as np
import pygame
from pygame.locals import *


class EventHandler1:

    def __init__(self):

        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

    def handle_events(self):
        speed = 0.2
        translation_speed = speed
        rotation_speed = speed

        for event in pygame.event.get():

            if event.type == pygame.QUIT: # close the window
                pygame.quit()
                quit()
            elif event.type == pygame.MOUSEWHEEL:
                print("zoom")
                if event.y > 0:
                    self.t[2, 0] -= translation_speed
                elif event.y < 0:
                    self.t[2, 0] += translation_speed

            elif event.type == pygame.MOUSEMOTION and event.buttons[0]:
                print("rotation")
                
                rx = -np.radians(event.rel[0] * rotation_speed)
                ry = -np.radians(event.rel[1] * rotation_speed)
                Ry = np.array([[np.cos(rx), 0, np.sin(rx)], [0, 1, 0], [-np.sin(rx), 0, np.cos(rx)]])
                Rx = np.array([[1, 0, 0], [0, np.cos(ry), -np.sin(ry)], [0, np.sin(ry), np.cos(ry)]])

                self.R = Rx @ Ry @ self.R
    
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            self.t[0, 0] += translation_speed
        if keys[pygame.K_RIGHT]:
            self.t[0, 0] -= translation_speed
        if keys[pygame.K_UP]:
            self.t[1, 0] -= translation_speed
        if keys[pygame.K_DOWN]:
            self.t[1, 0] += translation_speed
        
        
        return self.R, self.t
