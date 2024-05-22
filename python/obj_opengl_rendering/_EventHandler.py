# Author : Mirado Rajaomarosata
# Date : 22/09/2023

import numpy as np
import pygame
from pygame.locals import *

class EventHandler():
    
    def handle_events(self):
        speed = 0.2
        translation_speed = speed
        rotation_speed = speed

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Clic gauche
                    self.mouse_down = True
                    self.old_mouse_pos = event.pos
                elif event.button == 4:
                    self.t[2, 0] -= translation_speed  # Scroll up
                elif event.button == 5:
                    self.t[2, 0] += translation_speed  # Scroll down
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.mouse_down = False
            elif event.type == pygame.MOUSEMOTION and self.mouse_down:
                dx, dy = event.pos[0] - self.old_mouse_pos[0], event.pos[1] - self.old_mouse_pos[1]
                self.old_mouse_pos = event.pos
                if dx != 0:
                    rx = -np.radians(dx * rotation_speed)
                    Ry = np.array([[np.cos(rx), 0, np.sin(rx)], [0, 1, 0], [-np.sin(rx), 0, np.cos(rx)]])
                    self.R = np.dot(Ry, self.R)
                if dy != 0:
                    ry = -np.radians(dy * rotation_speed)
                    Rx = np.array([[1, 0, 0], [0, np.cos(ry), -np.sin(ry)], [0, np.sin(ry), np.cos(ry)]])
                    self.R = np.dot(Rx, self.R)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            self.t[0, 0] += translation_speed
        if keys[pygame.K_RIGHT]:
            self.t[0, 0] -= translation_speed
        if keys[pygame.K_UP]:
            self.t[1, 0] -= translation_speed
        if keys[pygame.K_DOWN]:
            self.t[1, 0] += translation_speed