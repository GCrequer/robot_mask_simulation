# Author : Mirado Rajaomarosata
# Date : 22/09/2023


import pygame
from pygame.locals import *
from OpenGL.GL import *

from _OBJFileLoader import OBJ
from _Camera import *


# Initialiser Pygame et créer une fenêtre
pygame.init()
display = (1200, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

# LOAD OBJECT AFTER PYGAME INIT
obj = OBJ("solids.obj")

glClearColor(1.0, 1.0, 1.0, 1.0)

obj.generate()

# Pose initial
#glTranslatef(10.0, 1.0, -20)

# Création de l'instance de la classe Camera
camera = Camera(display)

# Boucle principale
while True:
    camera.handle_events()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glViewport(0, 0, display[0], display[1])
    camera.perspective_projection()
    obj.render()


    pygame.display.flip()
    pygame.time.wait(10)
