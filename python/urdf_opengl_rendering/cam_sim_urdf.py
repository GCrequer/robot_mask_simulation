# Inspiration: Mirado Rajaomarosata

import pygame
from pygame.locals import *
from OpenGL.GL import *
import cv2
import os
import numpy as np
## update of numpy implies to redefine some types
np.int = np.int32
np.bool = np.bool_


from _URDFFileLoader import myURDF
from _Camera import *
from _EventHandler import EventHandler1


# Initialiser Pygame et créer une fenêtre
pygame.init()
display = (700, 700)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

# Création de l'instance de la classe Camera
camera = Camera(display, EventHandler1())


# LOAD OBJECT AFTER PYGAME INIT
bot = myURDF("../../urdf/ur5/ur5.urdf")

glClearColor(1.0, 1.0, 1.0, 1.0)
bot.generate()


order = 51
#gaussian kernel
myKernel = np.zeros((order, order))
myKernel[order//2, order//2] = 1

#update plot
def update_order(val):
    global window1D, myKernel
    order = 2*int(val/2)+1 #nombre impair supérieur le plus proche de val
    
    #gaussian kernel
    myKernel = np.zeros((order, order))
    myKernel[order//2, order//2] = 1
    myKernel = cv2.GaussianBlur(myKernel, (order, order), 0)

    #Hamming window
    #myKernel = np.hamming(order)[:, np.newaxis] * np.hamming(order)[np.newaxis, :]

    #kaiser window
    #myKernel = np.kaiser(order, 14)[:, np.newaxis] * np.kaiser(order, 14)[np.newaxis, :]

    #hanning window
    #myKernel = np.hanning(order)[:, np.newaxis] * np.hanning(order)[np.newaxis, :]

    #blackman window
    #myKernel = np.blackman(order)[:, np.newaxis] * np.blackman(order)[np.newaxis, :]

    #mean filter
    #myKernel = np.ones((order, order)) / (order**2)

    #median filter
    #myKernel = np.ones((order, order))

cv2.namedWindow('processed image')
cv2.createTrackbar('Order', 'processed image', order, 100, update_order)


# Boucle principale
while True:
    order = cv2.getTrackbarPos('Order', 'processed image')
    update_order(order)

    camera.handle_events()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    bot.render()

    #for i in range(bot.nb_parts):
        #glPushMatrix()
        #glMultMatrixf(bot.pose(i).T)  # Apply the pose transformation
        #bot.render_part(i)  # Draw the i-th part
        #glPopMatrix()
        
    camera.perspective_projection()
    

    # Capture the current frame
    pixels = glReadPixels(0, 0, display[0],display[1], GL_BGR, GL_UNSIGNED_BYTE)
    img = np.frombuffer(pixels, dtype=np.uint8).reshape(display[0],display[1], 3)
    img = cv2.flip(img, 0); img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blurred_img = cv2.filter2D(img, 0, myKernel)

    cv2.imshow('processed image', blurred_img)

    pygame.display.flip()
    pygame.time.wait(10)
    cv2.waitKey(1)
