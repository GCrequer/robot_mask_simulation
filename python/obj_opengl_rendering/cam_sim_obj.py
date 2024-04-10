# Inspiration: Mirado Rajaomarosata

import pygame
from pygame.locals import *
from OpenGL.GL import *
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

from _OBJFileLoader import OBJ
from _Camera import Camera


# Initialiser Pygame et créer une fenêtre
pygame.init()
display = (700, 700)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

# LOAD OBJECT AFTER PYGAME INIT
obj = OBJ("solids.obj")

glClearColor(1.0, 1.0, 1.0, 1.0)

obj.generate()

# Pose initial
#glTranslatef(10.0, 1.0, -20)

# Création de l'instance de la classe Camera
camera = Camera(display)


order = 51
#gaussian kernel
myKernel = np.zeros((order, order))
myKernel[order//2, order//2] = 1
myKernel = cv2.GaussianBlur(myKernel, (order, order), 0)


#update plot
def update_order(val):
    global window1D, myKernel
    order = 2*int(val/2)+1 #nombre impair supérieur le plus proche de val
    myKernel = np.zeros((order, order))
    myKernel[order//2, order//2] = 1
    myKernel = cv2.GaussianBlur(myKernel, (order, order), 0)

cv2.namedWindow('Settings')
cv2.createTrackbar('Order', 'Settings', order, 100, update_order)

fig = plt.figure()
ax0 = fig.add_subplot(111)



# Boucle principale
while True:
    order = cv2.getTrackbarPos('Order', 'Settings')
    update_order(order)

    camera.handle_events()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    obj.render()
    camera.perspective_projection()

    # Capture the current frame
    pixels = glReadPixels(0, 0, display[0],display[1], GL_BGR, GL_UNSIGNED_BYTE)
    img = np.frombuffer(pixels, dtype=np.uint8).reshape(display[0],display[1], 3)
    img = cv2.flip(img, 0); img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blurred_img = cv2.filter2D(img, 0, myKernel)
    #blurred_img = cv2.imread("rendu.png", cv2.IMREAD_GRAYSCALE)
    ft = np.fft.ifftshift(blurred_img)
    ft = np.fft.fft2(ft)
    ft = 20*np.log(np.fft.fftshift(abs(ft))+1)
    
    cv2.imshow('processed image', blurred_img)
    ax0.clear()
    ax0.imshow(ft, cmap='gray')

    pygame.display.flip()
    pygame.time.wait(10)
    cv2.waitKey(1)
    
    plt.pause(0.001)
    
plt.show()
