# Author : Mirado Rajaomarosata
# Date : 22/09/2023

# Inspired by https://www.pygame.org/wiki/OBJFileLoader

import os
from OpenGL.GL import *
import numpy as np
from urchin import URDF

class myURDF:

    def __init__(self, filename, swapyz=False):
        """Loads a urdf file. """


        os.chdir(os.path.dirname(__file__)) 
        self.bot = URDF.load(filename, lazy_load_meshes=True)
   
        self.nb_parts = len(list(self.bot.visual_trimesh_fk().keys()))
        self.gl_list = 0


    def face(self, i):
        return list(self.bot.visual_trimesh_fk().keys())[i].faces

    def vert(self, i):
        return list(self.bot.visual_trimesh_fk().keys())[i].vertices
    
    def pose(self, i):
        return list(self.bot.visual_trimesh_fk().values())[i]

    def generate(self):
        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glFrontFace(GL_CCW)
        for i in range(self.nb_parts):
            vertices = self.vert(i)
            faces = self.face(i)
            for face in faces:
                glBegin(GL_POLYGON)
                glColor([0., 0. , 0.])
                for v_ind in face:
                    vec = np.array([[vertices[v_ind][0], vertices[v_ind][1], vertices[v_ind][2], 1]]).T
                    glVertex3fv((self.pose(i) @ vec).flatten()[:3])
                glEnd()

        glEndList()

    def render(self):
        glCallList(self.gl_list)

    def free(self):
        glDeleteLists([self.gl_list])