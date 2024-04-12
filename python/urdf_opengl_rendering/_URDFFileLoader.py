# Author : Mirado Rajaomarosata
# Date : 22/09/2023

# Inspired by https://www.pygame.org/wiki/OBJFileLoader

import os
from OpenGL.GL import *
import numpy as np
from urchin import URDF
import threading

class myURDF:

    def __init__(self, filename, swapyz=False):
        """Loads a urdf file. """
        self.gl_list = 0

        os.chdir(os.path.dirname(__file__)) 
        self.bot = URDF.load(filename, lazy_load_meshes=True)
   
        self.nb_parts = len(list(self.bot.collision_trimesh_fk().keys()))
        

        self.faces = [list(self.bot.collision_trimesh_fk().keys())[i].faces for i in range(self.nb_parts)]
        self.vertices = [list(self.bot.collision_trimesh_fk().keys())[i].vertices for i in range(self.nb_parts)]



    def face(self, i):
        return list(self.bot.collision_trimesh_fk().keys())[i].faces

    def vert(self, i):
        return list(self.bot.collision_trimesh_fk().keys())[i].vertices
    
    def pose(self, i):
        return list(self.bot.collision_trimesh_fk().values())[i]


    def process_part(self, i):
        print(f"{i+1}/{self.nb_parts}: {len(self.faces[i])} faces")

        for face in self.faces[i]:
            glBegin(GL_POLYGON)
            glColor([0., 0. , 0.])
            for v_ind in face:
                vec = np.array([[self.vertices[i][v_ind][0], self.vertices[i][v_ind][1], self.vertices[i][v_ind][2], 1]]).T
                glVertex3fv((self.pose(i) @ vec).flatten()[:3])
            glEnd()

    def generate(self):
        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glFrontFace(GL_CCW)

        #threads = []
        for i in range(self.nb_parts):
            #t = threading.Thread(target=self.process_part, args=(i,))
            #t.start()
            #threads.append(t)
            self.process_part(i)

        # Wait for all threads to finish
        #for t in threads:
        #    t.join()

        glEndList()

    def render(self):
        glCallList(self.gl_list)

    def free(self):
        glDeleteLists([self.gl_list], range(1))

    def regenerate(self):
        self.free()
        self.generate()

"""
    def generate(self):
        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glFrontFace(GL_CCW)
        for i in range(self.nb_parts):
            print(f"{i}/{self.nb_parts}: {len(self.faces[i])} faces")

            for face in self.faces[i]:
                glBegin(GL_POLYGON)
                glColor([0., 0. , 0.])
                for v_ind in face:
                    vec = np.array([[self.vertices[i][v_ind][0], self.vertices[i][v_ind][1], self.vertices[i][v_ind][2], 1]]).T
                    glVertex3fv((self.pose(i) @ vec).flatten()[:3])
                glEnd()

        glEndList()
"""