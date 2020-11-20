
import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *

from . import tool

from .snap.tool import coroutine
from .snap.math import Quaternion

import math

topology = [

    ('head', 'neck'),
    ('neck', 'torso'),

    # ('torso', 'lshoulder'),
    ('lshoulder', 'lelbow'),
    ('lelbow', 'lhand'),

    ('torso', 'lhip'),
    ('lhip', 'lknee'),
    ('lknee', 'lfoot'),
    ('lfoot', 'ltoes'),

    # ('torso', 'rshoulder'),
    ('rshoulder', 'relbow'),
    ('relbow', 'rhand'),

    ('torso', 'rhip'),          
    ('rhip', 'rknee'),
    ('rknee', 'rfoot'),
    ('rfoot', 'rtoes'),    


    ('lhip', 'rhip'),

    ('lshoulder', 'neck'),
    ('rshoulder', 'neck'),
    
]


quad = gluNewQuadric()

@coroutine
def color_cylinder():
    quad = gluNewQuadric()
    tex = glGenTextures(1)

    while True:
        radius, height, start, end = yield
        glEnable(GL_TEXTURE_1D)
        glBindTexture(GL_TEXTURE_1D, tex)

        data = np.zeros(6, dtype = np.float32 )

        data[:3] = start
        data[3:] = end

        ptr = np.ctypeslib.as_ctypes(data)
        glTexImage1D(GL_TEXTURE_1D, 0, 3, 2, 0,
                     GL_RGB, GL_FLOAT, ptr)

        glTexParameter(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)        
        glTexParameter(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameter(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)        
        
        glEnable(GL_TEXTURE_GEN_S)
        
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
        glTexGenfv(GL_S, GL_OBJECT_PLANE, [0.0, 0.0, 1.0 / height, 0.0])
        glColor(1, 1, 1)
        
        gluCylinder(quad, radius, radius, height, 16, 4)

        glDisable(GL_TEXTURE_GEN_S)
        glBindTexture(GL_TEXTURE_1D, 0)
        glDisable(GL_TEXTURE_1D)
    

cr = color_cylinder()

def draw(pose, **kwargs):

    radius = kwargs.get('radius', 0.02)
    color = kwargs.get('color', None)
    
    deg = 180.0 / math.pi
    
    for name, position in pose.items():
        glPushMatrix()
        glTranslate(*position)

        if color is not None:
            glColor( color[name] )
            
        gluSphere(quad, 0.02, 16, 8)
        glPopMatrix()

    # glColor(1, 1, 1)
    for p, c in topology:

        try:
            vp = np.array( pose[p] )
            vc = np.array( pose[c] )

            # parent -> child
            diff = vc - vp
            q = Quaternion.from_vectors(tool.ez, diff)

            glPushMatrix()
            glTranslate(*vp)
            axis, angle = q.axis_angle()

            if axis is not None:
                glRotate(angle * deg, *axis )

            s = np.linalg.norm(diff)

            if color is None:
                gluCylinder(quad, radius, radius, s, 16, 2)
            else:
                cr.send( (radius, s, color[p], color[c]) )

            glPopMatrix()
        except KeyError:
            pass
        
