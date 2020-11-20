


#from . import path

import numpy as np
import scipy as sp

from scipy import linalg
from numpy import linalg

from numpy.linalg import norm

from OpenGL.GL import *
from OpenGL.GLU import *

import math
import os
import sys

#from . import mlcp
    
    
from scipy import spatial

# def convex_hull(points):
#     m, n = points.shape

#     if m <= 3: return np.arange(m)
#     return sp.spatial.ConvexHull(points).vertices
    


# def hull_project(hull, p, contraction = 0):
#     n = len(hull)
    
#     if n == 0: return p, 0.0
    
#     # constraint matrix/value
#     G = np.zeros( (n, 2) )
#     g = np.zeros(n)

#     # center
#     c = np.sum(hull, axis = 0) / n

#     for i, s in enumerate(hull):
#         i_next = (i + 1) % n

#         edge = hull[i_next] - s

#         orth = np.array([-edge[1], edge[0]])

#         inner = orth if orth.dot(s - c) <= 0 else -orth
#         inner /= norm(inner)

#         G[i, :] = inner
#         g[i] = inner.dot(s)

#     # lcp
#     M = G.dot(G.T)
#     # M += contraction * np.identity(n)
    
#     q = g - G.dot(p) + contraction

#     x = np.zeros(n)
#     mlcp.solve(x, M, -q, it = 20, nlnscg = 1)

#     # constraint forces
#     f = G.T.dot(x)

#     # qp solution
#     y = p + f
    
#     return y, norm(f)

joint_names = {
    'head',
    'neck',
    'torso',

    'lshoulder',
    'lelbow',
    'lhand',
    'lhip',
    'lknee',
    'lfoot',
    'ltoes',

    'rshoulder',
    'relbow',
    'rhand',
    'rhip',
    'rknee',
    'rfoot',
    'rtoes',
}


joint_index = { k: i for i, k in enumerate(joint_names) }

# def kinect2(**kwargs):
#     '''kinect2 data generator

#     yields a dictionary {
#       'body': body markers as a dictionary {name: position}
#       'color': bgra color image as a numpy array
#     }
#     '''

#     from . import kinect2 as k2
#     from .kinect2 import joint_type as jt

#     global joint_map, joint_index
#     joint_map = {
#         'head': jt.head,
#         'neck': jt.spine_shoulder,

#         'torso': jt.spine_mid,

#         'lshoulder': jt.shoulder_left,
#         'lelbow': jt.elbow_left,
#         'lhand': jt.wrist_left,

#         'lhip': jt.hip_left,

#         'lknee': jt.knee_left,
#         'lfoot': jt.ankle_left,
#         'ltoes': jt.foot_left,

#         'rshoulder': jt.shoulder_right,
#         'relbow': jt.elbow_right,
#         'rhand': jt.wrist_right,

#         'rhip': jt.hip_right,
#         'rknee': jt.knee_right,
#         'rfoot': jt.ankle_right,
#         'rtoes': jt.foot_right,
#     }

    
#     body = kwargs.get('body', True)
#     color = kwargs.get('color', True)

#     flags = 0

#     if body: flags += k2.source_type.body
#     if color: flags += k2.source_type.color
    
#     for f in k2.frames(flags):

#         res = {}
#         body = f.get('body', None)
        
#         if body is not None:
#             for index, pose in body.items():
#                 res['body'] = { name: pose[value] for name, value in joint_map.items() }

#         floor = f.get('floor', None)
#         if floor is not None:
#             res['floor'] = floor
            
#         img = f.get('color', None)
#         if img is not None:
#             res['color'] = img


#         yield res

# def kinect2_thread(queue, **kwargs):
#     '''start kinect thread and return communication queue'''
    
#     from threading import Thread
    
#     def target(q):

#         for frame in kinect2(**kwargs):
#             if not q.empty(): q.get_nowait()
#             q.put_nowait( frame )

#     thread = Thread(target = target, args = (queue, ))
#     thread.daemon = True
#     thread.start()


from atexit import register as on_exit
    
# def wiiboard():
    # from . import wii
    
    # '''wiiboard data generator

    # yields (cop, f) where cop are the center of pressure coordinates
    # in a normalized frame (or None), and f is the ground forces
    # magnitude

    # '''
    
    # vertices = np.array( [
    #     [1, 1],                 # TR
    #     [1, -1],                # BR
    #     [-1, 1],                # TL
    #     [-1, -1]                # BL
    # ])

    # for weights in wii.frames():

    #     total = sum(weights)

    #     if total > 0:
    #         cop = vertices.T.dot(weights) / total
    #     else:
    #         cop = None
            
    #     yield cop, total

ez = np.zeros(3)
ez[2] = 1

ey = np.zeros(3)
ey[1] = 1

ex = np.zeros(3)
ex[0] = 1

deg = 180.0 / math.pi

from numpy.linalg import norm


# TODO move
# dabeaz ftw !11
def coroutine(f):
    '''coroutine decorator.
    
    automatically bootstraps coroutines by advancing them once after
    creation.
    '''

    def start(*args, **kwargs):
        res = f(*args, **kwargs)
        next(res)

        return res

    return start

# TODO move ?
@coroutine
def savitsky_golay(order, degree = 3, **kwargs):
    '''savitsky-golay filter as a coroutine.

    receives (t, pos) where t is the time and pos is some vector,
    yields (dpos, d2pos), where dpos is the velocity and d2pos is the
    acceleration. 

    order: the number of points to consider when fitting polynomials
    degree: the degree of the fitted polynomial.
    '''

    only_new = kwargs.get('only_new', True)
    
    prev = []
    vel = None
    acc = None
    
    while True:
        t, x = yield vel, acc

        if not only_new or (not prev or norm(prev[-1][1] - x) != 0):
            prev.append( (t, np.copy(x)) )
            if len(prev) > order: prev.pop(0)

        start = prev[-1][0]

        # polynomial basis evaluated at previous timesxs
        J = np.array([ [ (p[0] - start) ** i for i in range(degree + 1)] for p in prev ])

        # previous samples
        Y = np.array([p[1] for p in prev])

        # we want JA = Y (least-squares): A[:, i] is the polynomial
        # for ith coordinate in Y
        m, n = J.shape

        # m = samples, n = degree + 1
        s = min(m, n)

        # lower degree when not enough points are available
        J = J[:, :s]

        M = J.T.dot(J)
        Minv = sp.linalg.cho_factor(M, check_finite = False)

        # solve for A
        A = sp.linalg.cho_solve( Minv, J.T.dot(Y), check_finite = False)

        # chop off first row in P
        def derive(P):
            rows, cols = P.shape
            D = np.outer( np.arange(1, rows),
                          np.ones(cols) ).view(dtype = float)
            return D * P[1:, :]

        # polynomial derivative
        eval_at = kwargs.get('eval_at', (prev[-1][0] - prev[0][0]) / 2.0)
        
        eval_vec = np.array([eval_at ** i for i in range(s)])
        
        if s > 1:
            dA = derive(A)
            vel = dA.T.dot(eval_vec[:-1])
        else:
            vel = np.zeros( x.shape )

        if s > 2:
            d2A = derive(dA)
            acc = d2A.T.dot(eval_vec[:-2])
        else:
            acc = np.zeros( vel.shape )


def make_deriv(order = 16):
    '''a simple averaging filter for numerical differentiation'''
    # TODO make this one a coroutine as well
    
    import time
    
    pos = []
    t = []

    def deriv(x):
        now = time.time()
        
        try:
            if not t:
                return np.zeros(x.shape)
            
            dt = now - t[0]
            dp = x - pos[0]

            return dp / dt
        
        finally:
            pos.append(np.copy(x))
            t.append(now)

            if len(pos) > order: pos.pop(0)
            if len(t) > order: t.pop(0)

            
    return deriv


class Struct:
    def __init__(self, **env):
        self.__dict__.update(env)



# def focal():
#     '''kinect2 focal length'''
#     hfov = 84.1 / 180.0 * math.pi

#     xscale = math.tan(hfov / 2.0)
#     depth = 1.0 / xscale

#     return depth


# @coroutine
# def draw_background(**kwargs):
#     '''draw kinect2 color stream as a coroutine.

#     receives the latest color texture, yields None.

#     '''
    
#     tex = glGenTextures(1)

#     fmt = kwargs.get('fmt', None)

#     flip = kwargs.get('flip', False)


#     if flip:
#         glMatrixMode(GL_TEXTURE)
#         glScale(-1, 1, 0)
#         glMatrixMode(GL_MODELVIEW)

    
#     while True:

#         img = yield
        
#         glColor(1, 1, 1)
#         glEnable(GL_TEXTURE_2D)
#         glBindTexture(GL_TEXTURE_2D, tex)

#         ptr = np.ctypeslib.as_ctypes(img)
#         height, width, chan = img.shape

#         fmt = fmt or (GL_BGRA if chan == 4 else GL_BGR)
        
#         glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0,
#                      fmt, GL_UNSIGNED_BYTE, ptr)

#         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
#         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

#         # glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
#         glDisable(GL_LIGHTING)
#         glPushMatrix()

#         hfov = 84.1 / 180.0 * math.pi

#         ratio = float(width) / float(height)
        
#         xscale = math.tan(hfov / 2.0)
#         yscale = xscale / ratio
        
#         depth = 1.0

#         glScale(xscale, yscale, 1);
#         glBegin(GL_QUADS)

#         # depth = 1

#         glTexCoord(0, 0)
#         glVertex(-1, 1, depth)

#         glTexCoord(0, 1)
#         glVertex(-1, -1, depth)

#         glTexCoord(1, 1)
#         glVertex(1, -1, depth)

#         glTexCoord(1, 0)
#         glVertex(1, 1, depth)

#         glEnd()
#         glPopMatrix()

#         glEnable(GL_LIGHTING)
#         glClear(GL_DEPTH_BUFFER_BIT)

#         glBindTexture(GL_TEXTURE_2D, 0)
#         glDisable(GL_TEXTURE_2D)


# def draw_logo(pos = [-0.67, 0.45 , focal()], filename = path() + '/share/logo-transparent.png'):
#     '''draw logo as a generator'''

#     import cv2
    
#     png = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
#     if png is None: raise Exception('{0} not found'.format(filename))

#     tex = None
    
#     while True:
#         yield
#         glEnable(GL_TEXTURE_2D)
#         if tex is None:
#             ptr = np.ctypeslib.as_ctypes(png)
#             height, width, _ = png.shape

#             tex = glGenTextures(1)
#             glBindTexture(GL_TEXTURE_2D, tex)

#             glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0,
#                          GL_BGRA, GL_UNSIGNED_BYTE, ptr)

#             glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
#             glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)

#         else:
#             glBindTexture(GL_TEXTURE_2D, tex)

#         glEnable(GL_BLEND);
#         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

#         glDisable(GL_LIGHTING)
#         glClear(GL_DEPTH_BUFFER_BIT)
#         glPushMatrix()

#         glTranslate(*pos)

#         size = 0.3

#         height, width, _ = png.shape
#         glScale(size, float(height) / float(width) * size, 1)

#         glBegin(GL_QUADS)

#         glNormal(0, 0, -1)

#         glTexCoord(1, 0)
#         glVertex(-1., 1)

#         glTexCoord(1, 1)
#         glVertex(-1, -1)

#         glTexCoord(0, 1)
#         glVertex(1, -1)

#         glTexCoord(0, 0)
#         glVertex(1, 1)

#         glEnd()        
#         glPopMatrix()

#         glBindTexture(GL_TEXTURE_2D, 0)
#         glDisable(GL_TEXTURE_2D)

    
# def draw_sphere(quad = gluNewQuadric(), **kwargs):
#     '''draw a(glu)  sphere.
    
#     kwargs:
#     - pos: position
#     - radius: radius
#     '''
    
#     pos = kwargs.get('pos', None)
#     radius = kwargs.get('radius', 1)

#     glPushMatrix()
 
#     if pos is not None:
#         glTranslate(*pos)        

#     gluSphere(quad, radius, 16, 8)        

#     glPopMatrix()
        


# def compute_com(body, calib):
#     res = np.zeros(3)

#     for name, pos in body.items():
#         res += calib[name] * np.array(pos)

#     return res


# def load_calib(filename):
#     with open(filename) as f:        
#         import json
#         return json.loads(f.read())


# TODO move ?
# @coroutine
# def kalman(P, forgetting_factor = 1.0):

#     '''kalman filter as a coroutine. 

#     P = state covariance

#     dy = y - H x_predicted = measurement error
#     H = measurement jacobian/configuration matrix
#     R = measurement covariance

#     F = transition jacobian
#     '''

#     dx = None
#     n, m = P.shape
#     assert n == m, 'square matrix required'
    
#     while True:
#         dy, H, R, F, Q = yield dx, P

# 		# P- = P+ + Q ????
#         P /= forgetting_factor
#         P = F.dot(P).dot(F.T) + Q
        
# 		#HP-H'+R
#         PHT = P.dot(H.T)
#         S = R + H.dot(PHT)

#         # print(S)
        
# 		#(HPH'+R)^-1
#         Sinv = sp.linalg.cho_factor(S, check_finite = False) 
#         SinvH = sp.linalg.cho_solve( Sinv, H, check_finite = False)
        
# 		#K = P-H'(HPH'+R)^-1
# 		#P+ = (E - KH)P-
#         P = (np.identity(n) - PHT.dot(SinvH)).dot(P)
        
# 		#dx = Kw
#         dx = PHT.dot(sp.linalg.cho_solve(Sinv, dy, check_finite = False))


# def minimum_volume_enclosing_ellipsoid(points, tolerance = 1e-3, itmax = 100):
#     '''epsilon-approximation of the minimum volume enclosing ellipsoid for
#     a set of points'''
    
#     P = points.T
#     d, n = P.shape

#     Q = np.zeros( (d + 1, n) )

#     Q[:d, :] = P
#     Q[d, :] = np.ones(n)

#     count = 1
#     err = 1
#     u = 1.0 / n * np.ones(n)
    
#     while err > tolerance:
#         X = Q.dot(np.diag(u)).dot(Q.T)

#         Xinv = sp.linalg.cho_factor(X, check_finite = False)
#         XinvQ = sp.linalg.cho_solve(Xinv, Q, check_finite = False)

#         m = np.sum( Q * XinvQ, axis = 0)
#         # m = np.diag(Q.T.dot(Xinv).dot(Q))

#         j = m.argmax()
#         mmax = m[j]

#         # line-search
#         step_size = (mmax - float(d) - 1.0) / ( float(d + 1) * (mmax - 1.0) )

#         new_u = (1.0 - step_size) * u
#         new_u[j] += step_size

#         count += 1
        
#         err = norm(new_u - u)

#         u[:] = new_u
#         if count >= itmax: break

#     # dual problem is solved
#     U = np.diag(u)
#     c = P.dot(u)
    
#     A = (1.0 / float(d)) * np.linalg.inv(P.dot(U).dot(P.T) - np.outer(c, c))

#     return A, c


# def hyperplane_union(vertices):
#     '''converts polytope to dual representation (convex sum => hyperplane
#     union) 2d only.

#     '''
#     c = np.sum(vertices, axis = 0)

#     n, m = vertices.shape
#     A = np.zeros( (n, m) )
    
#     for i, v in enumerate(vertices):
#         i_next = (i + 1) % n

#         edge = vertices[i_next] - vertices[i]
#         orth = np.array([-edge[1], edge[0]])

#         outer = orth if (v - c).dot(orth) >= 0 else - orth

#         A[i, :] = outer
#         b = outer.dot(v)

#         A[i, :] /= b

#     return A



# def maximum_volume_enclosed_ellipsoid(points, tolerance = 1e-3, itmax = 100):
#     '''epsilon-approximation of the maximum volume inscribed ellipsoid (2d only)'''

#     # convex hull
#     H = points[convex_hull(points)]

#     # compute dual
#     E = hyperplane_union(H)

#     # dual polytope of initial set
#     D = E[convex_hull(E)]

#     # compute mvee
#     A, c = minimum_volume_enclosing_ellipsoid(D, tolerance, itmax)

#     # dual ellipsoid
#     M = np.zeros( (3, 3) )

#     Ac = A.dot(c)
    
#     M[:2, :2] = A
#     M[2, :2] = -Ac
#     M[:2, 2] = -Ac
#     M[2, 2] = c.dot(Ac) + 1.0 # TODO why 1.0 ?

#     Minv = np.linalg.inv(M)
#     b = Minv[2, :2]
    
#     A = Minv[:2, :2]
#     c = -np.linalg.inv(A).dot(b)
    
#     return A, c


# @coroutine
# def lie_rls(**kwargs):
#     '''lie group recursive least squares'''
    
#     c = kwargs.get('c', 1.0)
#     r = kwargs.get('r', 1.0)
    
#     forgetting_factor = kwargs.get('forgetting_factor', 1.0)

#     # group state
#     state = kwargs['init']
#     G = type(state)
    
#     # kalman filter
#     C = np.identity( G.dim ) * c
#     R = None
#     cr_kalman = kalman(C, forgetting_factor)
    

#     # projection operator (for constraints)
#     proj = kwargs.get('proj', lambda C, x: x)
    
#     # delta state solved by kalman filter at each time step
#     dstate = np.zeros( G.dim )

#     # delta state used for stepping (may incorporate constraints)
#     dstate_step = np.zeros( G.dim )    
    
#     while True:
#         # error, measurement jacobian at current state
#         w, H = yield state

#         # transition
#         F = G.dexp( dstate_step )

#         # predicted state
#         dstate_pred = F.dot(dstate - dstate_step)

#         # update error with predicted state
#         w -= H.dot(dstate_pred)

#         # linear kalman update
#         R = R if R is not None else r * np.identity(len(w))
#         Q = 0
#         d2state, C = cr_kalman.send( (w, H, R, F, Q) )

#         # update prediction for dstate
#         dstate = dstate_pred + d2state

#         # project dstate using metric inv(C)
#         dstate_step = proj(C, dstate)

#         # step state
#         state = state * G.exp(dstate_step)


# from .snap import gl
# from .snap.math import Quaternion, Rigid3, vec
# from contextlib import contextmanager

# @contextmanager
# def ellipse_frame(E):
#         A, c = E
#         w, U = np.linalg.eigh(A)
#         winv = 1 / w
        
#         f = Rigid3()
#         f.center[:2] = c
#         u = U[:, 0]
#         f.orient = Quaternion.from_vectors( ex, vec(u[0], u[1], 0))
        
#         with gl.frame(f):
#             coords = np.sqrt(winv).tolist() + [1]
#             glScale(*coords)
#             yield
            

            
# class Ground(object):
#     dim = 6

#     def __init__(self, filename = None):
#         self.frame = Rigid3()


#     def load(self, filename):
#         with open(filename) as f:
#             print('reading ground info from', filename)
#             import json
#             self.frame[:] = json.loads(f.read())
#         return self
    
#     def save(self, filename = 'ground.json'):
#         with open(filename, 'w') as f:
#             import json
#             f.write( json.dumps( self.frame.tolist() ) )
#         print('wrote ground info to', filename)

#     def __mul__(self, other):
#         res = Ground()

#         res.frame.orient = self.frame.orient * other.frame.orient
#         res.frame.center = self.frame.center + other.frame.center

#         return res

#     def inv(self):
#         res = Ground()

#         res.frame.orient = self.frame.orient.inv()
#         res.frame.center = - self.frame.center

#         return res

#     @staticmethod
#     def exp(x):

#         res = Ground()

#         res.frame.orient = Quaternion.exp(x[:3])
#         res.frame.center = x[3:]

#         return res

#     @staticmethod
#     def dexp(x):

#         res = np.identity(Ground.dim)

#         res[:3, :3] = Quaternion.dexp(x[:3])
        
#         return res


#     def contacts(self, body, epsilon = 5e-2):
#         to = self.frame.inv()
        
#         local = {name: to(pos) for name, pos in body.items() }
#         depth = {name: pos[1] for name, pos in local.items() }

#         pairs = sorted( [ (h, name) for name, h in depth.items() if h <= epsilon] )

#         return [n for h, n in pairs]

        
#     def error(self, body, contacts):
#         c = contacts[0]

#         # body[c] should have zero y component
#         return -self.frame.inv()(body[c])[1]

#     def jacobian(self, body, contacts):
#         c = contacts[0]

#         local = self.frame.inv()(body[c])
#         up = self.frame.orient( ey )

#         res = np.zeros( (1, Ground.dim) )

#         res[0, :3] = -np.cross(local, ey)
#         res[0, 3:] = -up

#         return res

#     @coroutine
#     def estimator(self, epsilon = 1e-2, **kwargs):

#         rls = lie_rls( init = self, **kwargs )

#         state = self
        
#         while True:
#             body = yield state
            
#             contacts = self.contacts(body, epsilon)
#             # print(contacts)
            
#             if contacts:
#                 w = state.error(body, contacts)
#                 H = state.jacobian(body, contacts)

#                 state = rls.send( (w, H) )
#                 self.frame = state.frame
