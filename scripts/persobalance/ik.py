
import numpy as np

from collections import namedtuple

from .snap.math import *
from .snap import viewer, gl, tool
from . import graph

# TODO: Body/Joints/Constraints should fill the graph directly

# TODO joint nullspace (subclass)

class Body(object):

    def update(self): pass
    
    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Body({0})'.format(self.name)


    def draw(self):
        with gl.frame(self.dofs):
            gl.glScale(* (self.dim  /2))
            gl.cube()
    

    def __init__(self, **kwargs):

        self.dim = kwargs.get('dim', np.ones(3))
        self.dofs = kwargs.get('dofs', Rigid3())
        self.name = kwargs.get('name', 'unnamed body')
        
        volume = self.dim[0] * self.dim[1] * self.dim[2]

        mass = volume

        dim2 = self.dim * self.dim
        sum_dim2 = sum(dim2)

        self.mass = mass
        self.inertia = mass / 12.0 * (sum_dim2 - dim2)

        self.inertia_tensor = np.zeros( (6, 6) )
        
        self.inertia_tensor[:3, :3] = np.diag( self.inertia )
        self.inertia_tensor[3:, 3:] = mass * np.identity(3)
        
        
    def insert(self, graph):
        self.vertex = graph.add_vertex(6)

    def fill_vector(self, dt):
        self.vertex.vector = 0
        
    def fill_matrix(self, dt):
        self.vertex.matrix = self.inertia_tensor

    def step(self, dt):
        self.dofs[:] = self.dofs * Rigid3.exp( dt * self.vertex.vector )
        
        # delta = Rigid3()

        # delta.orient = Quaternion.exp( dt * self.vertex.vector[:3] )
        # delta.center = dt * self.vertex.vector[3:]

        # self.dofs.center = self.dofs.center + self.dofs.orient(delta.center)
        # self.dofs.orient = self.dofs.orient * delta.orient

    
        
class Joint:

    __slots__ = ['parent', 'child',  'name',  'nullspace',  'compliance', '_local',
                 'vertex', 'ep', 'ec']


    def __init__(self, parent, child, name, nullspace, compliance = 1):
        self.parent = parent
        self.child = child
        self.name = name
        self.nullspace = nullspace
        self.compliance = compliance

        self._local = (parent[1].center / parent[0].dim,
                       child[1].center / child[0].dim)


    def update(self):
        '''update local frames when body dim changes'''

        self.parent[1].center = self._local[0] * self.parent[0].dim
        self.child[1].center = self._local[1] * self.child[0].dim
        

    def __hash__(self):
        return id(self)


    def __str__(self):
        return 'Joint({0}, {1})'.format(self.parent[0].name, self.child[0].name)


    def insert(self, graph):
        rows, cols = self.nullspace.shape
        self.vertex = graph.add_vertex(rows)
        
        self.ep = graph.add_edge(self.vertex, self.parent[0].vertex)
        self.ec = graph.add_edge(self.vertex, self.child[0].vertex)        

        
    def fill_vector(self, dt):
        self.vertex.vector = self.error() / dt

    def fill_matrix(self, dt):

        self.ep.matrix, self.ec.matrix = self.jacobian()
        self.vertex.matrix = self.compliance * (-1.0/ (dt * dt))

    def jacobian(self, pdofs = None, cdofs = None):
        '''constraint jacobian'''

        pdofs = pdofs if pdofs is not None else self.parent[0].dofs
        cdofs = cdofs if cdofs is not None else self.child[0].dofs        
        
        p = pdofs * self.parent[1]
        c = cdofs * self.child[1]        
        
        r = p.inv() * c
        
        dp = self.parent[1].inv().Ad()
        dc = self.child[1].inv().Ad()

        return -self.nullspace.dot(r.inv().Ad().dot(dp)), self.nullspace.dot(dc)
    
    def error(self):
        '''constraint violation'''

        p = self.parent[0].dofs * self.parent[1]
        c = self.child[0].dofs * self.child[1]        
        
        r = p.inv() * c

        twist = r.log()

        return -self.nullspace.dot(twist)
        
    
class Constraint:
    '''an ik constraint'''

    __slots__ = ['body',  'local',  'target',  'stiffness', '_local',
                 'vertex', 'edge']

    def __init__(self, body, local, target, stiffness):
        self.body = body
        self.local = local
        self.target = target
        self.stiffness = stiffness

        self._local = local / body.dim

    def update(self):
        '''update local coords when body dim changes'''
        self.local = self._local * self.body.dim
    
    def __hash__(self):
        return id(self)

    def jacobian(self):
        # TODO this is constant, optimize
        return Rigid3(center = self.local).inv().Ad()[3:, :]

    def error(self):
        return self.body.dofs.orient.inv()(self.target - self.body.dofs(self.local))
    

    def insert(self, graph):
        self.vertex = graph.add_vertex(3)
        self.edge = graph.add_edge(self.vertex, self.body.vertex)

    def fill_matrix(self, dt):
        self.edge.matrix = self.jacobian()
        self.vertex.matrix = (-1.0/(self.stiffness * dt * dt)) * np.identity(3)
        
    def fill_vector(self, dt):
        self.vertex.vector = self.error() / dt
        

        
class Skeleton(namedtuple('Skeleton', 'bodies joints')):

    
    def draw(self):
        for b in self.bodies:
            b.draw()

        
    def insert(self, graph):
        for b in self.bodies:
            b.insert(graph)

        for j in self.joints:
            j.insert(graph)
            

    def fill_matrix(self, dt):
        
        for b in self.bodies:
            b.fill_matrix(dt)

        for j in self.joints:
            j.fill_matrix(dt)

            
    def fill_vector(self, dt):
        
        for b in self.bodies:
            b.fill_vector(dt)

        for j in self.joints:
            j.fill_vector(dt)
            
            
    @staticmethod
    def human(**kwargs):

        mass = 1
        inertia = np.ones(3)
        dim = np.ones(3)

        rho = 1.0
        stiffness = kwargs.get('stiffness', 1.0)
        
        def body(**kwargs):
            kwargs.setdefault('name', 'unnamed body')

            d = kwargs.setdefault('dim', dim)
            volume = d[0] * d[1] * d[2]

            m = rho * volume

            d2 = d * d
            inertia = m * (sum(d2) - d2) / 12.0

            kwargs.setdefault('mass', m )
            kwargs.setdefault('inertia', inertia)

            kwargs.setdefault('dofs', Rigid3() )                        

            return Body(**kwargs)


        def joint(*args, **kwargs):
            kwargs.setdefault('name', 'unnamed joint')
            kwargs.setdefault('nullspace', np.identity(6))
            return Joint(*args, **kwargs)


        def spherical(*args, **kwargs):
            kwargs.setdefault('name', 'unnamed joint')

            # nullspace = np.zeros( (3, 6) )
            # nullspace[:, 3:] = np.identity(3)
            # compliance = 1-3 * np.identity(3)

            nullspace = np.identity(6)
            compliance = kwargs.get('compliance', 1) * np.identity(6)
            compliance[3:, 3:] = 0 * np.identity(3)

            kwargs['nullspace'] = nullspace            
            kwargs['compliance'] = compliance

            return Joint(*args, **kwargs)


        def hinge(*args, **kwargs):
            kwargs.setdefault('name', 'unnamed joint')

            axis = kwargs.pop('axis', vec(1, 0, 0))

            nullspace = np.identity(6)
            kwargs['nullspace'] = nullspace

            compliance = np.zeros( (6, 6) )

            compliance[:3, :3] = kwargs.get('compliance', 1) * np.outer(axis, axis)
            compliance[3:, 3:] = 1e-7 * np.identity(3)

            kwargs['compliance'] = compliance

            return Joint(*args, **kwargs)


        # bodies
        size = 1

        head_size = kwargs.get('head_size', 1.0)
        torso_size = kwargs.get('torso_size', 2.0)

        waist_size = kwargs.get('waist_size', 1.0)
        
        arm_size = kwargs.get('arm_size', 2.0)
        forearm_size = kwargs.get('forearm_size', 2.0)        
        femur_size = kwargs.get('femur_size', 3.0)
        tibia_size = kwargs.get('tibia_size', 2.0)
        foot_size = kwargs.get('foot_size', 1.5)            

        
        head = body(name = 'head', dim = vec(0.9 * head_size, head_size, head_size) )
        torso = body(name = 'torso', dim = vec(torso_size,
                                               torso_size,
                                               1 * torso_size / 3.0) )
        waist = body(name = 'waist', dim = vec(3 / 4 * waist_size,
                                               waist_size,
                                               1 * waist_size / 3.0) )


        arm_dim = vec(arm_size / 4.0, arm_size, arm_size / 4.0)
        larm = body(name = 'larm', dim = arm_dim)
        rarm = body(name = 'rarm', dim = arm_dim )    

        forearm_dim = vec(forearm_size / 4.0, forearm_size, forearm_size / 4.0)
        lforearm = body(name = 'lforearm', dim = forearm_dim)
        rforearm = body(name = 'rforearm', dim = forearm_dim)

        femur_dim = vec(femur_size / 5.0, femur_size, femur_size / 5.0)
        lfemur = body(name = 'lfemur', dim = femur_dim)
        rfemur = body(name = 'rfemur', dim = femur_dim)

        tibia_dim = vec(tibia_size / 7.0, tibia_size, tibia_size / 7.0)
        ltibia = body(name = 'ltibia', dim = tibia_dim)
        rtibia = body(name = 'rtibia', dim = tibia_dim)

        foot_dim = vec(foot_size / 1.5, foot_size, foot_size / 2.0)
        lfoot = body(name = 'lfoot', dim = foot_dim)
        rfoot = body(name = 'rfoot', dim = foot_dim)

 
        bodies = [head, torso, waist,
                  larm, rarm,
                  lforearm, rforearm,
                  lfemur, rfemur,
                  ltibia, rtibia,
                  lfoot, rfoot]


        # joints
        neck = spherical( (torso, Rigid3(center = vec(0, 6 * torso.dim[1] / 5, - torso.dim[2] / 2))),
                          (head, Rigid3(center = vec(0, 0, -head.dim[2] / 2))),
                          name = 'neck',
                          compliance = 1.0 / stiffness)

        spine = spherical( (torso, Rigid3(center = vec(0, - torso.dim[1] / 2, 0))),
                          (waist, Rigid3(center = vec(0, waist.dim[1] / 2, 0))),
                          name = 'spine',
                          compliance = 1.0 / stiffness)
        
        

        lshoulder = spherical( (torso, Rigid3(orient = Quaternion.exp( math.pi / 4 * ez),
                                              center = vec(3 * torso.dim[0] / 5,
                                                           torso.dim[1] / 2,
                                                           0))),
                               (larm, Rigid3(center = vec(0, larm.dim[1] / 2, 0))),
                               name = 'lshoulder',
                          compliance = 1.0 / stiffness )

        rshoulder = spherical( (torso, Rigid3(orient = Quaternion.exp( -math.pi / 4 * ez),
                                              center = vec(-3 * torso.dim[0] / 5,
                                                           torso.dim[1] / 2,
                                                           0))),
                               (rarm, Rigid3(center = vec(0, rarm.dim[1] / 2, 0))),
                               name = 'rshoulder',
                          compliance = 1.0 / stiffness )

        relbow = hinge( (rarm, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                      center = vec(0, -rarm.dim[1] / 2, 0))),
                            (rforearm, Rigid3(center = vec(0, rforearm.dim[1] / 2, 0))),
                            name = 'relbow',
                          compliance = 1.0 / stiffness )

        lelbow = hinge( (larm, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                      center = vec(0, -larm.dim[1] / 2, 0))),
                            (lforearm, Rigid3(center = vec(0, lforearm.dim[1] / 2, 0))),
                            name = 'lelbow',
                          compliance = 1.0 / stiffness)


        lhip = spherical( (waist, Rigid3( center = vec(waist.dim[0] / 2,
                                                      -waist.dim[1] / 2,
                                                      0))),
                          (lfemur, Rigid3(center = vec(0, lfemur.dim[1] / 2, 0))),
                          name = 'lhip',
                          compliance = 1.0 / stiffness)


        rhip = spherical( (waist, Rigid3( center = vec(-waist.dim[0] / 2,
                                                      -waist.dim[1] / 2,
                                                      0))),
                          (rfemur, Rigid3(center = vec(0, rfemur.dim[1] / 2, 0))),
                          name = 'rhip',
                          compliance = 1.0 / stiffness)

        rknee = hinge( (rfemur, Rigid3(orient = Quaternion.exp( math.pi / 5 * ex),
                                       center = vec(0, -rfemur.dim[1] / 2, 0))),
                       (rtibia, Rigid3(center = vec(0, rtibia.dim[1] / 2, 0))),
                       name = 'rknee',
                          compliance = 1.0 / stiffness )

        lknee = hinge( (lfemur, Rigid3(orient = Quaternion.exp(math.pi / 5 * ex),
                                       center = vec(0, -lfemur.dim[1] / 2, 0))),
                       (ltibia, Rigid3(center = vec(0, ltibia.dim[1] / 2, 0))),
                       name = 'lknee',
                          compliance = 1.0 / stiffness)


        rankle = spherical( (rtibia, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                            center = vec(0, - rtibia.dim[1] / 2, 0))),
                            (rfoot, Rigid3(center = vec(0,  rfoot.dim[1] / 2, 0))),
                           name = 'rankle',
                          compliance = 1.0 / stiffness )

        lankle = spherical( (ltibia, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                           center = vec(0, -ltibia.dim[1] / 2, 0))),
                            (lfoot, Rigid3(center = vec(0, lfoot.dim[1] / 2, 0))),
                            name = 'lankle',
                          compliance = 1.0 / stiffness )



        joints = [neck, spine,
                  lshoulder, rshoulder,
                  relbow, lelbow,
                  lhip, rhip,
                  lknee, rknee,
                  lankle, rankle]


        constraints = []

        skeleton = Skeleton(bodies, joints)

        # some handy shortcuts
        skeleton.head = head
        skeleton.torso = torso
        skeleton.waist = waist
        
        skeleton.larm = larm
        skeleton.lforearm = lforearm
        skeleton.lfemur = lfemur
        skeleton.ltibia = ltibia
        skeleton.lfoot = lfoot

        skeleton.rarm = rarm
        skeleton.rforearm = rforearm
        skeleton.rfemur = rfemur
        skeleton.rtibia = rtibia
        skeleton.rfoot = rfoot


        skeleton.neck = neck
        skeleton.spine = spine
        
        skeleton.lshoulder = lshoulder
        skeleton.lelbow = lelbow
        skeleton.lhip = lhip
        skeleton.lknee = lknee
        skeleton.lankle = lankle

        skeleton.rshoulder = rshoulder
        skeleton.relbow = relbow
        skeleton.rhip = rhip
        skeleton.rknee = rknee
        skeleton.rankle = rankle
        
        return skeleton

                
    def step(self, dt = 1.0):

        for b in self.bodies:
            b.step(dt)



def solver(skeleton, constraints, **kwargs):

    gs = kwargs.get('gs', False)
    dt = kwargs.get('dt', 1.0)    
    root = kwargs.get('root', 0)
    
    g = graph.Graph()
    
    while True:

        # clear
        g.clear()

        # insert
        skeleton.insert(g)
        for c in constraints:
            c.insert(g)

        # allocate storage
        g.init()
        
        # fill matrix
        skeleton.fill_matrix(dt)
        for c in constraints:
            c.fill_matrix(dt)

        # factor
        g.factor(root)

        # fill vector
        skeleton.fill_vector(dt)
        for c in constraints:
            c.fill_vector(dt)
            
        # solve
        g.solve()

        # step
        skeleton.step(dt)
        
        yield

        
        
def draw():

    gl.glColor(1, 1, 1)
    skeleton.draw()

    gl.glLineWidth(4)
    gl.glPointSize(6)
    
    for c in constraints:
        with gl.disable(gl.GL_LIGHTING):

            gl.glColor(1, 0, 0)            
            with gl.begin(gl.GL_POINTS):
                gl.glVertex(c.target)

            gl.glColor(1, 1, 0)
            with gl.begin(gl.GL_LINES):
                start = c.body.dofs(c.local)
                end = c.target

                gl.glVertex(start)
                gl.glVertex(end)
                
    
def keypress(key):
    if key == ' ':
        next(s)
        return True


def animate():
    try:
        next(s)
    except StopIteration:
        import sys
        sys.exit(1)


@tool.coroutine
def dragger(c):

    while True:
        pos = yield
        c.target[:] = pos

        
def select(p):
    
    for c in constraints:

        # print(c.body.name, norm(p - c.target))
        if norm(p - c.target) < 0.2:
            global on_drag
            on_drag = dragger(c)


def drag(p):
    if on_drag: on_drag.send(p)

    
if __name__ == '__main__':
    on_drag = None


    skeleton = Skeleton.human()

    
    # constraints
    stiffness = 1e2


    constraints = ConstraintList()
    
    constraints += [
        Constraint(skeleton.lforearm, vec(0, -skeleton.lforearm.dim[1] / 2, 0),
                   vec(-2, 3, 1),
                   stiffness),
        Constraint(skeleton.rforearm, vec(0, -skeleton.rforearm.dim[1] / 2, 0),
                   vec(2, 3, 1),
                   stiffness),
        Constraint(skeleton.lfoot, vec(0, -skeleton.lfoot.dim[1] / 2, 0),
                   vec(-2, -2, 0),
                   stiffness),
        Constraint(skeleton.rfoot, vec(0, -skeleton.rfoot.dim[1] / 2, 0),
                   vec(2, -2, 0),
                   stiffness),
    ]
    
    graph = Graph([], [], {})


    skeleton.fill_graph( graph )
    constraints.fill_graph(graph)
    
    root_vertex = graph.helper[skeleton.bodies[1]]
    
    forward = graph.orient( root_vertex )


    s = solver(skeleton, graph, forward, 0.5)
    
    viewer.run()
