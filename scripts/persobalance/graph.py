import ctypes

from ctypes import CDLL, util
import numpy as np

libname = 'persobalance2'
soname = ctypes.util.find_library(libname) or libname
dll = CDLL(soname)

vertex = ctypes.c_ulong

graph_p = ctypes.c_void_p

graph_data_p = ctypes.POINTER(ctypes.c_double)
graph_data_pp = ctypes.POINTER(graph_data_p)

dll.graph_new.restype = graph_p
dll.graph_new.argtypes = []

dll.graph_delete.restype = None
dll.graph_delete.argtypes = [graph_p]

dll.graph_clear.restype = None
dll.graph_clear.argtypes = [graph_p]

dll.graph_num_vertices.restype = ctypes.c_uint
dll.graph_num_vertices.argtypes = [graph_p] 

dll.graph_vertex_dim.restype = ctypes.c_uint
dll.graph_vertex_dim.argtypes = [graph_p, vertex] 

dll.graph_add_vertex.restype = vertex
dll.graph_add_vertex.argtypes = [graph_p, ctypes.c_uint]

dll.graph_add_edge.restype = None
dll.graph_add_edge.argtypes = [graph_p, vertex, vertex]

dll.graph_init.restype = None
dll.graph_init.argtypes = [graph_p]

dll.graph_vertex_matrix.restype = None
dll.graph_vertex_matrix.argtypes = [graph_p, vertex, graph_data_pp]

dll.graph_vertex_vector.restype = None
dll.graph_vertex_vector.argtypes = [graph_p, vertex, graph_data_pp]

dll.graph_edge_matrix.restype = None
dll.graph_edge_matrix.argtypes = [graph_p, vertex, vertex, graph_data_pp]

dll.graph_factor.restype = None
dll.graph_factor.argtypes = [graph_p, ctypes.c_uint]

dll.graph_solve.restype = None
dll.graph_solve.argtypes = [graph_p]


class Vertex:

    __slots__ = ('_as_parameter_', 'graph')

    def __init__(self, g, v):
        self.graph = g
        self._as_parameter_ = v

    @property
    def index(self):
        return self._as_parameter_
    
    @property
    def dim(self):
        return dll.graph_vertex_dim(self.graph, self)

    @property
    def vector(self):
        ptr = graph_data_p()

        dll.graph_vertex_vector(self.graph, self, ctypes.pointer(ptr))
        dim = self.dim

        return np.ctypeslib.as_array(ptr, (dim, ))

    @vector.setter
    def vector(self, value):
        self.vector[:] = value
        
    
    @property
    def matrix(self):
        ptr = graph_data_p()

        dll.graph_vertex_matrix(self.graph, self, ctypes.pointer(ptr))
        dim = self.dim

        return np.ctypeslib.as_array(ptr, (dim, dim))


    @matrix.setter
    def matrix(self, value):
        self.matrix[:, :] = value

    
    def __gt__(self, other):
        return self._as_parameter_ > other._as_parameter_



class Edge:

    __slots__ = ('source', 'target', 'graph')

    def __init__(self, graph, source, target):
        self.graph = graph
        self.source = source
        self.target = target

        assert self.source > self.target

    @property
    def matrix(self):
        ptr = graph_data_p()

        dll.graph_edge_matrix(self.graph, self.source, self.target, ctypes.pointer(ptr))

        rows = self.source.dim
        cols = self.target.dim

        return np.ctypeslib.as_array(ptr, (rows, cols))
    
    @matrix.setter
    def matrix(self, value):
        self.matrix[:, :] = value

    

        
class Graph:
    _dll = dll
    
    __slots__ = ('_as_parameter_',)
    
    def __init__(self):
        self._as_parameter_ = dll.graph_new()

    def __del__(self):
        Graph._dll.graph_delete(self)

    def __len__(self):
        return dll.graph_num_vertices(self)

    def clear(self):
        dll.graph_clear(self)
    
    def add_vertex(self, dim):
        v = dll.graph_add_vertex(self, dim)
        return Vertex(self, v)
    

    def add_edge(self, src, dst):
        assert src > dst
        dll.graph_add_edge(self, src, dst)
        return Edge(self, src, dst)

    def init(self):
        dll.graph_init(self)

    def solve(self):
        dll.graph_solve(self)

    def factor(self, root):
        dll.graph_factor(self, root)

    
