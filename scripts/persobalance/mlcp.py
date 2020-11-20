import ctypes
from ctypes import CDLL
import numpy as np

libname = 'persobalance2'
soname = ctypes.util.find_library(libname) or libname
dll = CDLL(soname)

class MLCPParams(ctypes.Structure):
    _fields_ = [ ('iterations', ctypes.c_uint),
                 ('precision', ctypes.c_double),
                 ('nlnscg', ctypes.c_int)]

c_double_p = ctypes.POINTER(ctypes.c_double)

dll.mlcp_solve.restype = None
dll.mlcp_solve.argtypes = (ctypes.c_uint,
                           c_double_p,
                           c_double_p,
                           c_double_p,
                           c_double_p,
                           ctypes.POINTER(MLCPParams))

def solve(x, M, q, **kwargs):
    '''solve mixed LCP (M, q) modifying initial solution x. 

    kwargs: 
    - bilateral: bilateral constraint mask (default: None)
    - it: maximum iteration count
    - eps: solution precision
    - nlnscg: non-linear non-smooth conjugate gradient acceletation (0/1)
    '''
    
    def ptr(x):
        '''a c pointer to data buffer'''
        return ctypes.cast(np.ctypeslib.as_ctypes(x), c_double_p)
    
    px = ptr(x)
    pM = ptr(M)
    pq = ptr(q)

    bilateral = kwargs.get('bilateral', None)
    pbilateral = bilateral if bilateral is None else ptr(bilateral)
    
    n = x.shape[0]

    p = MLCPParams()

    p.iterations = kwargs.get('it', 100)
    p.precision = kwargs.get('eps', 1e-8)
    p.nlnscg = kwargs.get('nlnscg', 0)
    
    dll.mlcp_solve(n, px, pM, pq, pbilateral, p)

