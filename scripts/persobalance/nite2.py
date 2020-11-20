# monkey-patching yo !
import ctypes

# patch for nite library missing symbols
orig = ctypes.CDLL.__getattr__
def patch(self, name):
    try:
        return orig(self, name)
    except:
        print('warning, symbol not found:', name)
        return ctypes.CFUNCTYPE(None)

# disable strongly-typed enums since they cause errors with &
from primesense import utils
del utils.CEnumMeta.__new__


from primesense import nite2


# and restore
import os

_initialize = nite2.initialize


def initialize():
    old = ctypes.CDLL.__getattr__        
    ctypes.CDLL.__getattr__ = patch        
    _initialize()
    ctypes.CDLL.__getattr__ = old
    
nite2.initialize = initialize
