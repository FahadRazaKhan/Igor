
# reset interrupt handler for sigint
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

import sys
import os

# reopen stdout w/ line-buffering (fixes windows)
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 1)
sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', 1)

# disable opengl error checking
import OpenGL
OpenGL.ERROR_CHECKING = False


def path():
    return os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))



