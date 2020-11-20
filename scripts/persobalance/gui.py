
from PySide2 import QtGui
from PySide2 import QtWidgets
import sys

from . import snap
from .snap.math import *


__main__ = sys.modules['__main__']

from contextlib import contextmanager

@contextmanager
def app():
    app = QtWidgets.QApplication(sys.argv)

    try:
        yield app
    finally:
        sys.exit( app.exec_() )

@contextmanager
def noexcept():
    import traceback
    try:
        yield
    except Exception as e:
        traceback.print_exc()
        sys.exit(1)


def console(local):

    from threading import Thread
    
    def target(local):
        import code
        code.interact(local = local)
        a.quit()
        
    thread = Thread(target = target, args = (local, ))
    thread.daemon = True
    thread.start()
        
def run():

    class Viewer(snap.Viewer):

        def init(self):
            with noexcept():
                __main__.init()
                
        def animate(self):
            with noexcept():
                __main__.animate()
            
        def draw(self):
            with noexcept():
                __main__.draw()


        def keyPressEvent(self, e):
            with noexcept():
                if not __main__.keypress(e.text()):
                    snap.Viewer.keyPressEvent(self, e)

    with app() as a:
        w = Viewer()

        kinect_camera = snap.Camera( w )

        ratio = 1920.0 / 1080.0
        hfov = 84.1 / deg

        kinect_camera.ratio = ratio
        kinect_camera.vfov = 2.0 * math.atan( math.tan(hfov / 2.0) / ratio ) * deg

        w.show_axis = False

        # kinect camera
        kinect_camera.frame.orient.imag = ey
        kinect_camera.frame.orient.real = 0
        kinect_camera.frame.center[2] = 0
        
        old_camera = w.camera

        old_camera.frame.orient.imag = ey
        old_camera.frame.orient.real = 0
        old_camera.frame.center[2] = -2
        
        w.camera = kinect_camera
        
        w.animation.start()

        height = 640
        w.resize( height * ratio, height)
        w.move(0,0)

        global viewer
        viewer = lambda : w

        def _switch_cameras():
            ratio = w.camera.ratio
            w.camera = old_camera if w.camera is kinect_camera else kinect_camera
            w.camera.ratio = ratio
            w.show_axis = (w.camera is old_camera)

            
        global toggle_camera
        toggle_camera = _switch_cameras
        
        # w.showFullScreen()
        w.show()
        w.setWindowTitle('PersoBalance2: ' + __main__.__doc__ )

    
    
