
from . import wiiuse

from contextlib import contextmanager

@contextmanager
def board(timeout = 5):

    index = 0
    nmotes = 1
    
    wiimotes = wiiuse.init(nmotes)
    found = wiiuse.find(wiimotes, nmotes, timeout)

    if not found:
        raise Exception('no wiimote found')

    connected = wiiuse.connect(wiimotes, nmotes)

    if connected:
        print('Connected to %i wiimotes (of %i found).' % (connected, found))
    else:
        raise Exception('failed to connect to any wiimote.')
    
    try:
        wiiuse.set_leds(wiimotes[index], wiiuse.LED[index])
        yield wiimotes
        
    finally:
        wiiuse.set_leds(wiimotes[index], 0)
        wiiuse.disconnect(wiimotes[index])
        

def frames():

    with board() as w:
        while True:
            r = wiiuse.poll(w, 1)
            if r == 0: continue

            wm = w[0][0]

            if wm.exp.type == wiiuse.EXP_WII_BOARD:

                yield (wm.exp.u.board.tr,
                       wm.exp.u.board.br,
                       wm.exp.u.board.tl,
                       wm.exp.u.board.bl)
        

if __name__ == '__main__':

    for f in frames():
        print(f)
