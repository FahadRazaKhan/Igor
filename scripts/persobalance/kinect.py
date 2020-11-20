
from threading import Thread
# from queue import Queue

buffer_size = 2048

def client(host, port):

    import socket
    import time
    import json
    
    while True:
        try:
            print('connecting to {0}...'.format(host))
            s = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
            s.connect( (host, port) )
            print('ok')

            try:
                data = b''
                
                while True:
                    chunk = s.recv( buffer_size )
                    data += chunk

                    if data[-1] == ord('\n'):
                        yield json.loads( data[:-1].decode('ascii') )
                        s.send(b'ack')
                        data = b''
                        
            except socket.error as e:
                print('socket error:', e)
            finally:
                s.close()
        except socket.error:
            time.sleep(1)
                        


def start( host = '127.0.0.1', port = 6969 ):
    global thread

    global last_frame 
    last_frame = {}
    
    def target():

        for data in client(host, port):
            global last_frame 
            last_frame = data
            
    thread = Thread(target = target)
    
    thread.daemon = True
    thread.start()


def stop():
    thread.stop()
        
if __name__ == '__main__':

    import sys
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = '127.0.0.1'
    
    start( host )

