#!/bin/env/python3

import json

def calibration(data):

    for side in 'l':

        leg = sum( [data[side + part] for part in ['toes', 'foot', 'knee']] )
        leg += data[side + 'hip'] / 3.0

        arm = sum( [data[side + part] for part in ('hand', 'elbow')] )
        arm += data[side + 'shoulder'] / 3.0

        print( side + 'leg:', 100 * leg, 'ref:', 17)
        print( side + 'arm:', 100 * arm, 'ref:', 5.3)

    head = data['head'] + data['neck'] / 4.0

    trunk = ( 2 * (data['rhip'] + data['lhip']) / 3 + 
              2 * (data['rshoulder'] + data['lshoulder']) / 3 +
              data['torso'] + 3 * data['neck'] / 4.0 )

    print('head:', 100 * head, 'ref:', 6.8)
    print('trunk', 100 * trunk, 'ref:', 43)
    
    print('neck:',  data['neck'])
    print('head:',  data['head'])

    
if __name__ == '__main__':
    import sys
    filename = sys.argv[1]

    with open(filename) as f:
        data = json.loads( f.read() )['body']
        calibration(data)

   

    import matplotlib
    from matplotlib import pyplot as plt

    with open('residual.json') as f:
        residual = json.loads( f.read() )

        
    plt.plot(residual)
    plt.show()
