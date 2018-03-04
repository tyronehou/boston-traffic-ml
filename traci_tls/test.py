import os
import sys
from readtripinfo import *
# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME")))
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import numpy as np
import matplotlib.pyplot as plt

def window(lst, winlen, f):
    winlst = []
    for i in range(len(lst)-winlen+1):
        res = f(lst[i:i+winlen])
        winlst.append(res)
    return winlst

f = np.loadtxt('fixed')
d0 = np.loadtxt('qlearn_delay_episode1_rewards')
d1 = np.loadtxt('qlearn_delay_episode1_rewards')

fmean = window(f, 1000, np.mean)
d0mean = window(d0, 1000, np.mean)
d1mean = window(d1, 1000, np.mean)


episodes = 10
for i in range(episodes):
    visits = np.loadtxt('qlearn_delay_episode{}_visits'.format(i))
    plt.hist2d(visits[:, 0], visits[:, 1], bins=(28, 14))
    plt.savefig('qlearn_delay_episode{}_histogram'.format(i))

for i in range(episodes):  
    root_i = parse_xml('qlepisode{}_tripinfo.xml'.format(i)) 
    print('Cumulative Time Loss {}:'.format(i), get_loss(root_i))
    print('Average Time Loss {}:'.format(i), get_avg_delay(root_i))

print('FIXED')
root = parse_xml('f_tripinfo.xml')
print('Cumulative Time Loss:', get_loss(root))
print('Average Time Loss:', get_avg_delay(root)) 
print('Ids:', countTypes(root, 'vType'))
