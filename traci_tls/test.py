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

#f = np.loadtxt('fixed_episode40_rewards')
#d0 = np.loadtxt('qlearn_delay_episode0_rewards')
#d9 = np.loadtxt('qlearn_delay_episode9_rewards')
#
#fmean = window(f, 1000, np.mean)
#d0mean = window(d0, 1000, np.mean)
#d1mean = window(d1, 1000, np.mean)


episodes = 10
#for i in range(2, episodes):
#    visits = np.loadtxt('qlearn_delay_episode{}_visits'.format(i))
#    d = {}
#    for r, c in visits:
#        if (r, c) not in d:
#            d[(r, c)] = 1
#        else:
#            d[(r, c)] += 1
#    with open("qlearn_delay_episode{}_state_count".format(i), 'w') as f:
#        f.write(str(d))
#for i in range(10, 60, 5):
#    visits = np.loadtxt('fixed_episode{}_visits'.format(i))
#    d = {}
#    for r, c in visits:
#        if (r, c) not in d:
#            d[(r, c)] = 1
#        else:
#            d[(r, c)] += 1
#    with open("fixed_episode{}_state_count".format(i), 'w') as f:
#        f.write(str(d))
#
#    #visits = visits[visits.any(axis=1)]
#    plt.hist2d(visits[:, 0], visits[:, 1], bins=(28, 14))
#    plt.savefig('qlearn_delay_episode{}_histogram'.format(i))

result_dir = 'results_10_11_20'
result_dir = 'results_qew=.2'
result_dir = '.'
print('Qlearn')
#for i in range(2, episodes):  
#    root_i = parse_xml('{}/qleepisode{}_tripinfo.xml'.format(result_dir, i)) 
#    print('Episode', i)
#    print('Cumulative Time Loss {}:'.format(i), get_cum_attrib(root_i, "timeLoss"))
#    print('Average Time Loss {}:'.format(i), get_avg_attrib(root_i, "timeLoss"))
#    print('Cumulative Travel Time:', cumTravelTime(root_i))
#    print('Average Travel Time:', avgTravelTime(root_i))

root = parse_xml('{}/ql_tripinfo.xml'.format(result_dir))
print('Cumulative Time Loss:', get_cum_attrib(root, "timeLoss"))
print('Average Time Loss:', get_avg_attrib(root, "timeLoss"))
print('Cumulative Travel Time:', cumTravelTime(root))
print('Average Travel Time:', avgTravelTime(root))

print('FIXED')
for i in range(10, 60, 5):
    try:
        root = parse_xml('{}/fgreen={}_tripinfo.xml'.format(result_dir, i))
    except FileNotFoundError:
        print("File not found")
        continue
    print(i)
    print('Cumulative Time Loss:', get_cum_attrib(root, "timeLoss"))
    print('Average Time Loss:', get_avg_attrib(root, "timeLoss")) 
    print('Cumulative Travel Time:', cumTravelTime(root))
    print('Average Travel Time:', avgTravelTime(root))

    print('Ids:', countTypes(root, 'vType'))
