import numpy as np
episodes = 200
for i in range(episodes):
    visits = np.loadtxt('qlearn_delay_episode{}_visits'.format(i))
    d = {}
    for r, c in visits:
        if (r, c) not in d:
            d[(r, c)] = 1
        else:
            d[(r, c)] += 1
    with open("qlearn_delay_episode{}_state_count".format(i), 'w') as f:
        f.write(str(d))
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
