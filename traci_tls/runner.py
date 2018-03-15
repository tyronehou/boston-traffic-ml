#!/usr/bin/env python
"""
@file    runner.py
@author  Lena Kalleske
@author  Daniel Krajzewicz
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2009-03-26
@version $Id: runner.py 24864 2017-06-23 07:47:53Z behrisch $

Tutorial for traffic light control via the TraCI interface.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2017 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import argparse
import subprocess
import random
import numpy as np

# Define global tabular Q array
S = 28 * 14 # EW * N queue length maxes
A = 2
N = 10000  # number of vehicles
Q = np.zeros((S, A))
visits = []

gamma = 0.9 # discount rate
alpha = 0.1 # step size
rewards_t = []

LANEAREA_LENGTH = 200 # in meters
c = 10 # cell size in meters
NUM_LANES = 2
# Array of intersection state maps
state_rep = np.zeros((NUM_LANES*4, LANEAREA_LENGTH//c))

debug = False
# we need to import python modules from the $SUMO_HOME/tools directory
try:
    #sys.path.append(os.path.join(os.path.dirname(
    #    __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    #sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    #    os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    #sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    #    os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME")))

    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from traci import lanearea as la
from traci import vehicle as veh
from traci import trafficlight as tl

def db(*args, **kwargs):
    if debug:
        print(*args, **kwargs)

def generate_routefile(N):
    random.seed(42)  # make tests reproducible
    # demand per second from different directions
    #pWE = 1. / 5
    #pEW = 1. / 5
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 20
    pWE_end = 1. / 5
    pEW_end = 1. / 5
    pNS_end = 1. / 20

    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N//2):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
        for i in range(N//2, N):
            if random.uniform(0, 1) < pWE_end:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW_end:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS_end:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i

        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>

def run():
    """execute the TraCI control loop"""
        # we start with phase 2 where EW has green
    tl.setPhase("0", 2)
    i = 2
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print("E:", la.getLastStepHaltingNumber("0"))
        print("N:", la.getLastStepHaltingNumber("1"))
        # Get occupancy of East lane
        #print(traci.lane.getLastStepOccupancy("2i_0"))
        if tl.getPhase("0") == 2:
            # we are not already switching
            if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
                # there is a vehicle from the north, switch
                tl.setPhase("0", 3)
            else:
                # otherwise try to keep green for EW
                tl.setPhase("0", 2)
        
    traci.close()
    sys.stdout.flush()


def cumHaltingNumber():
    cum_halt_num = 0
    for det_id in la.getIDList():
        cum_halt_num += la.getLastStepHaltingNumber(det_id)
    return cum_halt_num

def getCumulativeDelay():
    # Get each vehicle in lanearea
    delay = 0
    for det_id in la.getIDList():
        for vid in la.getLastStepVehicleIDs(det_id):
            delay += veh.getAccumulatedWaitingTime(vid)

    return delay

def getState():
    DTSE()
    
    EW = la.getLastStepHaltingNumber("0")
    EW += la.getLastStepHaltingNumber("1")
    N = la.getLastStepHaltingNumber("2")
    N += la.getLastStepHaltingNumber("3")

    sm = stateMap(EW, N)
    visits.append((EW, N))
    return sm
 
def getReward():
    return -getCumulativeDelay()
    #return -cumHaltingNumber()

def stateMap(EW, NS):
    return 28 * NS + EW

def DTSE():
    # Get all vehicles in the lanearea
    # Given a list of relevant vehicles, convert it t
    for det_id in la.getIDList():
        det_len = la.getLength(det_id)
        for vid in la.getLastStepVehicleIDs(det_id):
            pos = veh.getLanePosition(vid)
            print("Veh {}: {}".format(vid, pos))
            
            # Given position, change it to an index for a binary array
            
            #state_rep[] = 1
            #state_rep = 


# Use epsilon-greedy action selection
def selectAction(state_idx, epsilon):
    seed = np.random.random()
    if seed < epsilon: # select random actions
        return np.random.choice(A, 1)[0]
    else: # select greedy action
        return np.argmax(Q[state_idx])

def fixed():
    tl.setPhase("0", 2)
    getState()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        getState()
        r = getReward()
        rewards_t.append(r)

def myfixed(greentime, yellowtime):
    tl.setPhase("0", 2)
    getState()
    while traci.simulation.getMinExpectedNumber() > 0:
        for i in range(greentime):
            traci.simulationStep()
            tl.setPhase("0", 2)
            getState()
            r = getReward()
            rewards_t.append(r)

        for i in range(yellowtime):
            traci.simulationStep()
            tl.setPhase("0", 3)
            getState()
            r = getReward()
            rewards_t.append(r)
 
        for i in range(greentime):
            traci.simulationStep()
            tl.setPhase("0", 0)
            getState()
            r = getReward()
            rewards_t.append(r)
 
        for i in range(yellowtime):
            traci.simulationStep()
            tl.setPhase("0", 1)
            getState()
            r = getReward()
            rewards_t.append(r)

def qlearn(epsilon, load_from_file=None, learn=True, anneal=False):
    print("File beginning", flush=True)
    global Q
    if load_from_file:
        Q = load_Q_values(load_from_file)
    
    action_time = 12
    traci.simulationStep()
    tl.setPhase("0", 2)
    # Find current state
    s = getState()

    i = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        # Take action 
        tl.setPhaseDuration("0", 10000)
        a = selectAction(s, epsilon)

        if a*2 != tl.getPhase("0"): # Change action
            db('Change to other green light')
            tl.setPhase("0", (tl.getPhase("0")+1) % 4)
            #print("phase:", tl.getPhase("0"))
            yellow_time = (tl.getNextSwitch("0") - traci.simulation.getCurrentTime()) // 1000
        else:
            yellow_time = 0

        #print('t:', traci.simulation.getCurrentTime() / 1000)

        if i % 100 == 0:
            print("t:", traci.simulation.getCurrentTime() / 1000)
            print('epsilon:', epsilon)
        db('action:', a)
        r = 0

        # Move simulation forward one step and obtain reward
        db(action_time + yellow_time)
        for i in range(yellow_time + action_time):
            traci.simulationStep()
            # Get some kind of reward signal from the E2 loops
            # Let's say its negative of the number of cars on the loop
            r += getReward()#-cumHaltingNumber()
            rewards_t.append(r)

            if traci.simulation.getMinExpectedNumber() <= 0:
                return
        
        db("reward:", r)
        s_ = getState()

        if learn:
            # update Q function according to the Q-learning rule
            # Maybe try Sarsa and Expected Sarsa at some point
            Q[s][a] += alpha * (r + gamma * np.max(Q[s_][a]) - Q[s][a])

            if anneal:
                epsilon -= epsilon * i / N
                i += 1
        s = s_
        db("-" * 10)

def save_Q_values(fname):
    np.savetxt(fname, Q)

def load_Q_values(fname):
    return np.loadtxt(fname)

def save_output(fname, episode=None):
    if episode is not None:
        np.savetxt('{}_episode{}_rewards'.format(args.saverewards, episode), rewards_t)
        np.savetxt('{}_episode{}_visits'.format(args.saverewards, episode), visits)
    else:
        np.savetxt('{}_rewards'.format(args.saverewards), rewards_t)
        np.savetxt('{}_visits'.format(args.saverewards), visits)

def get_arguments():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    argParser.add_argument("--saverewards")
    argParser.add_argument("--loadfromfile")
    argParser.add_argument("--savetofile")
    argParser.add_argument("--method")
    argParser.add_argument("--epsilon", type=float, default=0.0)

    args = argParser.parse_args()
    return args

# this is the main entry point of this script
if __name__ == "__main__":
    args = get_arguments()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run

    if args.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile(N)
        
    if args.method == 'ql':
        print('Using epsilon-greedy Qlearning controller')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                  "--tripinfo-output", "{}_tripinfo.xml".format(args.method),
                                  "--error-log", "errors"])

        qlearn(args.epsilon, args.loadfromfile, anneal=False) # don't load from file, do learn

        if args.savetofile:
            save_Q_values(args.savetofile)
        if args.saverewards:
            save_output(args.saverewards)

    elif args.method == 'qle':
        print('Using epsilon-greedy Qlearning controller')
        episodes = 10
        for i in range(episodes):
            # this is the normal way of using traci. sumo is started as a
            # subprocess and then the python script connects and runs
            traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                      "--tripinfo-output", "{}episode{}_tripinfo.xml".format(args.method, str(i)),
                                      "--error-log", "errors"])

            print("Starting episode", i)
            if args.loadfromfile:
                if i > 0:
                    args.loadfromfile += "_epsiode" + str(i-1)
                else:
                    args.loadfromfile = None
        
            qlearn(args.epsilon, args.loadfromfile, anneal=True) # don't load from file, do learn

            if args.savetofile:
                save_Q_values('{}_episode{}'.format(args.savetofile, str(i)))
            if args.saverewards:
                save_output(args.saverewards, i)
    elif args.method == 'f':
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        min_green_time = 10
        max_green_time = 60
        #min_green_time = 25
        #max_green_time = 25
        inc = 5
        for gt in range(min_green_time, max_green_time+1, inc):
            _gt = gt
            traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                     "--tripinfo-output", "{}green={}_tripinfo.xml".format(args.method, _gt),
                                     "--error-log", "errors"])
            
            print('Used fixed controller with cycle ({0},{1},{0},{1})'.format(gt, 6))
            myfixed(_gt, 6)
            if args.saverewards:
                save_output(args.saverewards, _gt)

    elif args.method == 'qg':
        print('Using greedy Q-learning controller')

        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                  "--tripinfo-output", "{}_tripinfo.xml".format(args.method),
                                  "--error-log", "errors"])

        qlearn(0, args.loadfromfile, learn=False)
        # Truly greedy should actually be zero
        if args.saverewards:
            save_output(args.saverewards)
    else:
        print('Invalid method selected')
            
    exit(0)
