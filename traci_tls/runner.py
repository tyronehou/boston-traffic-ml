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
from tabular_ql import QLC
from dtc import ReplayMemory, DTC, optimize_model
import torch
import torch.optim as optim
from tensorboardX import SummaryWriter

S = 28 * 14 * 4# EW * N queue length maxes
#S = 28 * 14
A = 2
N = 10000  # number of vehicles
visits = []
vehicles_in_queue = {} # maps vehicles to the time they entered the queue

gamma = 0.7 # discount rate
alpha = 0.1 # step size
rewards_t = []

LANEAREA_LENGTH = 200 # in meters DON"T FORGET TO CHANGE IF DET2 CHANGES
NUM_LANES = 1 # we only track the incoming lanes
c = 8 # cell size in meters
num_cells = int(np.ceil(LANEAREA_LENGTH / c)) # number of cells 
beta = 0.001 # target net parameter update rate

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
from traci import lane
from traci import vehicle as veh
from traci import trafficlight as tl
from traci import simulation as sim

def db(*args, **kwargs):
    if debug:
        print(*args, **kwargs)

# Make this read route config file
def generate_routefile(N):
    random.seed(42)  # make tests reproducible
    # demand per second from different directions
    #pWE = 1. / 5
    #pEW = 1. / 5
    pWE = 1. / 10
    pEW = 1. / 10
    pNS = 1. / 20
    pWE_end = 1. / 5
    pEW_end = 1. / 5
    pNS_end = 1. / 20

    with open("data/single/cross.rou.xml", "w") as routes:
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

def getTimeInQueue():
    t = sim.getCurrentTime() / 1000
    cumTime = 0
    for det_id in la.getIDList():
        for vid in la.getLastStepVehicleIDs(det_id):
            print('vid in getTimeinQueue:',vid)
            start = vehicles_in_queue.setdefault(vid, t)
            cumTime += t - vehicles_in_queue[vid]
    print(vehicles_in_queue)
    print(cumTime)
    return cumTime

def getState():
    #EW = len(la.getLastStepVehicleIDs("0"))#.getLastStepHaltingNumber("0")
    #EW += len(la.getLastStepVehicleIDs("1"))#.getLastStepHaltingNumber("1")
    #N = len(la.getLastStepVehicleIDs("2"))#.getLastStepHaltingNumber("2")
    #N += len(la.getLastStepVehicleIDs("3"))#.getLastStepHaltingNumber("3")
    EW = la.getLastStepHaltingNumber("0")
    EW += la.getLastStepHaltingNumber("1")
    N = la.getLastStepHaltingNumber("2")
    N += la.getLastStepHaltingNumber("3")

    phase = tl.getPhase("0") % 2 # only in this case, for four phases
    db('State:', EW, N)
    sm = stateMap(EW, N, phase)
    visits.append((EW, N))
    return sm

def getDTSE():
    # Get all vehicles in the lanearea
    # Given a list of relevant vehicles, convert it t
    # Array of intersection state maps

    DTSE_pos = np.zeros((NUM_LANES*4, num_cells)) 
    DTSE_spd = np.zeros((NUM_LANES*4, num_cells)) 
    for l, det_id in enumerate(la.getIDList()):
        det_len = la.getLength(det_id)
        lane_len = lane.getLength(la.getLaneID(det_id))

        assert det_len == LANEAREA_LENGTH
        for vid in la.getLastStepVehicleIDs(det_id):
            # What is the correct encoding of pos to cell? Should low indices specify cars close or far away for traffic light?
            #print('lid',veh.getLanePosition(vid))
            pos = LANEAREA_LENGTH - lane_len + veh.getLanePosition(vid)
            if pos < 0:
                continue
            cell = int(min(pos // c, num_cells)) # in case pos ends up being exactly the length of the detector
            
            speed = veh.getSpeed(vid)
            #print("Veh {}: {}".format(vid, pos))
            #print('cell',cell)
 
            # Record binary position and speed arrays
            DTSE_pos[l][cell] = 1
            DTSE_spd[l][cell] = speed

    phase = int(tl.getPhase("0")) % 2
    signal_state = torch.zeros(1, A).float()
    signal_state[0][phase] = 1
    
    # Add batch dimension and input filter dimensions to state vectors
    DTSE_pos = torch.from_numpy(DTSE_pos).unsqueeze(0).unsqueeze(0).float()
    DTSE_spd = torch.from_numpy(DTSE_spd).unsqueeze(0).unsqueeze(0).float()
    return DTSE_pos, DTSE_spd, signal_state
 
def numVehicles():
    vehicles = 0
    for l, det_id in enumerate(la.getIDList()):
        vehicles += len(la.getLastStepVehicleIDs(det_id))
    return vehicles

def getReward():
    #return -getTimeInQueue()
    return -getCumulativeDelay()
    # Try normalizing the reward and using queue length again
    #return -cumHaltingNumber()

def stateMap(EW, NS, phase):
    # What if we decrease the state space size:
    # Consider only every four cars or so
    #return  NS + EW // 4
    return 28 * 2 * NS + EW# + phase

# Use epsilon-greedy action selection
def select_action(state_idx, epsilon):
    seed = np.random.random()
    if seed < epsilon: # select random actions
        return np.random.choice(A, 1)[0]
    else: # select greedy action
        return np.argmax(Q[state_idx])

def step():
    ''' Wrapper for traci simulation step'''
    traci.simulationStep()

    # Assume we have a list of vehicles in queue at last time step: last
    # We now have list of vehicles in queue at current time step: cur
    # For each vehicle in cur but not last, update time in queue
    # For each vehicle in last but not cur, don't do anything...
    # Update list of vehicles in queue
    #t = sim.getCurrentTime() // 1000 # Current time in seconds
    #for det_id in la.getIDList():
    #    for vid in la.getLastStepVehicleIDs(det_id):
    #        start = vehicles_in_queue.setdefault(vid, t)
    #        vehicles_in_queue[vid] = t - start
 

# NOT USING RN
def inductionloop():
    """execute the TraCI control loop"""
        # we start with phase 2 where EW has green
    tl.setPhase("0", 2)
    i = 2
    while sim.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print("E:", la.getLastStepHaltingNumber("0"))
        print("N:", la.getLastStepHaltingNumber("1"))
        # Get occupancy of East lane
        #print(lane.getLastStepOccupancy("2i_0"))
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

def fixed():
    tl.setPhase("0", 2)
    getState()
    while sim.getMinExpectedNumber() > 0:
        traci.simulationStep()
        getState()
        r = getReward()
        rewards_t.append(r)

def myfixed(greentime, yellowtime):
    tl.setPhase("0", 2)
    getState()
    while sim.getMinExpectedNumber() > 0:
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

def qlearn(model, learn=True):
    print("File beginning", flush=True)
    print(model.Q)    
    
    action_time = 12
    step()#traci.simulationStep()
    tl.setPhase("0", 2)
    # Find current state
    s = getState()
    
    i = 0
    while sim.getMinExpectedNumber() > 0:
        # Take action 
        tl.setPhaseDuration("0", 10000)
        a = model(s)

        if a*2 != tl.getPhase("0"): # Change action
            db('Change to other green light')
            tl.setPhase("0", (tl.getPhase("0")+1) % 4)
            #print("phase:", tl.getPhase("0"))
            yellow_time = (tl.getNextSwitch("0") - sim.getCurrentTime()) // 1000
        else:
            yellow_time = 0

        #if i % 100 == 0:
        #    print('State idx:', s)
        #    print("t:", sim.getCurrentTime() / 1000)
        #    print("epsilon:", model.epsilon)

        db('action:', a)
        db("-" * 10)

        # Move simulation forward one step and obtain reward
        #db(action_time + yellow_time)
        for j in range(yellow_time + action_time):
            step()#traci.simulationStep()
            # Get some kind of reward signal from the E2 loops
            # Let's say its negative of the number of cars on the loop
            
            if sim.getMinExpectedNumber() <= 0:
                return
        r = getReward()#-cumHaltingNumber()
        rewards_t.append(r)
 
        db("reward:", r)
        s_ = getState()

        if learn:
            db('Before:', s, model.Q[s])
            model.update(s, a, r, s_, alpha)
            db('After:', s, model.Q[s])
            i += 1
        s = s_
        
def save_output(fname, episode=None):
    if episode is not None:
        np.savetxt('{}_episode{}_rewards'.format(args.saverewards, episode), rewards_t)
        np.savetxt('{}_episode{}_visits'.format(args.saverewards, episode), visits)
    else:
        np.savetxt('{}_rewards'.format(args.saverewards), rewards_t)
        np.savetxt('{}_visits'.format(args.saverewards), visits)

# Need to write a better framework for putting in models
def deepqlearn(model, target, beta, writer=None, learn=True):
    # Same as q learning
    # Get traffic state (from DTSE this time)
    # Perform action, get reward
    # Store transition in replay memory
    # Get next state
    # Now perform the update loop
    print("File beginning", flush=True)

    # Initialize deep traffic controller
    # Memory capacity is num timesteps for 1.5 hours over 200 episodes = 1.5 * 3600 * 200 = 108000
    # For now anyway
    action_time = 12
    step()
    tl.setPhase("0", 2)
    # Find current state
    s = getDTSE()
    db('state:', s)
    n_action_cycle = 0
    while sim.getMinExpectedNumber() > 0:
        # Take action 
        tl.setPhaseDuration("0", 10000)
        a = model.select_action(s)

        if a[0, 0]*2 != tl.getPhase("0"): # Change action
            db('Change to other green light')
            tl.setPhase("0", (tl.getPhase("0")+1) % 4)
            #print("phase:", tl.getPhase("0"))
            yellow_time = (tl.getNextSwitch("0") - sim.getCurrentTime()) // 1000
        else:
            yellow_time = 0

        #print('t:', sim.getCurrentTime() / 1000)

        if n_action_cycle % 100 == 0:
            print("t:", sim.getCurrentTime() / 1000)
            print('epsilon:', model.epsilon)
        db('action:', a)
        r = 0

        # Move simulation forward one step and obtain reward
        db(action_time + yellow_time)
        db("-" * 10)
        for j in range(yellow_time + action_time):
            step()
            rewards_t.append(r)
            if sim.getMinExpectedNumber() <= 0:
                return
        r = getReward()
        if writer:
            writer.add_scalar("data/reward", r, n_action_cycle)
        db("reward:", r)

        s_ = getDTSE()
        db("next state:", s_)
        if learn:
            # Add transition to replay memory
            # TODO: Make sure s, a, r, and s_ are Tensors this time
            model.memory.push(s, a, torch.Tensor([r]), s_)
            optimize_model(model, target, gamma, beta, writer=writer)
            n_action_cycle += 1
        s = s_
        
def get_arguments():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    argParser.add_argument("--saverewards")
    argParser.add_argument("--loadfromfile")
    argParser.add_argument("--savetofile")
    argParser.add_argument("--method")
    argParser.add_argument("--epsilon", type=float, default=0.0)
    argParser.add_argument("--episodes", type=int)

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
        traci.start([sumoBinary, "-c", "data/single/cross.sumocfg",
                                  "--tripinfo-output", "{}_tripinfo.xml".format(args.method),
                                  "--error-log", "errors"])
        # Initialize qlearn class
        print('loadfromfile in main:',args.loadfromfile)
        model = QLC(S, A, epsilon=args.epsilon, gamma=gamma, anneal=False, anneal_steps=N//36, loadQ=args.loadfromfile)

        qlearn(model) # don't load from file, do learn

        if args.savetofile:
            model.save_Q_values(args.savetofile)
        if args.saverewards:
            save_output(args.saverewards)

    elif args.method == 'qle':
        print('Using epsilon-greedy Qlearning controller')
        episodes = args.episodes

        model = QLC(S, A, epsilon=args.epsilon, gamma=gamma, anneal=True, anneal_steps=N//36, loadQ=args.loadfromfile)
        for i in range(episodes):
            visits = []
            rewards_t = []
            # this is the normal way of using traci. sumo is started as a
            # subprocess and then the python script connects and runs
            traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                                      "--tripinfo-output", "{}_episode{}_tripinfo.xml".format(args.method, str(i)),
                                      "--error-log", "errors"])

            print("Starting episode", i)
            if args.loadfromfile:
                if i > 0:
                    args.loadfromfile += "_epsiode" + str(i-1)
                else:
                    args.loadfromfile = None
            
            qlearn(model) # don't load from file, do learn

            if args.savetofile:
                model.save_Q_values('{}_episode{}'.format(args.savetofile, str(i)))
            if args.saverewards:
                save_output(args.saverewards, i)
    elif args.method == 'f':
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        #min_green_time = 10
        #max_green_time = 60
        min_green_time = 40 
        max_green_time = 40
        inc = 5
        for gt in range(min_green_time, max_green_time+1, inc):
            _gt = gt
            traci.start([sumoBinary, "-c", "data/single/cross.sumocfg",
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

        model = QLC(S, A, epsilon=args.epsilon, gamma=gamma, anneal_steps=N//18, loadQ=args.loadfromfile)
        qlearn(model, learn=False)

        if args.saverewards:
            save_output(args.saverewards)
    elif args.method == 'dql':
        print('Using Deep q-learning traffic controller')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        model = DTC(c * num_cells * NUM_LANES * 4, A, epsilon=args.epsilon, capacity=100000, batch_size=32)
        target = DTC(c * num_cells * NUM_LANES * 4, A)

        if args.loadfromfile is not None:
            state_dict = torch.load(args.loadfromfile)
            model.load_state_dict(state_dict["model"])
            target.load_state_dict(state_dict["target"])
            model.optimizer.load_state_dict(state_dict["optim"])

        # Init tensorboard summary writer
        writer = SummaryWriter()
        episodes = args.episodes
        
        for i in range(episodes):
            traci.start([sumoBinary, "-c", "data/single/cross.sumocfg",
                                  "--tripinfo-output", "{}_episode{}_tripinfo.xml".format(args.method, i),
                                  "--error-log", "errors"])

            deepqlearn(model, target, beta, writer=writer, learn=True)
            if args.savetofile:
                state = {
                    'model' : model.state_dict(),
                    'optim' : model.optimizer.state_dict(),
                    'target': target.state_dict()
                }
                torch.save(state, "{}_episode{}.pt".format(args.savetofile, i))
                # TODO: SAVE net parameters TO FILE
            if args.saverewards:
                save_output(args.saverewards)
    else:
        print('Invalid method selected')
            
    exit(0)
