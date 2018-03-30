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

from collections import OrderedDict
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

# If GPU usage
use_cuda = torch.cuda.is_available()
FloatTensor = torch.cuda.FloatTensor if use_cuda else torch.FloatTensor
LongTensor = torch.cuda.LongTensor if use_cuda else torch.LongTensor
ByteTensor = torch.cuda.ByteTensor if use_cuda else torch.ByteTensor
Tensor = FloatTensor

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
la_mapping = {} # mapping of lane ids to lanearea ids

debug = False

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME")))
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from traci import lanearea as la
from traci import lane
from traci import vehicle as veh
from traci import trafficlights as tl
from traci import simulation as sim

def db(*args, **kwargs):
    if debug:
        print(*args, **kwargs)

single_veh = """
    <vType id="0" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
    <vType id="1" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>
"""

single_routes = """
    <route id="WE" edges="51o 1i 2o 52i" />
    <route id="EW" edges="52o 2i 1o 51i" />
    <route id="NS" edges="54o 4i 3o 53i" />
"""

multi_routes = """
    <route id="00" edges="00_0i 0_1 1_11o" />
    <route id="01" edges="01_0i 0_3 3_30o" />
    <route id="10" edges="10_1i 1_2 2_21o" />
    <route id="11" edges="11_1i 1_0 0_00o" />
    <route id="20" edges="20_2i 2_3 3_31o" />
    <route id="21" edges="21_2i 2_1 1_10o" />
    <route id="30" edges="30_3i 3_0 0_01o" />
    <route id="31" edges="31_3i 3_2 2_20o" />
"""

def generate_lanearea_mapping():
    if la_mapping != {}:
        return

    for det_id in la.getIDList():
        l = la.getLaneID(det_id)
        print('lane:', l)
        la_mapping[l] = det_id

def generate_routefile(N, intersection):
    #random.seed(42)  # make tests reproducible
    # demand per second from different directions
    if intersection == 's':
        route_file = "data/single/cross.rou.xml" 
        queue_params = [OrderedDict(
            (
                ("WE", 2. / 5),
                ("EW", 2. / 5),
                ("NS", 2. / 20),
                ("length", N // 2)
            )),
            OrderedDict((
                ("WE", 3. / 10),
                ("EW", 3. / 10),
                ("NS", 3. / 10),
                ("length", N // 2)
            ))
        ]
        vehicles = single_veh
        routes = single_routes
    else:
        route_file = "data/multi/cross.rou.xml"
        queue_params = [OrderedDict(
            (
                ("00", 1. / 5),
                ("11", 1. / 5),
                ("20", 1. / 5),
                ("31", 1. / 5),
                ("01", 1. / 10),
                ("10", 1. / 10),
                ("21", 1. / 10),
                ("30", 1. / 10),
                ("length", N)
            ))
        ]
        vehicles = single_veh
        routes = multi_routes
        
    with open(route_file, "w") as f:
        print(queue_params)
        print("<routes>{}{}".format(vehicles, routes), file=f)
        vehNr = 0
        x = 0
        for queue_segment in queue_params:
            for i in range(x, x+queue_segment["length"]):
                for route_id, p in queue_segment.items():
                    if route_id != "length" and random.uniform(0, 1) < p:
                        print('<vehicle id="{route_id}_{vehicle_num}" type="0" route="{route_id}" depart="{time}" />'.format(
                            route_id=route_id, vehicle_num=vehNr, time=i), file=f)
                        vehNr += 1
            x += queue_segment["length"]
        print("</routes>", file=f)

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

def getCumulativeDelay(tls_id):
    # Get each vehicle in lanearea
    delay = 0
    for lane_id in tl.getControlledLanes(tls_id):
        det_id = la_mapping[lane_id]
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

def getState(tls_id):
    EW = 0
    N = 0

    for lane_id in tl.getControlledLanes(tls_id):
        det_id = la_mapping[lane_id]
        
        if det_id[-1] in ["E", "W"]:
            EW += la.getLastStepHaltingNumber(det_id)
        else: # in ["N", "S"]
            N += la.getLastStepHaltingNumber(det_id)

    phase = tl.getPhase(tls_id) % 2 # only in this case, for four phases
    db('State:', EW, N)
    sm = stateMap(EW, N, phase)
    visits.append((EW, N))
    return sm

def getDTSE(tls_id):
    # Get all vehicles in the lanearea
    # Given a list of relevant vehicles, convert it t
    # Array of intersection state maps

    DTSE_pos = np.zeros((NUM_LANES*4, num_cells)) 
    DTSE_spd = np.zeros((NUM_LANES*4, num_cells)) 
    for l, lane_id in enumerate(tl.getControlledLanes(tls_id)):
        det_id = la_mapping[lane_id]
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

    phase = int(tl.getPhase(tls_id)) % 2
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

def getReward(tls_id):
    #return -getTimeInQueue()
    return -getCumulativeDelay(tls_id)
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
    tls_ids = tl.getIDList()
    for tls_id in tls_ids:
        tl.setPhase(tls_id, 2)
        getState(tls_id)
    while sim.getMinExpectedNumber() > 0:
        for i in range(greentime):
            for tls_id in tls_ids:
                step()
                tl.setPhase(tls_id, 2)
                getState(tls_id)
                r = getReward(tls_id)
                rewards_t.append(r)

        for i in range(yellowtime):
            for tls_id in tls_ids:
                step()
                tl.setPhase(tls_id, 3)
                getState(tls_id)
                r = getReward(tls_id)
                rewards_t.append(r)
 
        for i in range(greentime):
            for tls_id in tls_ids:
                step()
                tl.setPhase(tls_id, 0)
                getState(tls_id)
                r = getReward(tls_id)
                rewards_t.append(r)
 
        for i in range(yellowtime):
            for tls_id in tls_ids:
                step()
                tl.setPhase(tls_id, 1)
                getState(tls_id)
                r = getReward(tls_id)
                rewards_t.append(r)

def run(models, state_fn, action_time = 12, learn=True):
    print("File beginning", flush=True)
    
    tls_ids = tl.getIDList()
    step()#traci.simulationStep()
    s = [None for i in range(len(tls_ids))]
    a = [None for i in range(len(tls_ids))]
    r = [None for i in range(len(tls_ids))]
    s_ = [None for i in range(len(tls_ids))]

    next_action_times = np.zeros(len(tls_ids), dtype=int)

    for t, tls_id in enumerate(tls_ids):
        tl.setPhase(tls_id, 2)
        # Find current states
        s[t] = state_fn(tls_id)
    
    while sim.getMinExpectedNumber() > 0:
        for t, tls_id in enumerate(tls_ids):
            db("Phase duration for " + tls_id, tl.getPhaseDuration(tls_id))
            if next_action_times[t] > 0:
                continue

            tl.setPhaseDuration(tls_id, 10000)
            # Take actions
            a[t] = models[t].select_action(s[t])

            current_phase = tl.getPhase(tls_id)
            if a[t]*2 != current_phase: # Change action
                db('Change to other green light')
                tl.setPhase(tls_id, (current_phase+1) % 4)
                #print("phase:", tl.getPhase("0"))
                yellow_time = (tl.getNextSwitch(tls_id) - sim.getCurrentTime()) // 1000
            else:
                yellow_time = 0

            next_action_times[t] = action_time + yellow_time

        db('actions:', a)
        db("-" * 10)

        # Move simulation forward one step and obtain reward
        #db(action_time + yellow_time)
        step_size = min(next_action_times)
        for j in range(step_size):
            step()#traci.simulationStep() 
            if sim.getMinExpectedNumber() <= 0:
                return
        next_action_times -= step_size
        
        # Get some kind of reward signal from the E2 loops
        # Let's say its negative of the number of cars on the loop

        for t, tls_id in enumerate(tls_ids):
            if next_action_times[t] > 0:
                continue

            r[t] = getReward(tls_id)
            rewards_t.append(r)
 
            db("reward:", r)
            s_[t] = state_fn(tls_id)

            if learn:
                models[t].update(s[t], a[t], r[t], s_[t], alpha)
            s[t] = s_[t]

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
    argParser.add_argument("--method", choices=["f", "ql", "dql"],
                        help="Choose type of traffic controller. f is fixed, ql is tabular ql, dql is deep q learning")
    argParser.add_argument("--intersection", choices=["s", "m"],
                        default="s", help="Choose type of intersection to simulate. s is single m is multiple")
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

    print("Running method {} on intersection {}".format(args.method, "single" if args.intersection == "s" else "multi"))
    # first, generate the route file for this simulation

    if args.intersection == 's': # single intersection
        sumo_cfgfile = "data/single/cross.sumocfg"
        num_models = 1
    else: # multiple intersection
        sumo_cfgfile = "data/multi/cross.sumocfg"
        num_models = 4

    if args.method == 'f':
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        min_green_time = 10
        max_green_time = 35
        #min_green_time = 40
        #max_green_time = 40
        inc = 5
        for gt in range(min_green_time, max_green_time+1, inc):
            _gt = gt
            generate_routefile(N, intersection=args.intersection)

            traci.start([sumoBinary, "-c", sumo_cfgfile,
                                     "--tripinfo-output", "{}green={}_tripinfo.xml".format(args.method, _gt),
                                     "--error-log", "errors"])
            generate_lanearea_mapping() 

            print('Used fixed controller with cycle ({0},{1},{0},{1})'.format(_gt, 6))
            myfixed(_gt, 6)
            if args.saverewards:
                save_output(args.saverewards, _gt)
    else:    

        if args.method == 'ql':
            print('Using epsilon-greedy Qlearning controller')
            models = []
            for i in range(num_models):
                models.append(QLC(S, A, epsilon=args.epsilon, gamma=gamma, anneal=True, anneal_steps=N//36, loadQ=args.loadfromfile))
            state_fn = getState

        elif args.method == 'dql':
            print('Using Deep q-learning traffic controller')
            # this is the normal way of using traci. sumo is started as a
            # subprocess and then the python script connects and runs
            models = []
            for i in range(num_models):
                models.append(DTC(c * num_cells * NUM_LANES * 4, A, epsilon=args.epsilon, capacity=100000, batch_size=32, gamma=0.9, beta=0.1))

                if args.loadfromfile is not None:
                    models[i].load_params(args.loadfromfile + "m" + i)

            state_fn = getDTSE
            
            # Init tensorboard summary writer
            writer = SummaryWriter()

        for i in range(args.episodes):
            # Generate a different route file every iteration
            generate_routefile(N, intersection=args.intersection)
            visits = []
            rewards_t = []

            tripinfo_xml = "{}_episode{}_tripinfo.xml".format(args.method, str(i))
            if args.savetofile:
                save_fname = "{}_episode{}".format(args.savetofile, i)

            traci.start([sumoBinary, "-c", sumo_cfgfile,
                                  "--tripinfo-output", tripinfo_xml,
                                  "--error-log", "errors"])

            generate_lanearea_mapping() 
            run(models, state_fn, learn=True)
            if args.savetofile:
                for model in models:
                    model.save_params(save_fname)
            if args.saverewards:
                save_output(args.saverewards, i)

    exit(0)
