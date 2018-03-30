# Architecture from https://arxiv.org/abs/1705.02755
# Adaptive Traffic Signal Control: Deep Reinforcement Learning Algorithm with Experience Replay and Target Network 
# Juntao et. al

import random
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
from itertools import count
from copy import deepcopy
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision.transforms as T

# If GPU usage
use_cuda = torch.cuda.is_available()
FloatTensor = torch.cuda.FloatTensor if use_cuda else torch.FloatTensor
LongTensor = torch.cuda.LongTensor if use_cuda else torch.LongTensor
ByteTensor = torch.cuda.ByteTensor if use_cuda else torch.ByteTensor
Tensor = FloatTensor

Transition = namedtuple('Transition',
        ('state', 'action', 'reward', 'next_state'))

class ReplayMemory():
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0
    def push(self, *args):
        """saves a transition"""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

# Deep traffic controller
class DNN(nn.Module):
    ''' Deep traffic controller '''
    def __init__(self, input_size, A, epsilon=0.1, batch_size=32):
        # Define conv and fully connected layers
        super(DNN, self).__init__()
        #TODO: try diff kernel sizes and strides
        # Layer 1 position conv - 16 filters w/ 4x4 kernel
        self.conv_pos1 = nn.Conv2d(1, 16, kernel_size=(2,2), stride=2)
        self.bn_pos1 = nn.BatchNorm2d(16)
        
        # Layer 2 position conv - 32 filters w/ 2x2 kernel
        self.conv_pos2 = nn.Conv2d(16, 32, kernel_size=(2,2), stride=2)
        self.bn_pos2 = nn.BatchNorm2d(32)
        
        # Layer 1 speed conv - 16 filters w/ 4x4 kernel
        self.conv_spd1 = nn.Conv2d(1, 32, kernel_size=(2,2), stride=2)
        self.bn_spd1 = nn.BatchNorm2d(32)

        # Layer 2 speed conv - 32 filters w/ 2x2 kernel
        self.conv_spd2 = nn.Conv2d(32, 32, kernel_size=(2,2), stride=2) 
        self.bn_spd2 = nn.BatchNorm2d(32)
        
        # Fully connected layers
        self.fc3 = nn.Linear(2*32*1*6+A, 128) # Input size depends on size of the input to the first layer
        self.fc4 = nn.Linear(128, 64) 
        self.out = nn.Linear(64, A) # |A|, size of action space

        # Things related to neural net training that are not layers
        self.epsilon = epsilon
        self.batch_size = 32
        self.optimizer = optim.RMSprop(self.parameters())

    def forward(self, traffic_state):
        ''' P -- Position array
            V -- Speed array
            L -- traffic signal state
        '''
        P, V, L = traffic_state

        #print('P:', P.size(), 'V:', V.size(), 'L:', L.size())
        # Subnet pos
        pos = F.relu(self.conv_pos1(P))
        pos = F.relu(self.conv_pos2(pos))
        # Subnet spd
        spd = F.relu(self.conv_spd1(V))
        spd = F.relu(self.conv_spd2(spd))
        
        p = pos.view(pos.size(0), -1) # 0th is batch dimension
        v = spd.view(spd.size(0), -1)
        l = L.view(L.size(0), -1)

        X = torch.cat((p, v, l), 1)
        # combine pos, speed and traffic signal state vector into one long vector
        X = self.fc3(X)
        X = self.fc4(X)
        return self.out(X)

    def select_action(self, s):
        ''' If greedy is True, the action selected ignores the epsilon value '''
        P, V, L = s
        seed = np.random.random()
        if seed < self.epsilon: # select random actions
            print('RANDOM ACTION:', seed)
            return np.random.randint(2) #np.random.choice(self.A, 1)[0]
        else: # select greedy action
            return self.__call__((Variable(P), Variable(V), Variable(L))).data.max(1)[1][0] # torch.max serves as both max and argmax
            
class DTC:
    def __init__(self, *args, capacity=10000, gamma=0.9, beta=0.1, **kwargs):
        self.model = DNN(*args, **kwargs)
        self.target = DNN(*args, **kwargs)
        self.memory = ReplayMemory(capacity)
        self.gamma = gamma
        self.beta = beta

    def select_action(self, s):
        return self.model.select_action(s)
    
    def update(self, s, a, r, s_, *args, **kwargs):
        # push memory
        a = torch.LongTensor([[a]])
        r = torch.Tensor([r])
        self.memory.push(s, a, r, s_)
        optimize_model(self.model, self.target, self.memory, self.gamma, self.beta)

    def save_params(self, fname):
        state = {
            'model' : self.model.state_dict(),
            'optim' : self.model.optimizer.state_dict(),
            'target': self.target.state_dict(),
            'memory': self.memory
        }
        torch.save(state, fname)

    def load_params(self, fname):
        state_dict = torch.load(fname)
        self.model.load_state_dict(state_dict["model"])
        self.model.optimizer.load_state_dict(state_dict["optim"])
        self.target.load_state_dict(state_dict["target"])
        self.memory = state_dict["memory"]

def optimize_model(model, target, memory, gamma, beta, writer=None):
    if len(memory) < model.batch_size:
        return

    # Get minibatch of sample transitions; paper uses 32
    transitions = memory.sample(model.batch_size)
    batch = Transition(*zip(*transitions)) # Create transition batch

    state_batch = [Variable(torch.cat(x)) for x in zip(*batch.state)]
    action_batch = Variable(torch.cat(batch.action))
    reward_batch = Variable(torch.cat(batch.reward))

    # Use DNN to compute predicted action
    state_action_values = model(state_batch).gather(1, action_batch)
    # Use Target network to compute target action
    target_action_values = reward_batch + gamma * target(state_batch).max(1)[1].float()

    # Compute MSE loss
    loss = nn.MSELoss()
    output = loss(state_action_values, target_action_values)
 
    model.optimizer.zero_grad()
    output.backward()

    # Update theta of DNN with optimizer
    for param in model.parameters():
        if param.grad is not None:
            param.grad.data.clamp_(-1, 1)
    model.optimizer.step()

    # Update theta_ of target network using theta soft update
    model_param_gen = model.parameters()
    for param in target.parameters():
        mp = next(model_param_gen)
        param.data = beta * mp.data + (1-beta) * param.data
