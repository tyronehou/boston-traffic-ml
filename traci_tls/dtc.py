import gym
import math
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
        ('state', 'action', 'next_state', 'reward'))

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

class DTC(nn.Module):
    ''' Deep traffic controller '''
    def __init__(self, A):
        # Define conv and fully connected layers
        super(DTC, self).__init__()
        self.conv_pos1 = nn.Conv2d(1, 16, kernel_size=(4,4), stride=2)
        #self.bn1 = nn.BatchNorm2d(16)
        self.conv_pos2 = nn.Conv2d(16, 32, kernel_size=(2,2), stride=2)
        #self.bn2 = nn.BatchNorm2d(32)
        self.conv_speed1 = nn.Conv2d(1, 32, kernel_size=(4,4), stride=2)
        #self.bn3 = nn.BatchNorm2d(32)
        self.conv_speed2 = nn.Conv2d(32, 32, kernel_size=(2,2), stride=2) 
        self.head1 = nn.Linear(128, 64)
        self.head2 = nn.Linear(64, A) # |A|, size of action space

    def forward(self, pos, speed, P):
        ''' P -- current traffic phase vector '''
        pos = F.relu(self.conv_pos1(pos))
        pos = F.relu(self.conv_pos2(pos))
        speed = F.relu(self.conv_speed1(speed))
        speed = F.relu(self.conv_speed2(speed))
        
        # combine pos speed and P vector into one long vector
        X
        X1 = self.head1(X.view(x.size(0), -1))
        return self.head2(X1)
def optimize_model(model):

