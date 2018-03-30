import numpy as np
# Tabular Q-learning traffic controller
class QLC:
    def __init__(self, S, A, epsilon=0.1, gamma=0.9, anneal=False, anneal_steps=0, loadQ=None):
        ''' S -- size of state space
            A -- size of action space
            epsilon -- chance of taking random action
            anneal-- if anneal
        '''
        self.S = S
        self.A = A

        self.anneal = anneal
        self.steps = anneal_steps
        if anneal:
            self.epsilon = 0.5
            self.epsilon_goal = epsilon
            self.increment = (self.epsilon - self.epsilon_goal) / self.steps
        else:
            self.epsilon = epsilon
        self.i = 0
 
        self.gamma = gamma
        print('model loadQ:', loadQ)
        if loadQ != None:
            self.Q = self.load_params(loadQ)
        else:
            self.Q = np.zeros((S, A))

    # select action
    def __call__(self, state_idx):
        seed = np.random.random()
        if seed < self.epsilon: # select random actions
            print('RANDOM ACTION:', seed)
            return np.random.choice(self.A, 1)[0]
        else: # select greedy action
            return np.argmax(self.Q[state_idx])

    def select_action(self, state_idx):
        return self.__call__(state_idx)

    def update(self, s, a, r, s_, *args, alpha=0.1, **kwargs):
        ''' alpha -- step size '''
        self.Q[s][a] += alpha * (r + self.gamma * np.max(self.Q[s_][a]) - self.Q[s][a])
        if self.anneal and self.i < self.steps:
            self.epsilon -= self.increment
            self.i += 1 

    def load_params(self, fname):
        return np.loadtxt(fname)

    def save_params(self, fname):
        np.savetxt(fname, self.Q) 
