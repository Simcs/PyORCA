import numpy as np

from utility import *
from simulator import RVOSimulator

class BaseEnv(object):

    def __init__(self, simulator):
        self.simulator = simulator
        self.goals = []
        self.colors = []

    def setUpScenario(self):
        pass
    
    def setPreferredVelocities(self):
        for i in range(self.num_agents):
            goal_vector = self.goals[i] - self.simulator.agents[i].position
            goal_vector = normalize(goal_vector)
            self.simulator.agents[i].pref_velocity = goal_vector

    def reachedGoal(self):
        for i in range(self.num_agents):
            dist = absSq(self.simulator.agents[i].position - self.goals[i])
            if dist > self.simulator.agents[i].radius ** 2:
                return False
        return True

    def printAgentsInfo(self):
        print('global time:', self.simulator.global_time)
        for agent in self.simulator.agents:
            print(f'Agent ID: {agent.id}, position: {agent.position}, velocity: {agent.velocity}')
