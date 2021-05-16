import numpy as np

from utility import *
from simulator import RVOSimulator

class Circle(object):

    def __init__(self, simulator):
        self.simulator = simulator
        self.num_agents = 8
        self.radius = 20
        self.goals = []

    def setUpScenario(self):
        self.simulator.time_step = 0.25
        self.simulator.setAgentDefaults(
            time_horizon=10.0,
            radius=1.5,
            max_speed=2.0,
            velocity=np.array([0., 0.]),
        )
        for i in range(self.num_agents):
            pos = self.radius * np.array([
                np.cos(2.0 * np.pi * i / self.num_agents),
                np.sin(2.0 * np.pi * i / self.num_agents)
            ])
            self.simulator.addAgent(pos)
            self.goals.append(-self.simulator.agents[i].position)
    
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


if __name__ == "__main__":
    simulator = RVOSimulator()
    scenario = Circle(simulator)
    scenario.setUpScenario()

    while not scenario.reachedGoal():
        scenario.printAgentsInfo()
        scenario.setPreferredVelocities()
        simulator.doStep()
    print('All agents reached goal!')
