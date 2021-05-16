import numpy as np
from agent import Agent

class RVOSimulator(object):

    def __init__(self):
        self.global_time = 0.0
        self.time_step = 0.0
        self.default_agent = None
        self.agents = []

    # def __init__(self, time_step, time_horizon, radius, max_speed, velocity):
        
    #     self.global_time = 0.0
    #     self.time_step = time_step
    #     self.agents = []

    #     self.default_agent = Agent(self)
    #     self.default_agent.time_horizon = time_horizon
    #     self.default_agent.radius = radius
    #     self.default_agent.max_speed = max_speed
    #     self.default_agent.velocity = velocity
    
    def addAgent(self, position):
        agent = Agent(self)
        agent.position = position
        agent.max_speed = self.default_agent.max_speed
        agent.radius = self.default_agent.radius
        agent.time_horizon = self.default_agent.time_horizon
        agent.velocity = self.default_agent.velocity
        agent.id = len(self.agents)

        self.agents.append(agent)
        return len(self.agents) - 1
    
    # def addAgent(sefl, position, time_horizon, radius, max_speed, velocity):
    #     agent = Agent(self)
    #     agent.position = position
    #     agnet.max_speed = max_speed
    #     agent.radius = radius
    #     agent.time_horizon = time_horizon
    #     agent.velocity = velocity
    #     agent.id = len(self.agents)

    #     self.agents.append(agent)
    #     return len(self.agents) - 1
    
    def doStep(self):
        for agent in self.agents:
            agent.computeNewVelocity()

        for agent in self.agents:
            agent.update()

        self.global_time += self.time_step

    def setAgentDefaults(self, time_horizon, radius, max_speed, velocity):
        if self.default_agent is None:
            self.default_agent = Agent(self)
        
        self.default_agent.time_horizon = time_horizon
        self.default_agent.radius = radius
        self.default_agent.max_speed = max_speed
        self.default_agent.velocity = velocity
