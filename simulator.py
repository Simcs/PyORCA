import numpy as np
from agent import Agent
from kd_tree import KDTree

class RVOSimulator(object):

    def __init__(self):
        self.global_time = 0.0
        self.global_steps = 0
        self.time_step = 0.0
        self.default_agent = None
        self.agents = []
        self.kd_tree = KDTree(self)

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
        agent.max_neighbors = self.default_agent.max_neighbors
        agent.neighbor_dist = self.default_agent.neighbor_dist
        agent.max_speed = self.default_agent.max_speed
        agent.radius = self.default_agent.radius
        agent.time_horizon = self.default_agent.time_horizon
        agent.velocity = self.default_agent.velocity
        agent.id = len(self.agents)
        agent.is_obstacle = False

        self.agents.append(agent)
        return len(self.agents) - 1
    
    def addObstacle(self, position, radius):
        agent = Agent(self)
        agent.position = position
        agent.radius = radius
        agent.max_speed = 0.0
        agent.velocity = np.array([0., 0.,])
        agent.time_horizon = self.default_agent.time_horizon
        agent.id = len(self.agents)
        agent.is_obstacle = True

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

    def computeAgentNeighbors(self, agent):
        for a in self.agents:
            if a == agent:
                continue
            agent.agent_neighbors.append(a)
    
    def doStep(self):

        # self.kd_tree.buildAgentTree()

        for agent in self.agents:
            if agent.is_obstacle:
                continue
            agent.computeNeighbors()
            agent.computeNewVelocity()

        for agent in self.agents:
            if agent.is_obstacle:
                continue
            agent.update()

        self.global_time += self.time_step
        self.global_steps += 1

    def setAgentDefaults(self, time_horizon, radius, max_speed, velocity):
        if self.default_agent is None:
            self.default_agent = Agent(self)
        
        self.default_agent.time_horizon = time_horizon
        self.default_agent.radius = radius
        self.default_agent.max_speed = max_speed
        self.default_agent.velocity = velocity

    # def setAgentDefaults(self, neighbor_dist, max_neighbors, time_horizon, radius, max_speed, velocity):
    #     if self.default_agent is None:
    #         self.default_agent = Agent(self)
        
    #     self.default_agent.neighbor_dist = neighbor_dist
    #     self.default_agent.max_neighbors = max_neighbors
    #     self.default_agent.time_horizon = time_horizon
    #     self.default_agent.radius = radius
    #     self.default_agent.max_speed = max_speed
    #     self.default_agent.velocity = velocity
