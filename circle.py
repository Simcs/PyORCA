import numpy as np

from utility import *
from simulator import RVOSimulator

goals = []

def setUpScenario(simulator):
    simulator.time_step = 0.25
    simulator.setAgentDefaults(
        time_horizon=10.0,
        radius=1.5,
        max_speed=2.0,
        velocity=np.array([0., 0.]),
    )
    num_agents = 8
    r = 20
    for i in range(num_agents):
        pos = r * np.array([
            np.cos(2.0 * np.pi * i / num_agents),
            np.sin(2.0 * np.pi * i / num_agents)
        ])
        simulator.addAgent(pos)
        goals.append(-simulator.agents[i].position)

def setPreferredVelocities(simulator):
    for i in range(len(simulator.agents)):
        goal_vector = goals[i] - simulator.agents[i].position
        goal_vector = normalize(goal_vector)
        simulator.agents[i].pref_velocity = goal_vector

def reachedGoal(simulator):
    for i in range(len(simulator.agents)):
        dist = absSq(simulator.agents[i].position - goals[i])
        if dist > simulator.agents[i].radius ** 2:
            return False
    return True

def printAgentsInfo(simulator):
    print('global time:', simulator.global_time)
    for agent in simulator.agents:
        print(f'Agent ID: {agent.id}, position: {agent.position}, velocity: {agent.velocity}')


if __name__ == "__main__":
    simulator = RVOSimulator()

    setUpScenario(simulator)

    while not reachedGoal(simulator):
        printAgentsInfo(simulator)
        setPreferredVelocities(simulator)
        simulator.doStep()
    print('All agents reached goal!')
