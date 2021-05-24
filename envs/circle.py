import numpy as np

from envs.base import BaseEnv


colors = [
    [229, 20, 0],
    [244, 114, 208],
    [27, 161, 226],
    [96, 169, 23],
    [0, 80, 239],
    [109, 135, 100],
    [23, 46, 127],
    [170, 0, 255]
]

class Circle(BaseEnv):

    def __init__(self, simulator):
        super(Circle, self).__init__(simulator)

        self.num_agents = 8
        self.radius = 20

    def setUpScenario(self):
        self.simulator.time_step = 0.25
        self.simulator.setAgentDefaults(
            time_horizon=2.0,
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
            self.colors.append(np.array(colors[i % 8]) / 255)

if __name__ == "__main__":
    simulator = RVOSimulator()
    scenario = Circle(simulator)
    scenario.setUpScenario()

    while not scenario.reachedGoal():
        scenario.printAgentsInfo()
        scenario.setPreferredVelocities()
        simulator.doStep()
    print('All agents reached goal!')
