import numpy as np

from envs.base import BaseEnv


l_to_r = [
    ([60, 0], [-90, 0]),
    ([70, 35], [-80, 35]),
    ([80, 20], [-70, 20]),
    ([90, 45], [-60, 45]),
    ([100, -40], [-50, -40]),
    ([100, 50], [-50, 50]),
    ([90, -5], [-60, -5]),
    ([80, 5], [-70, 5]),
    ([70, -30], [-80, -30]),
    ([60, 25], [-90, 25]),
]
c_lr = [244, 114, 208]

r_to_l = [
    ([-60, 0], [90, 0]),
    ([-70, 35], [+80, 35]),
    ([-80, 20], [70, 20]),
    ([-90, 45], [60, 45]),
    ([-100, -40], [50, -40]),
    ([-100, 50], [50, 50]),
    ([-90, -5], [60, -5]),
    ([-80, 5], [70, 5]),
    ([-70, -30], [80, -30]),
    ([-60, 25], [90, 25]),
]
c_rl = [27, 161, 226]

class Crossing(BaseEnv):

    def __init__(self, simulator):
        super(Crossing, self).__init__(simulator)

        self.num_agents = 20

    def setUpScenario(self):
        self.simulator.time_step = 0.1
        self.simulator.setAgentDefaults(
            time_horizon=10.0,
            radius=2.0,
            max_speed=2.0,
            velocity=np.array([0., 0.]),
        )

        for pos, goal in l_to_r:
            self.simulator.addAgent(np.array(pos, dtype=np.float64))
            self.goals.append(np.array(goal, dtype=np.float64))
            self.colors.append(np.array(c_lr) / 255)
        
        for pos, goal in r_to_l:
            self.simulator.addAgent(np.array(pos, dtype=np.float64))
            self.goals.append(np.array(goal, dtype=np.float64))
            self.colors.append(np.array(c_rl) / 255)