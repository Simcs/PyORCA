import numpy as np
import math
from utility import *

class Agent(object):

    def __init__(self, sim):

        self.simulator = sim

        self.id = 0
        self.max_neighbors = 0
        self.neighbor_dist = 0.0
        self.max_speed = 0.0
        self.radius = 0.0
        self.time_horizon = 0.0

        self.new_velocity = None
        self.pref_velocity = None
        self.position = None
        self.velocity = None

        self.orca_lines = []
        self.agent_neighbors = []

    def computeNeighbors(self):
        self.agent_neighbors = []
        self.simulator.computeAgentNeighbors(self)

    
    def computeNewVelocity(self):
        self.orca_lines = []
        inv_time_horizon = 1.0 / self.time_horizon

        for other in self.agent_neighbors:
            relative_position = other.position - self.position
            relative_velocity = self.velocity - other.velocity
            dist_sq = absSq(relative_position)
            combined_radius = self.radius + other.radius
            combined_radius_sq = combined_radius ** 2

            line = Line()
            
            if dist_sq > combined_radius_sq:
                # No collision
                w = relative_velocity - inv_time_horizon * relative_position
                w_length_sq = absSq(w)
                dot_product_1 = np.dot(w, relative_position)

                if dot_product_1 < 0.0 and dot_product_1 ** 2 > combined_radius_sq * w_length_sq:
                    # Project on cut-off circle (?)
                    w_length = math.sqrt(w_length_sq)
                    unit_w = w / w_length
                    line.direction = np.array([unit_w[1], -unit_w[0]])
                    u = (combined_radius * inv_time_horizon - w_length) * unit_w
                
                else:
                    # Project on legs
                    leg = math.sqrt(dist_sq - combined_radius_sq)
                    if det(relative_position, w) > 0.0:
                        # Project on left leg
                        line.direction = np.array([
                            relative_position[0] * leg - relative_position[1] * combined_radius,
                            relative_position[0] * combined_radius + relative_position[1] * leg,
                        ]) / dist_sq
                    else:
                        # Project on right leg
                        line.direction = -np.array([
                            relative_position[0] * leg - relative_position[1] * combined_radius,
                            -relative_position[0] * combined_radius + relative_position[1] * leg,
                        ]) / dist_sq

                    dot_product_2 = np.dot(relative_velocity, line.direction)
                    u = dot_product_2 * line.direction - relative_velocity

            else:
                # Collision. Project on cut-off circle of time timeStep
                inv_time_step = 1.0 / self.simulator.time_step

                w = relative_velocity - inv_time_step * relative_position
                w_length = abs(w)
                unit_w = w / w_length

                line.direction = np.array([unit_w[1], -unit_w[0]])
                u = (combined_radius * inv_time_step - w_length) * unit_w
            
            line.point = self.velocity + 0.5 * u
            self.orca_lines.append(line)
        
        line_fail, result = linearProgram2(self.orca_lines, self.max_speed, self.pref_velocity, False, self.new_velocity)
        self.new_velocity = result

        if line_fail < len(self.orca_lines):
            result = linearProgram3(self.orca_lines, line_fail, self.max_speed, self.new_velocity)
            self.new_velocity = result
    
    def update(self):
        self.velocity = self.new_velocity
        self.position += self.velocity * self.simulator.time_step
                    
