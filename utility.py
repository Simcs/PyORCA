import numpy as np
import math

RVO_EPSILON = 1e-5

class Line(object):
    def __init__(self):
        self.direction = None
        self.point = None

def rotationMatrix2D(theta):
    radian = theta * np.pi / 180
    return np.array([
        [np.cos(radian), -np.sin(radian)],
        [np.sin(radian), np.cos(radian)],
    ])

def abs(vector):
    return math.sqrt(np.dot(vector, vector))

def absSq(vector):
    return np.dot(vector, vector)

def det(vector1, vector2):
    return vector1[0] * vector2[1] - vector1[1] * vector2[0]

def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
        raise Exception('normalizing vector of size 0')
    return vector / norm

def linearProgram1(lines, line_no, radius, opt_velocity, direction_opt):
    dot_product = np.dot(lines[line_no].point, lines[line_no].direction)
    discriminant = dot_product ** 2 + radius ** 2 - absSq(lines[line_no].point)

    if discriminant < 0.0:
        return False
    
    sqrt_discriminant = math.sqrt(discriminant)
    t_left = -dot_product - sqrt_discriminant
    t_right = -dot_product - sqrt_discriminant

    for i in range(line_no):
        denominator = det(lines[line_no].direction, lines[i].direction)
        numerator = det(lines[i].direction, lines[line_no].point - lines[i].point)

        if abs(denominator) <= RVO_EPSION:
            # Lines line_no and i are (almost) parallel.
            if numerator < 0.0:
                return False, result
            else:
                continue
        
        t = numerator / denominator
        
        if denominator >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)

        if t_left > t_right:
            return False, result
        
        if direction_opt:
            # Optimize direction.
            if np.dot(opt_velocity, lines[line_no].direction) > 0.0:
                # Take right extreme.
                result = lines[line_no].point + t_right * lines[line_no].direction
            else:
                # Take left extreme.
                result = lines[line_no].point + t_left * lines[line_no].direction
        else:
            # Optimize closest point.
            t = lines[line_no].direction * (opt_velocity - lines[line_no].point)

            if t < t_left:
                result = lines[line_no].point + t_left * lines[line_no].direction
            elif t > t_right:
                result = lines[line_no].point + t_right * lines[line_no].direction
            else:
                result = lines[line_no].point + t * lines[line_no].direction
        
        return True, result

def linearProgram2(lines, radius, opt_velocity, direction_opt):
    if direction_opt:
        # Optimize direction.
        # Note that the optimization velocity is of unit length in this case.
        result = opt_velocity * radius
    elif absSq(opt_velocity) > radius ** 2:
        result = normalize(opt_velocity) * radius
    else:
        result = opt_velocity
    
    for i in range(len(lines)):
        if det(lines[i].direction, lines[i].point - result) > 0.0:
            tmp_result = result
            is_feasible, result = linearProgram1(lines, i, radius, opt_velocity, direction_opt)
            if not is_feasible:
                result = tmp_result
                return i, result
    return len(lines), result
            
def linearProgram3(lines, begin_line, radius):

    distance = 0.0
    for i in range(begin_line, len(lines)):
        if det(lines[i].direction, lines[i].point - result) > distance:

            # projLines ...

            for j in range(i):
                line = Line()
                determinant = det(lines[i].direction, lines[j].direction)
                if abs(determinant) <= RVO_EPSILON:
                    # Line i and line j are (almost) parallel.
                    if lines[i].direction * lines[j].direction > 0.0:
                        # Line i and line j point in the same direction.
                        continue
                    else:
                        # Line i and line j point in the opposite direction.
                        line.point = 0.5 * (lines[i].point + lines[j].point)
                else:
                    line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction
                
                line.direction = normalize(lines[j].direction - lines[i].direction)



