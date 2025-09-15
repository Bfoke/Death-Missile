import numpy as np

'''
Coordinate system for everything is:
Origin at center of rear axle on ground plane
X - forward
Y - Left
Z - Up
'''
class Wishbone:
    def __init__(self, front: np.array, rear: np.array, balljoint: np.array):
        #add rotation limits to restrict travel
        self.front = front
        self.rear = rear
        self.balljoint = balljoint

    def axis_of_rot(self):
        vec = self.front - self.rear
        mag = np.linalg.norm(vec)

        if mag == 0:
            raise ValueError("axis_of_rot unit vec length = 0")
        
        unit_vec = vec / mag
        return unit_vec
    
    def balljoint_pos(self):
        return self.balljoint
    
    def rotation(self, theta):
        axis = self.axis_of_rot()

        translated_point = self.balljoint - self.rear
        
        ux, uy, uz = axis
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        one_minus_cos = 1 - cos_t

        R = np.array([
            [cos_t + ux**2 * one_minus_cos,
            ux * uy * one_minus_cos - uz * sin_t,
            ux * uz * one_minus_cos + uy * sin_t],

            [uy * ux * one_minus_cos + uz * sin_t,
            cos_t + uy**2 * one_minus_cos,
            uy * uz * one_minus_cos - ux * sin_t],

            [uz * ux * one_minus_cos - uy * sin_t,
            uz * uy * one_minus_cos + ux * sin_t,
            cos_t + uz**2 * one_minus_cos]
        ])

        # Step 4: Rotate the translated point
        rotated_translated_point = R @ translated_point

        # Step 5: Translate back to the original coordinate system
        rotated_point = rotated_translated_point + self.rear

        #update balljoint position
        # self.balljoint = rotated_point
        return rotated_point

class Upright:
    def __init__(self, upper_balljoint, lower_balljoint, toe_link, axle_base, axle_tip):
        self.upper_balljoint = upper_balljoint
        self.lower_balljoint = lower_balljoint
        self.toe_link = toe_link
        self.axle_base = axle_base
        self.axle_tip = axle_tip
        self.axle_vec = axle_tip - axle_base
        self.joint_dist = np.linalg.norm(upper_balljoint - lower_balljoint)

    def kingpin_rotate(self):
        return 1
    
class Corner:
    def __init__(self, upper_wb, lower_wb, upright):
        self.u_wb = upper_wb
        self.l_wb = lower_wb
        self.upright = upright

    def rotate(self,theta, dTheta=.01):
        upper_pos = self.u_wb.rotation(theta)
        dtheta = np.deg2rad(dTheta)
        max_angle = np.deg2rad(20)
        steps = int(max_angle/dtheta)

        theta = 0
        theta_close = 0
        closest_dist = 0
        jd = self.upright.joint_dist

        for i in range(steps):
            lower_pos = self.l_wb.rotation(theta)
            dist = np.linalg.norm(upper_pos - lower_pos)
            theta += dtheta
        
            if abs(dist - jd) < abs(closest_dist - jd):
                closest_dist = dist
                theta_close = theta

        return (self.l_wb.rotation(theta_close))

class Rack:
    def __init__(self, right, left, range, rotations):
        #lets you initialize fucked up angled and/or off center rack but don't do that
        self.right = right #3d point
        self.left = left #3d point
        self.range = range #distance a tie rod end moves when going from lock to lock, is equal to 2x the distance from center
        self.rotations = rotations

    def steer(self, steer_dist):
        # positive = left turn
        # negative = right turn

        if steer_dist > (self.range/2):
            raise ValueError("steering exceeds rack range")
        str = np.array([0,steer_dist,0])
        right_rod_end = self.right + str
        left_rod_end = self.left + str
        return left_rod_end, right_rod_end
    
    # add another function later to return steering wheel theta as function of rack travel using range and rotations
    # input: current right or left tie rod pos 
    # output: wheel theta

class TieRod:
    # use the same tie rod length for both sides if rack is centered (probably should be)
    def __init__(self, length):
        self.length = length