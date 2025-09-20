import numpy as np

'''
Coordinate system for everything is:
Origin at center of rear axle on ground plane
X - Forward
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

        return rotated_point #upright balljoint location

class Upright:
    def __init__(self, upper_balljoint, lower_balljoint, toe_root, toe_link, axle_root, axle_tip):
        # roots are points along the vector between upper and lower balljoint that the orthogonal projection of toe_link and axle tip go to
        # they are used to locate the toe_link and axle_tip relative to the balljoint vector
        self.upper_balljoint = upper_balljoint
        self.lower_balljoint = lower_balljoint
        self.toe_root = toe_root
        self.toe_link = toe_link
        self.axle_root = axle_root
        self.axle_tip = axle_tip
        self.axle_vec = axle_tip - axle_root
        self.joint_dist = np.linalg.norm(upper_balljoint - lower_balljoint)

    def kingpin_rotate(self, theta, upper_bj, lower_bj, toeLink, axleTip):
        vec = upper_bj - lower_bj
        mag = np.linalg.norm(vec)

        if mag == 0:
            raise ValueError("axis_of_rot unit vec length = 0")
        
        axis = vec / mag

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

        rotated_toeLink = R @ (toeLink - lower_bj) + lower_bj
        rotated_axleTip = R @ (axleTip - lower_bj) + lower_bj

        return rotated_toeLink, rotated_axleTip
    
class Corner:
    def __init__(self, upper_wb, lower_wb, upright):
        self.u_wb = upper_wb #wishbone object
        self.l_wb = lower_wb #wishbone object
        self.upright = upright #upright object

    def travel(self, theta, steps = 1000, theta_range = 20):
        upper_pos = self.u_wb.rotation(theta) #rotate upper wishbone

        dTheta = np.linspace(0,(theta_range*np.sign(theta)), steps)

        theta_close = 0
        closest_dist = float('inf')
        jd = self.upright.joint_dist

        for dT in dTheta:
            lower_pos = self.l_wb.rotation(dT)
            dist = np.linalg.norm(upper_pos - lower_pos)
        
            if abs(dist - jd) < abs(closest_dist - jd):
                closest_dist = dist
                theta_close = dT

        return self.l_wb.rotation(theta_close) #return lower balljoint position that maintains upper/lower balljoint distance

class Rack:
    def __init__(self, right, left, range, rotations):
        #lets you make angled and/or off center rack but don't do that
        self.right = right #3d point
        self.left = left #3d point
        self.range = range #distance a tie rod end moves when going from lock to lock, is equal to 2x the distance from centered wheel
        self.rotations = rotations

    def steer(self, steer_dist):
        # positive steer_dist = left turn
        # negative steer_dist = right turn

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