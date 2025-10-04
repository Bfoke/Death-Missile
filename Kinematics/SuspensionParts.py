import numpy as np
import jax.numpy as jnp
from scipy.optimize import minimize_scalar
from scipy.optimize import root_scalar
'''
Coordinate system for everything is:
Origin at center of rear axle on ground plane
X - Forward
Y - Left
Z - Up

Use Kabsch algorithm for rigid body translation + rotation of upright
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
        self.upper_toe_dist = np.linalg.norm(upper_balljoint - toe_link)
        self.lower_toe_dist = np.linalg.norm(lower_balljoint - toe_link)

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

    def travel_old(self, theta, steps = 1000, theta_range = 20): #manual stepping through fixed theta range
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
    
    def wishbone_travel(self, upper_theta, theta_bounds=(-0.5, 0.5)):
        """
        Solve for lower wishbone theta such that the distance between upper and
        lower balljoints equals the upright joint distance.
        """

        # Fixed position of upper balljoint
        upper_pos = self.u_wb.rotation(upper_theta)
        joint_dist = self.upright.joint_dist

        # Define the function whose root we want to find
        def f(theta_l):
            lower_pos = self.l_wb.rotation(theta_l)
            dist = jnp.linalg.norm(upper_pos - lower_pos)
            return dist - joint_dist

        # Use root-finding to solve f(theta) = 0
        result = root_scalar(f, method='brentq', bracket=theta_bounds)

        if not result.converged:
            raise RuntimeError("Failed to solve for lower wishbone theta")

        best_theta = result.root
        return self.l_wb.rotation(best_theta)
    
    def tie_rod_pos_solve(self, upper_theta, rack_pos, tieRod, upright):
        # use "trilaterate" method to find position of toe link on the upright

        #find upper wishbone balljoint pos
        upper_pos = self.u_wb.rotation(upper_theta)
        upper_dist = upright.upper_toe_dist
        
        #find lower wishbone balljoint pos
        lower_pos = self.wishbone_travel(upper_theta)
        lower_dist = upright.lower_toe_dist

        #rack_pos gives third point
        tie_rod_dist = tieRod.length

        P1, P2, P3 = map(np.array, (upper_pos, lower_pos, rack_pos))

        # Create unit vectors
        ex = (P2 - P1)
        ex /= np.linalg.norm(ex)
        i = np.dot(ex, P3 - P1)
        temp = P3 - P1 - i * ex
        ey = temp / np.linalg.norm(temp)
        ez = np.cross(ex, ey)

        d = np.linalg.norm(P2 - P1)
        j = np.dot(ey, P3 - P1)

        # Coordinates in the new system
        x = (upper_dist**2 - lower_dist**2 + d**2) / (2 * d)
        y = (upper_dist**2 - tie_rod_dist**2 + i**2 + j**2 - 2 * i * x) / (2 * j)

        # Solve for z^2, and check if real solution exists
        z_sq = upper_dist**2 - x**2 - y**2
        if z_sq < 0:
            raise ValueError("No real solution exists (spheres don't intersect)")

        z = np.sqrt(z_sq)

        # Convert back to original coordinate system
        result1 = P1 + x * ex + y * ey + z * ez
        result2 = P1 + x * ex + y * ey - z * ez

        return result1 if result1[0] > result2[0] else result2 # this returns the solution where the tie rod is more forward corresponding with rack in front of wheels

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