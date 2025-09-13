import numpy as np

class Wishbone:
    def __init__(self, front: np.array, rear: np.array, balljoint: np.array):
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
        # Step 1: Get the unit vector of the axis
        axis = self.axis_of_rot()

        # Step 2: Translate the point so axis_point1 becomes the origin
        translated_point = self.balljoint - self.rear
        
        # Step 3: Build the rotation matrix using Rodrigues' rotation formula
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

    def rotate(theta):
        return 1

# define chassis pickup points
# origin at center of rear axle on ground
# F/R - front/rear of car
# r/l - right/left
# U/L - upper/lower
# 1/2/3 - 1=front/2=rear/3=upright of the specific wishbone

# Should make control arm and upright classes to make this easier

x1 = np.array([1,0,0])
x2 = np.array([0,0,0])
x3 = np.array([.5,-.5,0])

test_wishbone = Wishbone(x1, x2, x3)

# front right corner
# upper
FrU1 = np.array([1.9,-.4,.4])
FrU2 = np.array([1.7,-.4,.4])
FrU3 = np.array([1.8,-.6,.4])

# lower
FrL1 = np.array([1.9,-.4,.2])
FrL2 = np.array([1.7,-.4,.2])
FrL3 = np.array([1.8,-.6,.2])

FR_upper = Wishbone(FrU1, FrU2, FrU3)
FR_lower = Wishbone(FrL1, FrL2, FrL3)


# front right upright
joint_dist = .5
# define coords for ball joints and toe link so u can apply the same axis/ rotation checks to find 
# how far the upright rotated
# also define axle somehow to do same kind of math for wheel angles

theta = np.deg2rad(-90)
# print(theta)
print(FR_upper.rotation(theta))
print(test_wishbone.rotation(theta))