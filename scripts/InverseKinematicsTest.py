import Stewart
import numpy as np


# reference angles, approximate:
# -4 degree roll independent of servo 1, with -7 degrees servo 2 and 10 degrees servo 3

# prototype params:
pbi = [[8.25, 0, 1.6], [-4.125, 7.145, 1.6], [-4.125, -7.145, 1.6]]
ppi = [[8.805, 0, 0], [-4.402, 7.625, 0], [-4.402, -7.625, 0]]
pa = 8.55
ps = 8.55
pbeta_i = [0, 120, 240]
ph0 = 5

# 100.53 mm from center
# rest position with arm flat
# platform anchors 35.346mm below platform proper.

r0 = 13.818  # cm, radial displacement from base of servo joints
arm = 8
leg = 8
servo_orientations = [0, 120, 240]  # even distribution of 3 angles wrt x
h0 = leg  # platform height when all servos lay flat
servo_0 = np.pi*45/180 # servo angle that has the arms level with the x-y plane

base_anchors = [[r0, 0, 0],
                [-1*r0*np.sin(np.pi/6), r0*np.cos(np.pi/6), 0],
                [-1*r0*np.sin(np.pi/6), -1*r0*np.cos(np.pi/6), 0]]
platform_anchors = [[(r0 + arm), 0, 0],
                    [-1*(r0 + arm)*np.sin(np.pi/6), (r0 + arm)*np.cos(np.pi/6), 0],
                    [-1*(r0 + arm)*np.sin(np.pi/6), -1*(r0+arm)*np.cos(np.pi/6), 0]]



# protostew = Stewart.Stewart(pbi, ppi, ps, pa, pbeta_i, ph0)
# angles1 = protostew.orient(0, 0)
# angles2 = protostew.orient(-4*np.pi/180, 0)

theseus = Stewart.Stewart(base_anchors, platform_anchors, leg, arm, servo_orientations, h0, )
angles1 = theseus.orient(0, 0)
angles2 = theseus.orient(10, 0)

print("flat: " + str(angles1))
print("tilt: " + str(angles2))
