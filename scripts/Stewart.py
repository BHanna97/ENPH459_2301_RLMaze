import numpy as np

class Stewart:

    """
    Based on https://cdn.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf,
    With constraint that the linear translation of platform center is 0.
    For our specific platform, servo angles shall not exceed 72 degrees.
    The zero position for our platform is such that all servos are at 45 degree angles.

    Parameters:
        bi - set of 3 vectors describing the coordinates of each base anchor point wrt base origin
        pi - set of 3 vectors describing the coordinates of each platform anchor point wrt platform origin
        s - length of actuating leg
        a - length of servo operating arm
        h0 - vertical displacement from base center
        beta_i - set of 3 angles between plane of actuation and x-axis for each servo in degrees
        servo_0 - Angle for which servo arms are aligned with x-y plane
    """

    def __init__(self, bi, pi, s, a, beta_i, h0, servo_0):
        self.b1, self.b2, self.b3 = bi[0], bi[1], bi[2]
        self.p1, self.p2, self.p3 = pi[0], pi[1], pi[2]
        self.s = s
        self.a = a
        self.beta1, self.beta2, self.beta3 = np.pi/180*beta_i[0], np.pi/180*beta_i[1], np.pi/180*beta_i[2]
        self.h0 = h0
        self.servo_0 = servo_0

    def orient(self, roll, pitch):
        """
        :param roll: angle about x-axis in degrees
        :param pitch: angle about y-axis in degrees
        :return: servo angles required to achieve this orientation in degrees
        """
        # 3D generalization, not compatible with our platform
        # rotator_column1 = np.array([np.cos(yaw)*np.cos(pitch),
        #                             np.sin(yaw)*np.cos(pitch),
        #                             -1*np.sin(pitch)])
        # rotator_column2 = np.array([np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(roll),
        #                             np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll),
        #                             np.cos(pitch)*np.sin(roll)])
        # rotator_column3 = np.array([np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll),
        #                             np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(pitch),
        #                             np.cos(pitch)*np.cos(roll)])

        roll = roll * np.pi/180
        pitch = pitch * np.pi/180

        rotator = np.array([[np.cos(pitch), np.sin(pitch)*np.sin(roll), np.sin(pitch)*np.cos(roll)],
                            [0, np.cos(roll), -1*np.sin(roll)],
                            [-1*np.sin(pitch), np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(roll)]])

        q1 = np.array([0, 0, self.h0]) + rotator@self.p1
        q2 = np.array([0, 0, self.h0]) + rotator@self.p2
        q3 = np.array([0, 0, self.h0]) + rotator@self.p3

        leg1 = q1 - self.b1
        leg2 = q2 - self.b2
        leg3 = q3 - self.b3

        M1 = 2*self.a*leg1[2]
        M2 = 2*self.a*leg2[2]
        M3 = 2*self.a*leg3[2]

        N1 = 2*self.a*(np.cos(self.beta1)*leg1[0] + np.sin(self.beta1)*(leg1[1]))
        N2 = 2*self.a*(np.cos(self.beta2)*leg2[0] + np.sin(self.beta2)*(leg2[1]))
        N3 = 2*self.a*(np.cos(self.beta3)*leg3[0] + np.sin(self.beta3)*(leg3[1]))

        k = self.s**2 - self.a**2
        L1 = np.dot(leg1, leg1) - k
        L2 = np.dot(leg2, leg2) - k
        L3 = np.dot(leg3, leg3) - k

        alpha1 = 180/np.pi*(np.arcsin(L1/np.sqrt(M1**2 + N1**2)) + np.arctan(M1/N1)) - self.servo_0
        alpha2 = 180/np.pi*(np.arcsin(L2/np.sqrt(M2**2 + N2**2)) + np.arctan(M2/N2)) - self.servo_0
        alpha3 = 180/np.pi*(np.arcsin(L3/np.sqrt(M3**2 + N3**2)) + np.arctan(M3/N3)) - self.servo_0

        return alpha1, alpha2, alpha3

## 20cm across, center point of attachment sillhouette ~.69 (1.75 cm) inches in from edge
## radial: