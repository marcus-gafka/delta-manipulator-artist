import math
from Constants import Constants as c

class InverseKinematics:
    
    @staticmethod
    def ik(x, y):
        x += c.OFFSET_X
        y += c.OFFSET_Y

        d = math.hypot(x - c.l1, y)
        theta2_rad = math.atan2(y, x - c.l1) - math.acos((-d**2 - c.l2**2 + (c.l3 + c.a)**2) / (-2.0 * c.l2 * d))

        x3 = (c.a / (c.l3 + c.a)) * c.l2 * math.cos(theta2_rad) + (c.a / (c.l3 + c.a)) * c.l1 + (c.l3 / (c.l3 +c.a)) * x
        y3 = (c.a / (c.l3 + c.a)) * c.l2 * math.sin(theta2_rad) + (c.l3 / (c.l3 + c.a)) * y

        r = math.hypot(x3, y3)
        theta1_rad = math.atan2(y3, x3) + math.acos((-r**2 - c.l5**2 + c.l4**2) / (-2.0 * c.l5 * r))

        theta1_deg = math.degrees(theta1_rad)
        theta2_deg = math.degrees(theta2_rad)
        return theta1_deg, theta2_deg
    
    @staticmethod
    def calc_steps(x, y, currentTheta1, currentTheta2, targetTheta1, targetTheta2):
        deltaTheta1 = targetTheta1 - currentTheta1
        deltaTheta2 = targetTheta2 - currentTheta2

        steps1 = int(round(deltaTheta1 / c.DEGREES_PER_STEP))
        steps2 = int(round(deltaTheta2 / c.DEGREES_PER_STEP))

        max_steps = max(abs(steps1), abs(steps2))
        if max_steps == 0:
            interval = 0
        else:
            interval = (int) (1000 / c.MAX_STEPS_PER_SEC)

        return -steps1, -steps2, interval
