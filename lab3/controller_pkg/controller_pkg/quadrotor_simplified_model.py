from casadi import MX, vertcat, horzcat, diag, inv_minor, cross, sqrt, cos, sin, times
import numpy as np

class QuadrotorSimplified:
    def __init__(self, mass, arm_length, Ixx, Iyy, Izz, tau, gravity=9.80665):
        self.mass = mass
        self.gravity = gravity
        self.arm_length = arm_length
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.tau = tau
    
    def dynamics(self):
        px = MX.sym('px')
        py = MX.sym('py')
        pz = MX.sym('pz')
        vx = MX.sym('vx')
        vy = MX.sym('vy')
        vz = MX.sym('vz')
        roll = MX.sym('roll')
        pitch = MX.sym('pitch')
        yaw = MX.sym('yaw')
        roll_c = MX.sym('roll_c')
        pitch_c = MX.sym('pitch_c')
        yaw_c = MX.sym('yaw_c')
        thrust = MX.sym('thrust')

        x = vertcat(px, py, pz, vx, vy, vz, roll, pitch, yaw)
        u = vertcat(roll_c, pitch_c, yaw_c, thrust)

        #Position kinematics
        pxdot = vx
        pydot = vy
        pzdot = vz

        #Rotation matrix 
        R = vertcat(
            horzcat(cos(yaw)*cos(pitch),
                    cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),
                    cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll)),
            horzcat(sin(yaw)*cos(pitch),
                    sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),
                    sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll)),
            horzcat(-sin(pitch),
                    cos(pitch)*sin(roll),
                    cos(pitch)*cos(roll))
        )

        thrust_b = vertcat(0, 0, thrust)
        acc = (1/self.mass) * mtimes(R, thrust_b) - vertcat(0, 0, self.gravity)
        vxdot = acc[0]
        vydot = acc[1]
        vzdot = acc[2]

        #Attitude dynamics
        rolldot = (roll_c - roll) / self.tau
        pitchdot = (pitch_c - pitch) / self.tau
        yawdot = (yaw_c - yaw) / self.tau

        f_expl = vertcat(pxdot, pydot, pzdot, vxdot, vydot, vzdot, rolldot, pitchdot, yawdot)

        return (f_expl, x, u)
