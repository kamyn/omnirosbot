import numpy as np
import math
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep
from time import sleep

left = rospy.Publisher('omnirosbot/left_joint_velocity_controller/command', Float64, queue_size=10)
right = rospy.Publisher('omnirosbot/right_joint_velocity_controller/command', Float64, queue_size=10)
back = rospy.Publisher('omnirosbot/back_joint_velocity_controller/command', Float64, queue_size=10)

#CONSTANTS
VMAX = 1.0
OMEGAMAX = 10.0
R = 0.04
r = 0.01905
P = 5.0
I = 0.001
STOP_DIST = 0.01
STOP_ANG = 0.1

 # world's coordinates
xw = 0.0
yw = 0.0
theta = 0.0
# target world's coordinates
targetX = 0.0
targetY = 0.0
targetTheta = 0.0
# linear world's velocities and angular velocity
vxw = 0.0 
vyw = 0.0
omega = 0.0
# errors
eix = 0.0
eiy = 0.0
eit = 0.0
# is robot moving
move = False

# norms for error
errorL1 = []
errorL2 = []
errorInf = []

# is showing plots
showPlot = False

def normRad(rad):
    rad = math.fmod(rad, math.pi*2)
    if rad < 0:
        rad += math.pi * 2
    return rad

def link_state(state):
    global xw, yw, theta
    basePos = state.pose[1].position
    baseOrient = state.pose[1].orientation
    xw = basePos.x
    yw = basePos.y
    (_,_,theta) = euler_from_quaternion([baseOrient.x, baseOrient.y,
                                         baseOrient.z, baseOrient.w])
    onUpdate()

def generateTrajectory(points):
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    theta = [p[2] for p in points]
    t = np.linspace(0, 1, len(points))
    sx = splrep(t, x, k=3)
    sy = splrep(t, y, k=3)
    stheta = splrep(t, theta, k=1)

    tn = np.linspace(0, 1, 50) 
    xn = splev(tn, sx)
    yn = splev(tn, sy)
    thetan = splev(tn, stheta)

    trajectory = [(x, y, th) for x, y, th in zip(xn, yn, thetan)]
    return trajectory


def moveToWorldPoint(x,y,theta):
    global targetX, targetY, targetTheta, move
    targetX = x
    targetY = y
    targetTheta = theta
    move = True

def moveBSpline(points): # points - list of points and angles
    global move, errorL2, xw, yw, theta
    trajectory = generateTrajectory(points)
    for (x,y,th) in trajectory:
        while True:
            if not move:
                print(f'move to: {x}, {y}, {th}')
                moveToWorldPoint(x,y,th)
                break 
            errorL2.append((xw - x) * (xw - x) + (yw - y) * (yw - y) + (theta - th) * (theta - th))
            sleep(1/100)

    time = np.linspace(1, len(errorL2), len(errorL2))
    #plt.plot(time, errorL1)
    plt.plot(time, errorL2) 
    #plt.plot(time, errorInf)
    #plt.legend(['L1', 'L2', 'Linf'])
    plt.xlabel('time')
    plt.ylabel('error')
    plt.title(f'P: {P}, I: {I}')
    plt.show()


def onUpdate():
    global xw, yw, theta, targetX, targetY, targetTheta, eix, eiy, eit, showPlot, STOP_DIST, STOP_ANG, move
    if move:
        # PI regulator
        ex = targetX - xw
        ey = targetY - yw
        et = math.pi - normRad(theta + math.pi - targetTheta)
        eix += ex
        eiy += ey
        eit += et  
        
        vxw = ex * P + eix * I
        vyw = ey * P + eiy * I
        omega = et * P + eit * I

        vxa = abs(vxw)
        vya = abs(vyw)
        if vxa > VMAX or vya > VMAX:
            scale = VMAX / (vxa if vxa > vya else vya)
            vxw *= scale
            vyw *= scale
        if omega > OMEGAMAX:
            omega = OMEGAMAX

        # inverse kinematics
        vxm = math.cos(theta) * vxw + math.sin(theta) * vyw
        vym = math.cos(theta) * vyw - math.sin(theta) * vxw
        vxw = vxm
        vyw = vym

        omegaL = omega * R
        sqrtVym2 = math.sqrt(3)/2 * vyw
        vxm2 = vxw / 2.0
        omegaLVxm2 = omegaL - vxm2

        left.publish((omegaLVxm2 - sqrtVym2)/r)
        right.publish((omegaLVxm2 + sqrtVym2)/r)
        back.publish((omegaL + vxw)/r) 

        if max(ex, ey) < STOP_DIST and et < STOP_ANG:
            left.publish(0.0)
            right.publish(0.0)
            back.publish(0.0)
            move = False
            eix = eiy = eit = 0.0

        # add errors to plot
        #errorL1.append(abs(ex) + abs(ey))
        #errorL2.append(ex*ex + ey*ey)
        #errorInf.append(max(abs(ex), abs(ey)))
'''
    if not showPlot and not move:
        time = np.linspace(1, len(errorL2), len(errorL2))
        plt.plot(time, errorL1)
        plt.plot(time, errorL2) 
        plt.plot(time, errorInf)
        plt.legend(['L1', 'L2', 'Linf'])
        plt.xlabel('time')
        plt.ylabel('error')
        plt.title(f'P: {P}, I: {I}')
        plt.show()
        showPlot = True 
'''    

def start():
    rospy.init_node('omnirosbot_controller')
    rate = rospy.Rate(20)
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_state)
    #moveToWorldPoint(0.5, 0.8, 1.57)
    moveBSpline([(0.2, 0.3, 1.57), (0.5, 0.2, 0.0), (0.7, 0.8, 1.57), (1.0, 1.0, 0.0)])
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
