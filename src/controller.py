import numpy as np
import math
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion

left = rospy.Publisher('omnirosbot/left_joint_velocity_controller/command', Float64, queue_size=10)
right = rospy.Publisher('omnirosbot/right_joint_velocity_controller/command', Float64, queue_size=10)
back = rospy.Publisher('omnirosbot/back_joint_velocity_controller/command', Float64, queue_size=10)

#CONSTANTS
VMAX = 1.0
OMEGAMAX = 3.0
R = 0.04
r = 0.01905
P = 1.0
I = 0.0001

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
#errors
eix = 0.0
eiy = 0.0
eit = 0.0

# is robot moving
move = False 

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

def moveToWorldPoint(x,y,theta):
    global targetX, targetY, targetTheta, move
    targetX = x
    targetY = y
    targetTheta = theta
    move = True

def onUpdate():
    global xw, yw, theta, targetX, targetY, targetTheta, eix, eiy, eit 
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
        

def start():
    rospy.init_node('omnirosbot_controller')
    rate = rospy.Rate(20)
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_state)
    moveToWorldPoint(0.5, 0.8, 1.57)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
