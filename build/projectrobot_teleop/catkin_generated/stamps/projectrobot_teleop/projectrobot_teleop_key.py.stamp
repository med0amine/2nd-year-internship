import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

ROBOT_MAX_LIN_VEL = 0.22
ROBOT_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
teleoperation:
        z
   q    s    d
        x
"""

e = """
Communications Error
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(linear_vel, angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel, angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -ROBOT_MAX_LIN_VEL, ROBOT_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('projectrobot_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    status = 0
    linear_vel   = 0.0
    angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'z' :
                linear_vel = checkLinearLimitVelocity(linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(linear_vel,angular_vel))
            elif key == 'x' :
                linear_vel = checkLinearLimitVelocity(linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(linear_vel,angular_vel))
            elif key == 'a' :
                angular_vel = checkAngularLimitVelocity(angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(linear_vel,angular_vel))
            elif key == 'd' :
                angular_vel = checkAngularLimitVelocity(angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(linear_vel,angular_vel))
            elif key == ' ' or key == 's' :
                linear_vel   = 0.0
                control_linear_vel  = 0.0
                angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(linear_vel, angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 30 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
