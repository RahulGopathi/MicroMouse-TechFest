#! /usr/bin/env python
from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

unit_distance = 0.1795
ctr = 0
x = 0.0
y = 0.0
x_init = 0.0
y_init = 0.0
theta = 0.0
hz = 20                     # Cycle Frequency
loop_index = 0              # Number of sampling cycles
# Limit to Laser sensor range in meters, all distances above this value are
inf = 10
# considered out of sensor range
wall_dist = 0.08            # Distance desired from the wall
max_speed = 0.3            # Maximum speed of the robot on meters/seconds
p = 15                      # Proportional constant for controller
d = 1                       # Derivative constant for controller
# Proportional constant for angle controller (just simple P controller)
angle = 1
# 1 for wall on the left side of the robot (-1 for the right side)
direction = -1
e = 0                       # Diference between current wall measurements and previous one
# Angle, at which was measured the shortest distance between the robot and a wall
angle_min = 0
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

pub_ = None
sub = None
sub_ = None

def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)


def clbk_laser(msg):
    """
    Read sensor messages, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall
    size = len(msg.ranges)
    min_index = int(size*(direction+1)/4)
    max_index = int(size*(direction+3)/4)

    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
            min_index = i

    angle_min = (min_index-size/2)*msg.angle_increment
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[int(size/2)]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)

def following_wall():
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    speed = Twist()
    if dist_front < wall_dist:
        speed.linear.x = 0
    elif dist_front < wall_dist*2:
        speed.linear.x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        speed.linear.x = 0.4*max_speed
    else:
        speed.linear.x = max_speed
        
    speed.angular.z = max(min(direction*(p*e+d*diff_e) +
                        angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
    return speed

def newOdom (msg):
    global x
    global y
    global ctr
    global x_init
    global y_init

    if(ctr == 0):
        x_init = msg.pose.pose.position.x
        y_init = msg.pose.pose.position.y
        ctr+=1
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


def main():
    global pub_, active_, hz, loop_index, sub, sub_

    rospy.init_node('move', "speed_controller")

    sub = rospy.Subscriber("/odom", Odometry, newOdom)

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_ = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)

    speed = Twist()

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        loop_index = loop_index + 1

        distance = math.sqrt((y - y_init)**2 + (x - x_init)**2)

        if (distance > unit_distance):
            speed.linear.x = 0
            speed.angular.z = 0
        else:
            print("distance = ", distance)
            print("y = ", y)
            speed = following_wall()

        pub_.publish(speed)


        rate.sleep()


if __name__ == '__main__':
    main()