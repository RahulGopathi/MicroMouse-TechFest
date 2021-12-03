#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import random

# Global Variables
sensor_l, sensor_c, sensor_r = 0, 0, 0
pub = 0 

def clbk_laser(msg):
    global sensor_l, sensor_c, sensor_r
    # 720 / 5 = 144
    regions = [
        round(100*min(min(msg.ranges[0:72]), 100)),
        round(100*min(min(msg.ranges[72:144]), 100)),
        round(100*min(min(msg.ranges[144:216]), 100)),
        round(100*min(min(msg.ranges[216:288]), 100)),
        round(100*min(min(msg.ranges[288:360]), 100)),
    ]
    # rospy.loginfo(regions)
    print("l: {} \t c: {} \t r: {}".format(regions[4], regions[2], regions[0]))
    
    sensor_l = regions[4]
    sensor_c = regions[2]
    sensor_r = regions[0]

    # print("l: {} \t c: {} \t r: {}".format(sensor_l, sensor_c, sensor_r))

def motion_go_straight():
    global pub
    msg = Twist()
    msg.linear.x = 0.1
    pub.publish(msg)

def motion_stop():
    global pub
    msg = Twist()
    msg.linear.x = 0.0
    pub.publish(msg)

# def clbk_laser(msg):
#     """
#     Read sensor messagens, and determine distance to each region. 
#     Manipulates the values measured by the sensor.
#     Callback function for the subscription to the published Laser Scan values.
#     """
#     global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall
#     size = len(msg.ranges)
#     min_index = int(size*(direction+1)/4)
#     max_index = int(size*(direction+3)/4)

#     # Determine values for PD control of distance and P control of angle
#     for i in range(min_index, max_index):
#         if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
#             min_index = i
#     angle_min = (min_index-size/2)*msg.angle_increment
#     dist_min = msg.ranges[min_index]
#     dist_front = msg.ranges[int(size/2)]
#     diff_e = min((dist_min - wall_dist) - e, 100)
#     e = min(dist_min - wall_dist, 100)

#     # Determination of minimum distances in each region
#     regions_ = {
#         'bright':  min(mean(msg.ranges[0:143]), inf),
#         'right': min(mean(msg.ranges[144:287]), inf),
#         'fright':  min(mean(msg.ranges[288:431]), inf),
#         'front':  min(mean(msg.ranges[432:575]), inf),
#         'fleft':   min(mean(msg.ranges[576:719]), inf),
#         'left':   min(mean(msg.ranges[720:863]), inf),
#         'bleft':   min(mean(msg.ranges[864:1007]), inf),
#     }
#     # regions_ = {
#     #     'bright':  min(min(msg.ranges[0:143]), inf),
#     #     'right': min(min(msg.ranges[144:287]), inf),
#     #     'fright':  min(min(msg.ranges[288:431]), inf),
#     #     'front':  min(min(msg.ranges[432:575]), inf),
#     #     'fleft':   min(min(msg.ranges[576:719]), inf),
#     #     'left':   min(min(msg.ranges[720:863]), inf),
#     #     'bleft':   min(min(msg.ranges[864:1007]), inf),
#     # }
#     # rospy.loginfo(regions_)

#     # Detection of Outer and Inner corner
#     bool_outer_corner = is_outer_corner()
#     bool_inner_corner = is_inner_corner()
#     if bool_outer_corner == 0 and bool_inner_corner == 0:
#         last_kinds_of_wall[index] = 0

#     # Indexing for last five pattern detection
#     # This is latter used for low pass filtering of the patterns
#     index = index + 1  # 5 samples recorded to asses if we are at the corner or not
#     if index == len(last_kinds_of_wall):
#         index = 0

#     # take_action()

# def following_wall():
#     """
#     PD control for the wall following state. 
#     Returns:
#             Twist(): msg with angular and linear velocities to be published
#                     msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
#                     msg.angular.z -> PD controller response
#     """
#     global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
#     msg = Twist()
#     if dist_front < wall_dist:
#         msg.linear.x = 0
#     elif dist_front < wall_dist*2:
#         msg.linear.x = 0.5*max_speed
#     elif abs(angle_min) > 1.75:
#         msg.linear.x = 0.4*max_speed
#     else:
#         msg.linear.x = max_speed
#     msg.angular.z = max(min(direction*(p*e+d*diff_e) +
#                             angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
#     # print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
#     return msg


# def change_direction():
#     """
#     Toggle direction in which the robot will follow the wall
#         1 for wall on the left side of the robot and -1 for the right side
#     """
#     global direction, last_change_direction, rotating
#     # Elapsed time since last change direction
#     elapsed_time = time.time() - last_change_direction_time
#     if elapsed_time >= 20:
#         last_change_direction = time.time()
#         direction = -direction  # Wall in the other side now
#         rotating = 1

def main():
    
    global sensor_l, sensor_c, sensor_r
    global pub

    msg = Twist()

    rospy.init_node('node_maze_runner')
    
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # pub.publish(msg)
    
    # rospy.spin()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print("l: {} \t c: {} \t r: {}".format(sensor_l, sensor_c, sensor_r))
        
        if(sensor_c > 15):
            motion_go_straight()
        else:
            motion_stop()

        rate.sleep()

if __name__ == '__main__':
    main()
