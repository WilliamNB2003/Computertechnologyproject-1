#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #


import rospy
import math
import time
from random import randrange
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios


import smbus
import RPi.GPIO as GPIO

bus = smbus.SMBus(1)
time.sleep(1)
bus.write_byte_data(0x44, 0x01, 0x05)



GPIObus = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIObus,GPIO.OUT)


ANGULAR_VEL = 3
LINEAR_VEL = 1
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05

SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


def ProcentLinear(distance):
    # y = (5*distance**2)-0.2 #2.grads
    y = (10/3 *  (distance-0.2)) # linear

    # y = (1/(1+(2.718281828459045**(-20*(distance-0.384871))))) # sigmoid
    # y = (91.44*distance**3 -78.45*distance**2+22.31*distance-2.07) # 3. grads
    if(y < 0):
        y = 0
    if(y > 1):
        y = 1
    return y

def ProcentAngular(distance):
    return 1.0 - ProcentLinear(distance)

class Obstacle():

    def is_key_available(self):
        return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])
            
    def get_key(self):
        return sys.stdin.read(1)

    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 90            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)


        for i in range(samples_view):

            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5

            # elif(scan_filter[i] == 0.0):
            #     scan_filter[i] = 1000

            elif math.isnan(scan_filter[i]):
                print("NaN")
                scan_filter[i] = 0.0

        
        return scan_filter

    def obstacle(self):
        averageLinSpeed = 0
        temp_sum = 0
        dir = [0,0,0]
        RawDistance = [0,0,0]
        twist = Twist()
        colliding = False
        first_hit = True
        iteration = 1
        Speed_Accumilation = 0
        Collision_counter = 0
        Collision_radius = 0.13
        Victim_counter = 0
        timer = 1
        color = " "
        start_time = time.time()
        while not rospy.is_shutdown():
            runtime = (time.time() - start_time)
            if self.is_key_available() or (runtime >= 120):
                char = self.get_key()
                if char.lower() == 'q' or (runtime >= 120):
                    GPIO.output(GPIObus,GPIO.LOW)
                    print("Average speed: \n")
                    print(averageLinSpeed)
                    print("Collisions: \n")
                    print(Collision_counter)
                    print("victims: \n")
                    print(Victim_counter)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    return

            lidar_distances = self.get_scan()
            counter = 0 
            for i in range(0,30):
                if(lidar_distances[i] != 0):
                    temp_sum += lidar_distances[i]
                    counter += 1

                
            if(counter != 0):
                temp_sum /= counter
            else:
                temp_sum = 3.5
            RawDistance[0] = temp_sum
            # to binary value
            if(temp_sum < STOP_DISTANCE):
                temp_sum = 1
            else:
                temp_sum = 0

            dir[0] = temp_sum
            temp_sum = 0 

            counter = 0
            for i in range(30,60):
                if(lidar_distances[i] != 0):
                    temp_sum += lidar_distances[i]
                    counter += 1
           

            if(counter != 0):
                temp_sum /= counter
            else:
                temp_sum = 3.5
            RawDistance[1] = temp_sum

            #to binary data
            if(temp_sum < STOP_DISTANCE):
                temp_sum = 1
            else:
                temp_sum = 0
            dir[1] = temp_sum
            temp_sum = 0 

            counter = 0
            for i in range(60,90):
                if(lidar_distances[i] != 0):
                    temp_sum += lidar_distances[i]
                    counter += 1
          
            if(counter != 0):
                temp_sum /= counter
            else:
                temp_sum = 3.5
            RawDistance[2] = temp_sum

            #to binary data
            if(temp_sum < STOP_DISTANCE):
                temp_sum = 1
            else:
                temp_sum = 0
            dir[2] = temp_sum
            temp_sum = 0 

            

            if (not dir[0] and not dir[1] and not dir[2] ):
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(not dir[0] and not dir[1] and dir[2] ):
                # print("left")
                twist.linear.x = LINEAR_VEL * ProcentLinear(RawDistance[2])
                twist.angular.z = -ANGULAR_VEL * ProcentAngular(RawDistance[2])
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(not dir[0] and dir[1] and not dir[2] ):
                # print("middle")
                twist.linear.x = LINEAR_VEL * ProcentLinear(RawDistance[1])
                twist.angular.z = ANGULAR_VEL * ProcentAngular(RawDistance[1])
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(not dir[0] and dir[1] and dir[2] ):
                # print("middle left")
                twist.linear.x = LINEAR_VEL * (2*ProcentLinear(RawDistance[1]) + ProcentLinear(RawDistance[2]))/3
                twist.angular.z = -ANGULAR_VEL * (2*ProcentAngular(RawDistance[1])+ ProcentAngular(RawDistance[2]))/3
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif( dir[0] and not dir[1] and not dir[2] ):
                # print("right")
                twist.linear.x = LINEAR_VEL * ProcentLinear(RawDistance[0])
                twist.angular.z = ANGULAR_VEL * ProcentAngular(RawDistance[0])
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(dir[0] and not dir[1] and dir[2] ):
                # print("right left")
                twist.linear.x = LINEAR_VEL
                twist.angular.z = -ANGULAR_VEL * ProcentAngular(RawDistance[0])
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(dir[0] and dir[1] and not dir[2] ):
                # print("right middle")
                twist.linear.x = LINEAR_VEL * (2*ProcentLinear(RawDistance[1]) + ProcentLinear(RawDistance[0]))/3
                twist.angular.z = ANGULAR_VEL * (2*ProcentAngular(RawDistance[1])+ ProcentAngular(RawDistance[0]))/3
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            elif(dir[0] and dir[1] and dir[2] ):
                # print("all")
                twist.linear.x = LINEAR_VEL * ProcentLinear(RawDistance[1])
                twist.angular.z = ANGULAR_VEL * ProcentAngular(RawDistance[1])
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                #rospy.loginfo('Distance of the obstacle : %f', min_distance)
            
            # print(RawDistance)


            Speed_Accumilation += twist.linear.x
            averageLinSpeed = Speed_Accumilation/iteration
            iteration += 1

            data = bus.read_i2c_block_data(0x44, 0x09, 6)
            green = data[1] + data[0]/256
            red = data[3] + data[2]/256
            blue = data[5] + data[4]/256
            blue *= 3
            green /= 1.5
            my_colors = [red,green,blue]
            if green >= red and green >= blue:
                color = "Green"
                GPIO.output(GPIObus,GPIO.LOW)
                timer = 1
                first_hit = True
        
            elif blue >= red and blue >= green:
                color = "Blue"
                GPIO.output(GPIObus,GPIO.LOW)
                timer = 17
                first_hit = True
            else:
                if(first_hit):
                    Victim_counter += 1
                    first_hit = False
                    print("VICTIM")
                if timer >= 1:
                    GPIO.output(GPIObus,GPIO.HIGH)
                else:
                    GPIO.output(GPIObus,GPIO.LOW)
                
                timer += 1
                if timer > 1:
                    timer = 0
                color = "Red"
            

            for i in range(0,3):
                if(RawDistance[i] < Collision_radius):
                    GPIO.output(GPIObus,GPIO.HIGH)
                    if not colliding:
                        Collision_counter += 1
                        colliding = True
                    print("COLLISION")
                else:
                    colliding = False


        
                        
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
