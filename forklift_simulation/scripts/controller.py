#!/usr/bin/env python3

import numpy as np
from numpy import array, pi
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import  Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8

from tf.transformations import euler_from_quaternion

from time import sleep

#####################################################################
####          Skeleton code for controller node                  ####
####    Feel free to change/add functions as you wish            ####
####    You can also change the arguments of the functions       ####
####    Note: Don't alter the node name                          ####
#####################################################################

class Controller():
    def __init__(self):

        self.setpointVelocity = 1.0
        self.currentVelocity = 0.0
        self.lookaheadDistance = 1.5
        self.brakeMargin = 0.05

        self.L = 0.5
        self.angleLimit = 90*pi/180

        self.action = 0
        self.steering_angle = 0

        self.pose= [0,0,0]

        # PID VARIABLES
        self.kp_drive, self.ki_drive, self.kd_drive = [1, 0.5, 0]
        self.kp_steer, self.ki_steer, self.kd_steer = [5, 2, 0]
        self.error_integral_steer = 0
        self.error_integral_drive = 0
          
        self.error = 0.0
        self.error_dot = 0.0
        self.error_integral = 0.0
        self.error_previous = 0.0
        
        # PATH VARIABLES
        self.path = []
        self.lastPath = []
        self.targetIndex = 0
        self.pathReceived = False
        self.map_frame = "odom"

        self.stop = False
        self.brake = False

        self.check = True

        # INIT Nodes
        rospy.init_node('controller2')
        print("controller ON")

        # INIT PUBLISHERS
        self.nextPointPub =  rospy.Publisher('/target_point',Marker, queue_size=1)
        self.updateControl = rospy.Publisher('/external_control', Twist, queue_size=10)
    
        # INIT SUBSCRIBERS
        rospy.Subscriber('/encoder', Odometry, self.velocityCallback)
        rospy.Subscriber('/robot_pose_ekf/odom_combined' ,PoseWithCovarianceStamped, self.positionCallback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.pathCallback)
        rospy.Subscriber('/Estop',Int8,  self.EstopCallback)

        rate = rospy.get_param("/simulation/controller_rate", 50) #get rate else set to default
        self.dt = 1/rate 
        rate = rospy.Rate(rate)


        while not rospy.is_shutdown():
            self.executer()
            rate.sleep()

    def velocityCallback(self, state:Odometry):
        ### Call back function for state message ###
        self.currentVelocity = state.twist.twist.linear.x

    def positionCallback(self, data):
        orientations = data.pose.pose.orientation
        orientations = [orientations.x ,orientations.y, orientations.z, orientations.w] 
        orientations = list(euler_from_quaternion(orientations))

        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, orientations[2]]

    def EstopCallback(self,msg):
        if msg.data == 1 :
            self.stop = True

        elif msg.data == 0:
            self.stop = False

    def pathCallback(self, mypath):
        ### Call back function for path message ###

        newPath = []
        for pose in mypath.poses:
            newPath.append([pose.pose.position.x, pose.pose.position.y])

        if newPath != self.path:
            self.pathReceived = False
            self.path.clear()
            self.path = newPath
            # self.stop = True
            self.targetIndex = 0
            # self.stop=False
            self.pathReceived = True

        elif newPath == None:
            self.pathReceived = False

    def executer(self):
        myx = self.pose[0]
        myy = self.pose[1]
        theta = self.pose[2]
        try:
            targetPoint = self.path[self.targetIndex]

            target_x = targetPoint[0]
            target_y = targetPoint[1]

            # direc_vector = np.arctan2((target_y - myy), (target_x - myx))
            # print(f"{direc_vector*180/pi:.2f} {theta*180/pi:.2f} {(direc_vector-theta)*180/pi:.2f}")

        except:
            pass


        if self.pathReceived == True:
            update = Twist()

            self.action = self.speed_control()
            self.steering_angle = self.purePursuit()
            update.linear.x = self.action
            update.angular.z = self.steering_angle

            self.updateControl.publish(update)
            self.nextPoint()

    def nextPoint(self): #next point marker

        nextPoint = Marker()
        target_waypoint = self.path[self.targetIndex]
        tx = target_waypoint[0]
        ty = target_waypoint[1]
        nextPoint.header.frame_id = self.map_frame
        nextPoint.header.stamp = rospy.Time.now()
        nextPoint.ns = "point"
        nextPoint.id = 3
        nextPoint.type = Marker.SPHERE
        nextPoint.action = Marker.ADD
        nextPoint.pose.position.x = tx
        nextPoint.pose.position.y = ty   
        nextPoint.pose.position.z = 0.1
        nextPoint.pose.orientation.x = 0.0
        nextPoint.pose.orientation.y = 0.0
        nextPoint.pose.orientation.z = 0.0
        nextPoint.pose.orientation.w = 1.0
        nextPoint.scale.x = 1.0
        nextPoint.scale.y = 1.0
        nextPoint.scale.z = 0.1
        nextPoint.color.a = 1.0
        nextPoint.color.r = 1.0
        nextPoint.color.g = 0.0
        nextPoint.color.b = 0.0
        self.nextPointPub.publish(nextPoint)
      
    def speed_control(self):
        
        ###            Implement PID controller here                   ###
        #          Take as an input the current state of the car         #
        #                   Return acceleration                          #

        if self.stop :
            return -2.0
        elif self.brake:
            # print("arrived")
            return 0.0
        
        action = self.PID("driving", self.kp_drive, self.ki_drive, self.kd_drive, self.setpointVelocity, self.currentVelocity, clip = 1)

        return action
     
    def purePursuit(self):
        
        ###              Implement pure pursuit controller here               ###
        #            Take as an input the current state of the car,             #
        #                     waypoints & targetIndex                           #
        #                      Return steering angle                            #
        if self.stop or self.brake:
            return 0.0
        
        self.targetIndex = self.searchTargetPoint()
        if self.targetIndex == None:
            return 0
        
        targetPoint = self.path[self.targetIndex]


        myx = self.pose[0]
        myy = self.pose[1]
        theta = self.pose[2]
        target_x = targetPoint[0]
        target_y = targetPoint[1]


        alpha = np.arctan2(target_y - myy, target_x - myx) - theta
        steering_angle = np.arctan2(2*self.L*np.sin(alpha) , self.lookaheadDistance)
        steering_angle = np.clip(steering_angle, -self.angleLimit, self.angleLimit)

        # print(np.arctan2((target_y - myy), (target_x - myx))*180/pi)
        direc_vector = np.arctan2((target_y - myy), (target_x - myx))
        print(f"{direc_vector*180/pi:.2f} {theta*180/pi:.2f} {(direc_vector-theta)*180/pi:.2f}")


        print(f'xcurrent {myx:.2f} ycurrent {myy:.2f} targetx {target_x:.2f} targety {target_y:.2f} steering {steering_angle*180/pi:.2f} speed {self.action:.2f}')
        return -steering_angle
    
    def PID(self,selector, kp, ki, kd, setpoint, feedback, **kwargs):
        error = setpoint - feedback

        error_dot = (error - self.error_previous)/self.dt

        self.error_previous = error

        if selector == "steering":
            self.error_integral_steer += error*self.dt
            action = kp*error + ki*self.error_integral_steer + kd*error_dot

        elif selector == "driving":
            self.error_integral_drive += error*self.dt
            action = kp*error + ki*self.error_integral_drive + kd*error_dot
    
        try:
            if kwargs['clip']:
                action = np.clip(action, -kwargs["clip"], kwargs["clip"])
        except:
            pass

        if abs(error) > (2*pi/180) or selector == "driving" :
            return action
        else:
            self.error_integral_steer = 0
            self.error_integral_drive = 0
            return 0  
    
    def searchTargetPoint(self):
        ###           Search for target point in the given path               ###
        #       Take as an input the current state of the car & waypoints       #
        #                   Return index of target point                        #

        distances = np.linalg.norm(array(self.path[self.targetIndex:]) - array(self.pose[0:2]).transpose(), axis=1)
        min_index = np.argmin(distances)

        if distances[-1] < self.brakeMargin:
            self.brake = True
        else:
            self.brake = False

        pathPoints = len(distances)
        for i in range(min_index, pathPoints):
            if distances[i] >= self.lookaheadDistance:
                return i + self.targetIndex
            elif distances[-1] < self.lookaheadDistance:
                return pathPoints + self.targetIndex - 1
        return min_index + self.targetIndex
    

if __name__ == '__main__':
    # sleep(7)
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass



