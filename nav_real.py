#!/usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

TOLERANCE_AGAINST_ERROR = 0.0001
TURN_ANGLE_TOLERANCE = 0.01
MIN_TURN_SPEED = 0.03
FORWARD_SPEED = 0.1

class NavReal:
    def __init__(self):
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cur_dist = None
        self.cur_pin_dist = None
        self.cur_pin_yaw = None
        self.cur_yaw = None
       
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        self.cur_dist = msg.x
        self.cur_yaw = msg.y
        #print(msg.y)
        #self.cur_yaw = self.convertNegativeAngles(msg.y) #Point is (Change in distance, yaw, 0)
        #raise NotImplementedError


    #CHANGE SO THAT IT CAN SLOW DOWN FOR TARGET YAW OF 0.0
    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        if target_yaw < 0.05:
            target_yaw = 2*math.pi #helps it slow down at the appropriate time.
        rate = rospy.Rate(10)
        print(self.cur_yaw)
        #self.cur_yaw = 0 #testing to see if we need to reset
        while self.cur_yaw == None:
            continue #waiting for new odometry values
            rate.sleep()
        
        target_yaw = self.convertNegativeAngles(target_yaw)
        headingTwist = Twist()
        proportion = 0.5

        while not math.isclose(self.cur_yaw, target_yaw, abs_tol=TURN_ANGLE_TOLERANCE): #since going past pi rads turns heading value to negative
            #slows rotation as the target is approached, with a minimum and maximum rotation speed defined.
            print("curr:" ,self.cur_yaw)
            print("target: ", target_yaw)
            headingTwist.angular.z = max(min(abs(proportion*(target_yaw - self.cur_yaw)) , base_vel) , MIN_TURN_SPEED)
            # if self.cur_yaw > 6.1 or self.cur_yaw < 0.1:
            #     headingTwist.angular.z = 0.05
            self.cmd_vel_pub.publish(headingTwist)
            #print(headingTwist)
            while self.cur_yaw == None:
                continue #waiting for new odometry values
            rate.sleep()
       
        self.cmd_vel_pub.publish(Twist()) #a 0-rotation twist to stop movement
    
        #raise NotImplementedError
        
    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """
        self.turn_to_heading(self.headingToRelative(1.98*math.pi), 0.25)
        
        #raise NotImplementedError

    def match_pin_rotation(self, pin_id):
        """
        Rotates the robot so that its `base_link` frame's orientation matches
        that of the target pin's frame.
        """
        pin_yaw = self.get_pin_orientation(pin_id)
        for i in range(100):
            print(pin_yaw)
        target_yaw = self.headingToRelative(self.convertNegativeAngles(pin_yaw))

        self.turn_to_heading(target_yaw, 0.5)
        #raise NotImplementedError
        
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        (x, y) = self.get_pin_coords(pin_id)
        for i in range(100):
            print("x: ", x, ", y: ", y)
        target_heading = self.headingToRelative(self.convertNegativeAngles(math.atan2(y, x)))
        print("pin heading: ", target_heading)
        self.turn_to_heading(target_heading, 0.3)
        #raise NotImplementedError

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        #since my_odom published the distance traveled since the previous odom reading, this method calculates
        # the total distance moved by summing the my_odom values.
        target_dist = max(0, self.get_dist_to_pin(pin_id) - 0.25) #to avoid crashing
        dist_moved = 0  
        forwardTwist = Twist()
        forwardTwist.linear.x = FORWARD_SPEED
        while dist_moved < target_dist:
            print('target: ', target_dist)
            print('moved: ', dist_moved)
            print(self.cur_dist)
            self.cmd_vel_pub.publish(forwardTwist)
            while self.cur_dist == None:
                continue #waiting for new odometry values
            if (self.cur_dist > TOLERANCE_AGAINST_ERROR): #making sure we are actually capturing movement, not measuring error
                dist_moved += self.cur_dist #updating based on last odom_distance reading
            self.cur_dist = None 
            
        self.cmd_vel_pub.publish(Twist()) #a 0-velocity twist to stop movement
        #raise NotImplementedError

    def get_dist_to_pin(self, pin_id):
        found_transform = False
        
        while not found_transform:
            try:
                A_to_B_transl = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.translation
                #pin_yaw = A_tf.rotation.z #does this work?
                #self.cur_pin_yaw = pin_yaw
                #print("pin yaw:" ,pin_yaw)
                found_transform = True
            # C_tfs.transform.rotation = A_tf.rotation
            # C_tfs.transform.translation = A_to_B_tf.translation
            # C_tfs.header.stamp = rospy.Time.now()
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue

        print('hiiiiii', A_to_B_transl)

        dist = math.sqrt(A_to_B_transl.x**2 + A_to_B_transl.y**2) #np.linalg.norm(np.array([A_to_B_transl.x, A_to_B_transl.y, A_to_B_transl.z,])) #use something less complex
        self.cur_pin_dist = dist
        return dist

    def get_pin_coords(self, pin_id):
        found_transform = False
        
        while not found_transform:
            try:
                A_to_B_transl = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.translation
                #pin_yaw = A_tf.rotation.z #does this work?
                #self.cur_pin_yaw = pin_yaw
                #print("pin yaw:" ,pin_yaw)
                found_transform = True
            # C_tfs.transform.rotation = A_tf.rotation
            # C_tfs.transform.translation = A_to_B_tf.translation
            # C_tfs.header.stamp = rospy.Time.now()
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue
            return [A_to_B_transl.x, A_to_B_transl.y]

    def get_pin_orientation(self, pin_id):
        #self.tf_listener.waitForTransform('/odom',f'fiducial_{pin_id}',rospy.Time(), rospy.Duration(4.0))
        pin = 'pin_'+str(pin_id)
        print(self.tf_buffer)
        found_transform = False
        
        while not found_transform:
            try:
                pin_rotation = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.rotation
                # A_tf = self.tf_buffer.lookup_transform('base_link', pin, rospy.Time()).transform
                orientations = [pin_rotation.x, pin_rotation.y, pin_rotation.z, pin_rotation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientations)
                self.cur_pin_yaw = yaw
                print("pin yaw:" ,yaw)
                found_transform = True
            # C_tfs.transform.rotation = A_tf.rotation
            # C_tfs.transform.translation = A_to_B_tf.translation
            # C_tfs.header.stamp = rospy.Time.now()
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue
        return yaw


    def convertNegativeAngles(self, angle):
        if angle >= 0:
            return angle
        else:
            return 2*math.pi + angle 
    
    #makes sure there are no angles over 2pi
    def normalizePosAngles(self, heading):
        return abs(heading) % (2*math.pi)

    #ensures yaw targets are relative to the robot, allowing it to move in the desired shapes from any initial orientation.
    def headingToRelative(self, target_yaw):
        while self.cur_yaw == None:
            continue #waiting for new odometry values
        return self.normalizePosAngles(self.cur_yaw + target_yaw)
        
if __name__ == '__main__':
    rospy.init_node('nav_real')

    nav = NavReal()
    nav.scan_for_fids()
    target_pin_ids = [100, 102, 104, 108]#[0, 1, 2, 3]
    for pin_id in target_pin_ids:
        nav.match_pin_rotation(pin_id)
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)
