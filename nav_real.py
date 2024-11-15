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
FORWARD_SPEED = 0.3

class NavReal:
    def __init__(self):
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #Transform buffer to receives TFs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.cur_dist = None
        #self.cur_pin_dist = None
        #self.cur_pin_yaw = None
        self.cur_yaw = None
       
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        self.cur_dist = msg.x
        self.cur_yaw = msg.y

    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        if target_yaw < 0.05:
            target_yaw = 2*math.pi #helps it slow down at the appropriate time.
        rate = rospy.Rate(20)
        while self.cur_yaw == None:
            continue #waiting for new odometry values
            rate.sleep()
        
        target_yaw = self.convertNegativeAngles(target_yaw)
        headingTwist = Twist()
        proportion = 0.5

        while not math.isclose(self.cur_yaw, target_yaw, abs_tol=TURN_ANGLE_TOLERANCE):
            rospy.loginfo("current yaw:" + str(self.cur_yaw))
            rospy.loginfo("target yaw: " + str(target_yaw))
            #slows rotation as the target is approached, with a minimum and maximum rotation speed defined.
            headingTwist.angular.z = max(min(abs(proportion*(target_yaw - self.cur_yaw)) , base_vel) , MIN_TURN_SPEED)
            self.cmd_vel_pub.publish(headingTwist)
            while self.cur_yaw == None:
                continue #waiting for new odometry values
            rate.sleep()
       
        self.cmd_vel_pub.publish(Twist()) #a 0-rotation twist to stop movement

        
    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """
        rospy.loginfo("Scanning for fiducials")
        #1.98*2pi is used instead of 2pi because sometimes the odom error makes the initial yaw reading close to 2pi instead of 0.
        self.turn_to_heading(self.headingToRelative(1.98*math.pi), 0.25) #a slow rotation is used so that the fiducials can be clearly read.
        

    def match_pin_rotation(self, pin_id):
        """
        Rotates the robot so that its `base_link` frame's orientation matches
        that of the target pin's frame.
        """
        rospy.loginfo("Matching pin rotation")
        #yaw of the pin relative to base_link
        pin_yaw = self.get_pin_orientation(pin_id)
        target_yaw = self.headingToRelative(self.convertNegativeAngles(pin_yaw))

        self.turn_to_heading(target_yaw, 1)
        
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        rospy.loginfo("Facing pin")
        #x and y values of pin coordinates relative to base_link
        (x, y) = self.get_pin_coords(pin_id)

        #finding the arctan of the pin x and y distance from base_link tells you how much to turn.
        target_heading = self.headingToRelative(self.convertNegativeAngles(math.atan2(y, x)))
        self.turn_to_heading(target_heading, 1)

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        #since my_odom published the distance traveled since the previous odom reading, this method calculates
        # the total distance moved by summing the my_odom values.
        rospy.loginfo("Moving toward pin")
        target_dist = self.get_dist_to_pin(pin_id) - 0.25 # -0.25 to avoid crashing
        dist_moved = 0  
        forwardTwist = Twist()
        forwardTwist.linear.x = FORWARD_SPEED
        while dist_moved < target_dist:
            rospy.loginfo('Target distance: ' + str(target_dist))
            rospy.loginfo('Distance moved: ' + str(dist_moved))
            self.cmd_vel_pub.publish(forwardTwist)
            while self.cur_dist == None:
                continue #waiting for new odometry values
            if (self.cur_dist > TOLERANCE_AGAINST_ERROR): #making sure we are actually capturing movement, not measuring error
                dist_moved += self.cur_dist #updating based on last odom_distance reading
            self.cur_dist = None 
            
        self.cmd_vel_pub.publish(Twist()) #a 0-velocity twist to stop movement

    def get_dist_to_pin(self, pin_id):
        '''Returns distance from base_link to the pin'''
        found_transform = False
        #waiting for TF buffer to populate
        while not found_transform:
            try:    
                #looking up the transform and translation between base_link and pin
                A_to_B_transl = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.translation
                found_transform = True
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue

        #calculating distance between base_link (origin) and the pin.
        dist = math.sqrt(A_to_B_transl.x**2 + A_to_B_transl.y**2)
        return dist

    def get_pin_coords(self, pin_id):
        '''Returns the (x, y) distances of a pin relative to base_link (origin)'''
        found_transform = False
        
        #waiting for TF buffer to populate
        while not found_transform:
            try:
                #Looking up the transform and translation between base_link and pin
                A_to_B_transl = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.translation
                found_transform = True
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue
        return [A_to_B_transl.x, A_to_B_transl.y]

    def get_pin_orientation(self, pin_id):
        '''Returns the yaw of the pin relative to base_link'''
        found_transform = False
        
        #waiting for TF buffer to populate
        while not found_transform:
            try:
                #Looking up transform and quaternion rotation of the pin relative to base_link.
                pin_rotation = self.tf_buffer.lookup_transform('base_link',f'pin_{pin_id}',rospy.Time()).transform.rotation
                orientations = [pin_rotation.x, pin_rotation.y, pin_rotation.z, pin_rotation.w]
                #Converting from quaternion to euler
                (roll, pitch, yaw) = euler_from_quaternion(orientations)
                #self.cur_pin_yaw = yaw
                found_transform = True
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ): continue
        return yaw


    def convertNegativeAngles(self, angle):
        '''Converts from [-pi,pi] to [0,2pi]'''
        if angle >= 0:
            return angle
        else:
            return 2*math.pi + angle 
    
    #makes sure there are no angles over 2pi
    def normalizePosAngles(self, heading):
        return abs(heading) % (2*math.pi)

    #ensures yaw targets are relative to the robot, allowing it to move in the desired direction from any initial orientation.
    def headingToRelative(self, target_yaw):
        while self.cur_yaw == None:
            continue #waiting for new odometry values
        return self.normalizePosAngles(self.cur_yaw + target_yaw)
        
if __name__ == '__main__':
    rospy.init_node('nav_real')

    nav = NavReal()
    nav.scan_for_fids()
    target_pin_ids = [100, 102, 104, 108]
    for pin_id in target_pin_ids:
        nav.match_pin_rotation(pin_id)
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)
