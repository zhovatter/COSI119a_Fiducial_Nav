# COSI119a_Fiducial_Nav

This package allows a robot to identify specified fiducials in the area and move to each fiducial one at a time.

Aruco Detect is used to identify the fiducials and publish their TFs to the TF tree. 

mapper_real.py is then used to map each fiducial with a corresponding fiducial id to a pin with a transform relative to odom.

nav_real.py first spins around to detect all relevant fiducials (using mapper to acquire the pin TFs), then uses TF lookups to first match the pins rotation, then to face the pin using trigonometry since the pin yaws all match odom, and finally moves the robot toward each pin and stops just in front of it. This process is repeated until all fiducials/pins have been visited. All movements are monitored using the prebuilt my_odom node, which mainly receives data from odom and reformates it for ease of use.

#How to run:

After bringing up the robot:
1. Edit the fiducial_len and dictionary parameters in fiducials_real.launch to match whatever type of fiducials you are using
2. Edit the fid_ids and target_pin_ids lists in mapper_real.py and nav_real.py, respectively to match the ids of the fiducials that you want to detect
3. roslaunch fiducial_nav fiducials_real.launch
4. rosrun fiducial_nav nav_real.py
  The first roslaunch (2) runs Aruco Detect, mapper_real.py, and my_odom.
  nav_real.py was written by me and contains the logic for moving the robot to each pin.
