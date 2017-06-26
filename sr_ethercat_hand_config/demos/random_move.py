#!/env/bin python

import rospy
from std_msgs.msg import Float64
from math import pi
import random

rospy.init_node("random_moves")

MIN_SLEEP = 0.1
MAX_SLEEP = 2.0

joints = {"lfj0": [0.0, pi], "lfj3": [0.0, pi/2.]}
pubs = {}

for joint in joints:
    pubs[joint] = rospy.Publisher("/sh_rh_"+joint+"_position_controller/command", Float64)

while not rospy.is_shutdown():
    for joint in joints:
        target = random.uniform(joints[joint][0], joints[joint][1])
        pubs[joint].publish(target)

    rospy.sleep(random.uniform(MIN_SLEEP, MAX_SLEEP))
