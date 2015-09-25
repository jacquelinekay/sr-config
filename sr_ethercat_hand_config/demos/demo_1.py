#!/usr/bin/env python

# Script to move the right hand into store position.

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("store_right_hand", anonymous=True)

hand_commander = SrHandCommander(name="rh_shadow_hand", prefix="rh")

open_hand = {'rh_FFJ1': -11.39817456222876, 'rh_FFJ2': 55.463852605884334, 'rh_FFJ3': 6.91927389993643,
             'rh_FFJ4': -7.45314764163997, 'rh_THJ4': 31.818444559149807, 'rh_THJ5': -45.359023760596386,
             'rh_THJ1': 77.84292622858992, 'rh_THJ2': -14.538342666116007, 'rh_RFJ4': -5.604412024059816,
             'rh_RFJ1': 8.91519787974375, 'rh_RFJ2': 93.33151179643137, 'rh_RFJ3': 0.5121250302352169}

pack_hand = {'rh_FFJ1': -12.251714667233164, 'rh_FFJ2': 87.36768802228413, 'rh_FFJ3': 61.3626336439792,
             'rh_FFJ4': -5.609199617655059, 'rh_THJ4': 32.998596632346086, 'rh_THJ5': -43.762914027470806,
             'rh_THJ1': 89.6830102685592, 'rh_THJ2': -12.846461254489313, 'rh_RFJ4': -5.484564447029029,
             'rh_RFJ1': 7.9274251243862, 'rh_RFJ2': 96.75283328185512, 'rh_RFJ3': 83.41044781836959}



# Move hand to open position
joint_states = open_hand
rospy.loginfo("Moving hand to open position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True, angle_degrees=True)
rospy.sleep(2)

# Move hand to closed position
joint_states = pack_hand
rospy.loginfo("Moving hand to pack position")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True, angle_degrees=True)
rospy.sleep(2)
