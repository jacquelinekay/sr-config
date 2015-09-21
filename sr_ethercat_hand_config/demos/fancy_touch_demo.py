#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#


import rospy
import random
import time
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node('receiver_example')
hand_commander = SrHandCommander()

#
# RANGES #
#

# Minimum allowed range for the joints in this particular script
min_range = {"rh_THJ2": -40, "rh_THJ3": -12, "rh_THJ4": 0, "rh_THJ5": -55,
             "rh_FFJ1": 10, "rh_FFJ2": 10, "rh_FFJ3": 0, "rh_FFJ4": -20,
             "rh_MFJ1": 10, "rh_MFJ2": 10, "rh_MFJ3": 0, "rh_MFJ4": -10,
             "rh_RFJ1": 10, "rh_RFJ2": 10, "rh_RFJ3": 0, "rh_RFJ4": -10,
             "rh_LFJ1": 10, "rh_LFJ2": 10, "rh_LFJ3": 0, "rh_LFJ4": -20, "rh_LFJ5": 0,
             "rh_WRJ1": -20, "rh_WRJ2": -10}

# Maximum allowed range for the joints in this particular script
max_range = {"rh_THJ2": 20, "rh_THJ3": 12, "rh_THJ4": 70, "rh_THJ5": 0,
             "rh_FFJ1": 55, "rh_FFJ2": 55, "rh_FFJ3": 90, "rh_FFJ4": 0,
             "rh_MFJ1": 55, "rh_MFJ2": 55, "rh_MFJ3": 90, "rh_MFJ4": 0,
             "rh_RFJ1": 55, "rh_RFJ2": 55, "rh_RFJ3": 90, "rh_RFJ4": 0,
             "rh_LFJ1": 55, "rh_LFJ2": 55, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 1,
             "rh_WRJ1": 10, "rh_WRJ2": 5}


#
# POSE DEFINITIONS #
#
# starting position for the hand
start_pos = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0,
             "rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0,
             "rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0,
             "rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0,
             "rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0,
             "rh_WRJ1": 0, "rh_WRJ2": 0}
# Start position for the Hand
pregrasp_pos = {"rh_THJ2": 12, "rh_THJ3": 15, "rh_THJ4": 69, "rh_THJ5": -23,
                "rh_FFJ1": 20, "rh_FFJ2": 20, "rh_FFJ3": 21, "rh_FFJ4": -15,
                "rh_MFJ1": 20, "rh_MFJ2": 20, "rh_MFJ3": 21, "rh_MFJ4": 0,
                "rh_RFJ1": 20, "rh_RFJ2": 20, "rh_RFJ3": 21, "rh_RFJ4": -7,
                "rh_LFJ1": 20, "rh_LFJ2": 20, "rh_LFJ3": 21, "rh_LFJ4": -10, "rh_LFJ5": 0,
                "rh_WRJ1": 0, "rh_WRJ2": 0}
# Close position for the Hand
grasp_pos = {"rh_THJ2": 30, "rh_THJ3": 15, "rh_THJ4": 69, "rh_THJ5": -3,
             "rh_FFJ1": 39, "rh_FFJ2": 39, "rh_FFJ3": 67, "rh_FFJ4": -19,
             "rh_MFJ1": 41, "rh_MFJ2": 41, "rh_MFJ3": 62, "rh_MFJ4": 0,
             "rh_RFJ1": 45, "rh_RFJ2": 45, "rh_RFJ3": 64, "rh_RFJ4": -18,
             "rh_LFJ1": 49, "rh_LFJ2": 49, "rh_LFJ3": 64, "rh_LFJ4": -19, "rh_LFJ5": 0,
             "rh_WRJ1": 0, "rh_WRJ2": 0}
# Random position for the Hand (initialied at 0)
rand_pos = {"rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0,
            "rh_FFJ1": 0, "rh_FFJ2": 0, "rh_FFJ3": 0, "rh_FFJ4": 0,
            "rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0,
            "rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0,
            "rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0, "rh_LFJ5": 0,
            "rh_WRJ1": 0, "rh_WRJ2": 0}
# flex first finger
flex_ff = {"rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0}
# extend first finger
ext_ff = {"rh_FFJ2": 0, "rh_FFJ1": 0, "rh_FFJ3": 0, "rh_FFJ4": 0}
# flex middle finger
flex_mf = {"rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0}
# extend middle finger
ext_mf = {"rh_MFJ1": 0, "rh_MFJ2": 0, "rh_MFJ3": 0, "rh_MFJ4": 0}
# flex ring finger
flex_rf = {"rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0}
# extend ring finger
ext_rf = {"rh_RFJ1": 0, "rh_RFJ2": 0, "rh_RFJ3": 0, "rh_RFJ4": 0}
# flex little finger
flex_lf = {"rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0}
# extend middle finger
ext_lf = {"rh_LFJ1": 0, "rh_LFJ2": 0, "rh_LFJ3": 0, "rh_LFJ4": 0}
# flex thumb step 1
flex_th_1 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 70, "rh_THJ5": 0}
# flex thumb step 2
flex_th_2 = {"rh_THJ1": 20, "rh_THJ2": 40, "rh_THJ3": 10, "rh_THJ4": 70, "rh_THJ5": 58}
# extend thumb step 1
ext_th_1 = {"rh_THJ1": 10, "rh_THJ2": 20, "rh_THJ3": 5, "rh_THJ4": 35, "rh_THJ5": 25}
# extend thumb step 2
ext_th_2 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}
# zero thumb
zero_th = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 0, "rh_THJ5": 0}
# Pre O.K. with first finger
pre_ff_ok = {"rh_THJ4": 70}
# O.K. with first finger
ff_ok = {"rh_THJ1": 15, "rh_THJ2": 20, "rh_THJ3": 0, "rh_THJ4": 56, "rh_THJ5": 11,
         "rh_FFJ1": 37, "rh_FFJ2": 37, "rh_FFJ3": 65, "rh_FFJ4": -0.2,
         "rh_MFJ1": 21, "rh_MFJ2": 21, "rh_MFJ3": 33, "rh_MFJ4": -3,
         "rh_RFJ1": 25, "rh_RFJ2": 25, "rh_RFJ3": 18, "rh_RFJ4": 0.5,
         "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. transition from first finger to middle finger
ff2mf_ok = {"rh_THJ1": 5, "rh_THJ2": 12, "rh_THJ3": -10, "rh_THJ4": 60, "rh_THJ5": 2,
            "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 21, "rh_MFJ2": 21, "rh_MFJ3": 33, "rh_MFJ4": -3,
            "rh_RFJ1": 25, "rh_RFJ2": 25, "rh_RFJ3": 18, "rh_RFJ4": 0.5,
            "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. with middle finger
mf_ok = {"rh_THJ1": 15, "rh_THJ2": 18, "rh_THJ3": 7, "rh_THJ4": 66, "rh_THJ5": 23,
         "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 44, "rh_MFJ2": 44, "rh_MFJ3": 63, "rh_MFJ4": 11,
         "rh_RFJ1": 25, "rh_RFJ2": 25, "rh_RFJ3": 18, "rh_RFJ4": -10,
         "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -6, "rh_LFJ5": 7}
# O.K. transition from middle finger to ring finger
mf2rf_ok = {"rh_THJ1": 5, "rh_THJ2": -5, "rh_THJ3": 15, "rh_THJ4": 70, "rh_THJ5": 19,
            "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 22, "rh_MFJ2": 22, "rh_MFJ3": 4, "rh_MFJ4": -1,
            "rh_RFJ1": 25, "rh_RFJ2": 25, "rh_RFJ3": 18, "rh_RFJ4": -19,
            "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -12, "rh_LFJ5": 7}
# O.K. with ring finger
rf_ok = {"rh_THJ1": 15, "rh_THJ2": 5, "rh_THJ3": 15, "rh_THJ4": 70, "rh_THJ5": 42,
         "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 22, "rh_MFJ2": 22, "rh_MFJ3": 4, "rh_MFJ4": -1,
         "rh_RFJ1": 51, "rh_RFJ2": 51, "rh_RFJ3": 52, "rh_RFJ4": -19,
         "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -12, "rh_LFJ5": 7}
# O.K. transition from ring finger to little finger
rf2lf_ok = {"rh_THJ1": 5, "rh_THJ2": 4.5, "rh_THJ3": 8, "rh_THJ4": 60, "rh_THJ5": 21,
            "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
            "rh_MFJ1": 22, "rh_MFJ2": 22, "rh_MFJ3": 4, "rh_MFJ4": -1,
            "rh_RFJ1": 15, "rh_RFJ2": 15, "rh_RFJ3": 6, "rh_RFJ4": 0.5,
            "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -10, "rh_LFJ5": 7}
# O.K. with little finger
lf_ok = {"rh_THJ1": 30, "rh_THJ2": 8, "rh_THJ3": 10, "rh_THJ4": 69, "rh_THJ5": 26,
         "rh_FFJ1": 7, "rh_FFJ2": 7, "rh_FFJ3": 7, "rh_FFJ4": -0.4,
         "rh_MFJ1": 7, "rh_MFJ2": 7, "rh_MFJ3": 4, "rh_MFJ4": -1,
         "rh_RFJ1": 7, "rh_RFJ2": 7, "rh_RFJ3": 6, "rh_RFJ4": 0.5,
         "rh_LFJ1": 48, "rh_LFJ2": 48, "rh_LFJ3": 19, "rh_LFJ4": -7, "rh_LFJ5": 45}
# zero wrist
zero_wr = {"rh_WRJ1": 0, "rh_WRJ2": 0}
# north wrist
n_wr = {"rh_WRJ1": 15, "rh_WRJ2": 0}
# south wrist
s_wr = {"rh_WRJ1": -20, "rh_WRJ2": 0}
# east wrist
e_wr = {"rh_WRJ1": 0, "rh_WRJ2": 8}
# west wrist
w_wr = {"rh_WRJ1": 0, "rh_WRJ2": -14}
# northeast wrist
ne_wr = {"rh_WRJ1": 15, "rh_WRJ2": 8}
# northwest wrist
nw_wr = {"rh_WRJ1": 15, "rh_WRJ2": -14}
# southweast wrist
sw_wr = {"rh_WRJ1": -20, "rh_WRJ2": -14}
# southeast wrist
se_wr = {"rh_WRJ1": -20, "rh_WRJ2": 8}
# lateral lf ext side
l_ext_lf = {"rh_LFJ4": -15}
# lateral rf ext side
l_ext_rf = {"rh_RFJ4": -15}
# lateral mf ext side
l_ext_mf = {"rh_MFJ4": 15}
# lateral ff ext side
l_ext_ff = {"rh_FFJ4": 15}
# lateral all int side
l_int_all = {"rh_FFJ4": -15, "rh_MFJ4": -15, "rh_RFJ4": 15, "rh_LFJ4": 15}
# lateral all ext side
l_ext_all = {"rh_FFJ4": 15, "rh_MFJ4": 15, "rh_RFJ4": -15, "rh_LFJ4": -15}
# lateral ff int side
l_int_ff = {"rh_FFJ4": -15}
# lateral mf int side
l_int_mf = {"rh_MFJ4": -15}
# lateral rf int side
l_int_rf = {"rh_RFJ4": 15}
# lateral lf int side
l_int_lf = {"rh_LFJ4": 15}
# all zero
l_zero_all = {"rh_FFJ4": 0, "rh_MFJ4": 0, "rh_RFJ4": 0, "rh_LFJ4": 0}
# spock
l_spock = {"rh_FFJ4": -20, "rh_MFJ4": -20, "rh_RFJ4": -20, "rh_LFJ4": -20}
# grasp for shaking hands step 1
shake_grasp_1 = {"rh_THJ1": 0, "rh_THJ2": 6, "rh_THJ3": 10, "rh_THJ4": 37, "rh_THJ5": 9,
                 "rh_FFJ1": 10, "rh_FFJ2": 10, "rh_FFJ3": 26, "rh_FFJ4": 0,
                 "rh_MFJ1": 9, "rh_MFJ2": 9, "rh_MFJ3": 24, "rh_MFJ4": 0,
                 "rh_RFJ1": 15, "rh_RFJ2": 15, "rh_RFJ3": 16, "rh_RFJ4": 0,
                 "rh_LFJ1": 15, "rh_LFJ2": 15, "rh_LFJ3": 0, "rh_LFJ4": -10, "rh_LFJ5": 10}
# grasp for shaking hands step 2
shake_grasp_2 = {"rh_THJ1": 21, "rh_THJ2": 21, "rh_THJ3": 10, "rh_THJ4": 42, "rh_THJ5": 21,
                 "rh_FFJ1": 37, "rh_FFJ2": 37, "rh_FFJ3": 29, "rh_FFJ4": 0,
                 "rh_MFJ1": 37, "rh_MFJ2": 37, "rh_MFJ3": 41, "rh_MFJ4": 0,
                 "rh_RFJ1": 37, "rh_RFJ2": 37, "rh_RFJ3": 41, "rh_RFJ4": 0,
                 "rh_LFJ1": 50, "rh_LFJ2": 50, "rh_LFJ3": 41, "rh_LFJ4": 0, "rh_LFJ5": 0}
# store step 1 PST
store_1_PST = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 60, "rh_THJ5": 0,
               "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
               "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
               "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
               "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
               "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 2 PST
store_2_PST = {"rh_THJ1": 50, "rh_THJ2": 12, "rh_THJ3": 0, "rh_THJ4": 60, "rh_THJ5": 27,
               "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
               "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
               "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
               "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
               "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 1 Bio_Tac
store_1_BioTac = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 30, "rh_THJ5": 0,
                  "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
                  "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
                  "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
                  "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
                  "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 2 Bio_Tac
store_2_BioTac = {"rh_THJ1": 20, "rh_THJ2": 36, "rh_THJ3": 0, "rh_THJ4": 30, "rh_THJ5": -3,
                  "rh_FFJ1": 90, "rh_FFJ2": 90, "rh_FFJ3": 90, "rh_FFJ4": 0,
                  "rh_MFJ1": 90, "rh_MFJ2": 90, "rh_MFJ3": 90, "rh_MFJ4": 0,
                  "rh_RFJ1": 90, "rh_RFJ2": 90, "rh_RFJ3": 90, "rh_RFJ4": 0,
                  "rh_LFJ1": 90, "rh_LFJ2": 90, "rh_LFJ3": 90, "rh_LFJ4": 0, "rh_LFJ5": 0,
                  "rh_WRJ1": 0, "rh_WRJ2": 0}
# store step 3
store_3 = {"rh_THJ1": 0, "rh_THJ2": 0, "rh_THJ3": 0, "rh_THJ4": 65, "rh_THJ5": 0}


#
# FUNCTION DEFINITIONS #
#

def sequence_ff():
    # Start sequence 1
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(store_3, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, angle_degrees=True)
    rospy.sleep(1.0)
    hand_commander.move_to_joint_value_target_unsafe(flex_th_1, angle_degrees=True)
    rospy.sleep(0.7)
    tmp = flex_th_2.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=2.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['rh_TH'] > force_zero['rh_TH']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'Thumb contact'
            break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ext_th_2, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_lf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_rf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_mf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_ff, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_all, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_ext_all, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_ff, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_mf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_rf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_int_lf, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_spock, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(l_zero_all, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, angle_degrees=True)
    rospy.sleep(1)
    tmp = ff_ok.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=2.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['rh_TH'] > force_zero['rh_TH'] or tactile_values['rh_FF'] > force_zero['rh_FF']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'First finger contact'
            break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(ff2mf_ok, angle_degrees=True)
    rospy.sleep(0.8)
    tmp = mf_ok.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=2.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['TH'] > force_zero['rh_TH'] or tactile_values['rh_MF'] > force_zero['rh_MF']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'Middle finger contact'
            break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(mf2rf_ok, angle_degrees=True)
    rospy.sleep(0.8)
    tmp = rf_ok.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=2.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['rh_TH'] > force_zero['rh_TH'] or tactile_values['rh_RF'] > force_zero['rh_RF']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'Ring finger contact'
            break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(rf2lf_ok, angle_degrees=True)
    rospy.sleep(0.8)
    tmp = lf_ok.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=2.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['TH'] > force_zero['TH'] or tactile_values['LF'] > force_zero['LF']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'Little finger contact'
            break

    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(flex_lf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_ff, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_mf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_rf, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(ext_lf, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(pre_ff_ok, angle_degrees=True)
    rospy.sleep(1)
    tmp = ff_ok.copy()
    hand_commander.move_to_joint_value_target_unsafe(tmp, time=3.0, angle_degrees=True)

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()
        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()
        # If the tactile sensor is triggered stop movement
        if tactile_values['rh_TH'] > force_zero['rh_TH'] or tactile_values['rh_FF'] > force_zero['rh_FF']:
            hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
            print 'First finger contact'
            break

    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, angle_degrees=True)
    rospy.sleep(1.4)
    hand_commander.move_to_joint_value_target_unsafe(ne_wr, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(nw_wr, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(sw_wr, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(se_wr, angle_degrees=True)
    rospy.sleep(0.7)
    hand_commander.move_to_joint_value_target_unsafe(zero_wr, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    return


def sequence_mf():
    # Start the sequence 2
    rospy.sleep(2.0)
    # Initialize wake time
    wake_time = time.time()

    while True:
        # Check if any of the tactile senors have been triggered
        # If so, send the Hand to its start position
        read_tactile_values()
        if (tactile_values['rh_FF'] > force_zero['rh_FF'] or
                    tactile_values['rh_MF'] > force_zero['rh_MF'] or
                    tactile_values['rh_RF'] > force_zero['rh_RF'] or
                    tactile_values['rh_LF'] > force_zero['rh_LF'] or
                    tactile_values['rh_TH'] > force_zero['rh_TH']):
            hand_commander.move_to_joint_value_target_unsafe(start_pos)
            print 'HAND TOUCHED!'
            rospy.sleep(2.0)

        if tactile_values['rh_TH'] > force_zero['rh_TH']:
            break

        # If the tactile sensors have not been triggered and the Hand
        # is not in the middle of a movement, generate a random position
        # and interpolation time
        else:
            if time.time() > wake_time:
                for i in rand_pos:
                    rand_pos[i] = random.randrange(min_range[i], max_range[i])

                rand_pos['rh_FFJ4'] = random.randrange(
                    min_range['rh_FFJ4'], rand_pos['rh_MFJ4'])
                rand_pos['rh_LFJ4'] = random.randrange(
                    min_range['rh_LFJ4'], rand_pos['rh_RFJ4'])
                rand_time = 4 * random.random()

                hand_commander.move_to_joint_value_target_unsafe(rand_pos, time=rand_time)
                wake_time = time.time() + rand_time * 0.9
    return


def sequence_rf():
    # Start the sequence 3
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_1, angle_degrees=True)
    rospy.sleep(2.5)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_2, angle_degrees=True)
    rospy.sleep(1)
    hand_commander.move_to_joint_value_target_unsafe(e_wr, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(w_wr, angle_degrees=True)
    rospy.sleep(0.4)
    hand_commander.move_to_joint_value_target_unsafe(zero_wr, angle_degrees=True)
    rospy.sleep(0.8)
    hand_commander.move_to_joint_value_target_unsafe(shake_grasp_1, angle_degrees=True)
    rospy.sleep(1.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    return


def sequence_lf():
    # Start the sequence 4
    # Trigger flag array
    trigger = [0, 0, 0, 0, 0]

    # Move Hand to zero position
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to starting position
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, angle_degrees=True)
    rospy.sleep(2.0)

    # Move Hand to close position
    hand_commander.move_to_joint_value_target_unsafe(grasp_pos, time=10, angle_degrees=True)
    offset1 = 3

    # Initialize end time
    end_time = time.time() + 11

    while True:
        # Check  the state of the tactile senors
        read_tactile_values()

        # Record current joint positions
        hand_pos = hand_commander.get_joints_position()

        # If any tacticle sensor has been triggered, send
        # the corresponding digit to its current position
        if tactile_values['rh_FF'] > force_zero['rh_FF'] and trigger[0] == 0:
            hand_commander.move_to_joint_value_target_unsafe(
                {"rh_FFJ1": hand_pos['rh_FFJ1'] + offset1, "rh_FFJ2": hand_pos['rh_FFJ2'] + offset1,
                 "rh_FFJ3": hand_pos['rh_FFJ3'] + offset1}, angle_degrees=True)
            print 'First finger contact'
            trigger[0] = 1

        if tactile_values['rh_MF'] > force_zero['rh_MF'] and trigger[1] == 0:
            hand_commander.move_to_joint_value_target_unsafe(
                {"rh_MFJ1": hand_pos['rh_MFJ1'] + offset1, "rh_MFJ2": hand_pos['rh_MFJ2'] + offset1,
                 "rh_MFJ3": hand_pos['rh_MFJ3'] + offset1}, angle_degrees=True)
            print 'Middle finger contact'
            trigger[1] = 1

        if tactile_values['rh_RF'] > force_zero['rh_RF'] and trigger[2] == 0:
            hand_commander.move_to_joint_value_target_unsafe(
                {"rh_RFJ1": hand_pos['rh_RFJ1'] + offset1, "rh_RFJ2": hand_pos['rh_RFJ2'] + offset1,
                 "rh_RFJ3": hand_pos['rh_RFJ3'] + offset1}, angle_degrees=True)
            print 'Ring finger contact'
            trigger[2] = 1

        if tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0:
            hand_commander.move_to_joint_value_target_unsafe(
                {"rh_LFJ1": hand_pos['rh_LFJ1'] + offset1, "rh_LFJ2": hand_pos['rh_LFJ2'] + offset1,
                 "rh_LFJ3": hand_pos['rh_LFJ3'] + offset1}, angle_degrees=True)
            print 'Little finger contact'
            trigger[3] = 1

        if tactile_values['rh_TH'] > force_zero['rh_TH'] and trigger[4] == 0:
            hand_commander.move_to_joint_value_target_unsafe(
                {"rh_THJ2": hand_pos['rh_THJ2'] + offset1, "rh_THJ5": hand_pos['rh_THJ5'] + offset1},
                angle_degrees=True)
            print 'Thumb contact'
            trigger[4] = 1

        if (trigger[0] == 1 and
            trigger[1] == 1 and
            trigger[2] == 1 and
            trigger[3] == 1 and
            trigger[4] == 1):
            break

        if time.time() > end_time:
            break

    # Send all joints to current position to compensate
    # for minor offsets created in the previous loop
    hand_pos = hand_commander.get_joints_position()
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
    rospy.sleep(2.0)

    # Generate new values to squeeze object slightly
    offset2 = 3
    squeeze = hand_pos.copy()
    squeeze.update(
        {"rh_THJ5": hand_pos['rh_THJ5'] + offset2, "rh_THJ2": hand_pos['rh_THJ2'] + offset2,
         "rh_FFJ3": hand_pos['rh_FFJ3'] + offset2, "rh_FFJ1": hand_pos['rh_FFJ1'] + offset2 / 2,
         "rh_FFJ2": hand_pos['rh_FFJ2'] + offset2 / 2,
         "rh_MFJ3": hand_pos['rh_MFJ3'] + offset2, "rh_MFJ1": hand_pos['rh_MFJ1'] + offset2 / 2,
         "rh_MFJ2": hand_pos['rh_MFJ2'] + offset2 / 2,
         "rh_RFJ3": hand_pos['rh_RFJ3'] + offset2, "rh_RFJ1": hand_pos['rh_RFJ1'] + offset2 / 2,
         "rh_RFJ2": hand_pos['rh_RFJ2'] + offset2 / 2,
         "rh_LFJ3": hand_pos['rh_LFJ3'] + offset2, "rh_LFJ1": hand_pos['rh_LFJ1'] + offset2 / 2,
         "rh_LFJ2": hand_pos['rh_LFJ2'] + offset2 / 2})

    # Squeeze object gently
    hand_commander.move_to_joint_value_target_unsafe(squeeze, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(squeeze, angle_degrees=True)
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(hand_pos, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(pregrasp_pos, angle_degrees=True)
    rospy.sleep(2.0)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(2.0)

    return


def sequence_th():
    # Start the sequence 5
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)
    rospy.sleep(1.5)
    return


def zero_tactile_sensors():
    # Move Hand to zero position
    rospy.sleep(0.5)
    hand_commander.move_to_joint_value_target_unsafe(start_pos, angle_degrees=True)

    print '\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n'
    # raw_input ('Press ENTER to continue')
    rospy.sleep(1.0)

    for _ in xrange(1, 1000):
        # Read current state of tactile sensors to zero them
        read_tactile_values()

        if tactile_values['rh_FF'] > force_zero['rh_FF']:
            force_zero['rh_FF'] = tactile_values['rh_FF']
        if tactile_values['rh_MF'] > force_zero['rh_MF']:
            force_zero['rh_MF'] = tactile_values['rh_MF']
        if tactile_values['rh_RF'] > force_zero['rh_RF']:
            force_zero['rh_RF'] = tactile_values['rh_RF']
        if tactile_values['rh_LF'] > force_zero['rh_LF']:
            force_zero['rh_LF'] = tactile_values['rh_LF']
        if tactile_values['rh_TH'] > force_zero['rh_TH']:
            force_zero['rh_TH'] = tactile_values['rh_TH']

    force_zero['rh_FF'] = force_zero['rh_FF'] + 3
    force_zero['rh_MF'] = force_zero['rh_MF'] + 3
    force_zero['rh_RF'] = force_zero['rh_RF'] + 3
    force_zero['rh_LF'] = force_zero['rh_LF'] + 3
    force_zero['rh_TH'] = force_zero['rh_TH'] + 3

    print 'Force Zero', force_zero

    rospy.loginfo("\n\nOK, ready for the demo")

    print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
    print "   FF: Standard Demo"
    print "   MF: Shy Hand Demo"
    print "   RF: Handshake Demo"
    print "   LF: Grasp Demo"
    print "   TH: Open Hand"

    return


def read_tactile_values():
    # Read tactile type
    tactile_type = hand_commander.get_tactile_type()
    # Read current state of tactile sensors
    tactile_state = hand_commander.get_tactile_state()

    if tactile_type == "biotac":
        tactile_values['rh_FF'] = tactile_state.tactiles[0].pdc
        tactile_values['rh_MF'] = tactile_state.tactiles[1].pdc
        tactile_values['rh_RF'] = tactile_state.tactiles[2].pdc
        tactile_values['rh_LF'] = tactile_state.tactiles[3].pdc
        tactile_values['rh_TH'] = tactile_state.tactiles[4].pdc

    elif tactile_type == "PST":
        tactile_values['rh_FF'] = tactile_state.pressure[0]
        tactile_values['rh_MF'] = tactile_state.pressure[1]
        tactile_values['rh_RF'] = tactile_state.pressure[2]
        tactile_values['rh_LF'] = tactile_state.pressure[3]
        tactile_values['rh_TH'] = tactile_state.pressure[4]

    elif tactile_type is None:
        print "You don't have tactile sensors. Talk to your Shadow representative to purchase some"

    return


#
# MAIN #
#

# Zero values in dictionary for tactile sensors (initialized at 0)
force_zero = {"rh_FF": 0, "rh_MF": 0, "rh_RF": 0, "rh_LF": 0, "rh_TH": 0}
# Initialize values for current tactile values
tactile_values = {"rh_FF": 0, "rh_MF": 0, "rh_RF": 0, "rh_LF": 0, "rh_TH": 0}
# Zero tactile sensors
zero_tactile_sensors()

while not rospy.is_shutdown():
    # Check the state of the tactile senors
    read_tactile_values()

    # If the tactile is touched, trigger the corresponding function
    if tactile_values['rh_FF'] > force_zero['rh_FF']:
        print 'First finger contact'
        sequence_ff()
        print 'FF demo completed'
        zero_tactile_sensors()
    if tactile_values['rh_MF'] > force_zero['rh_MF']:
        print 'Middle finger contact'
        sequence_mf()
        print 'MF demo completed'
        zero_tactile_sensors()
    if tactile_values['rh_RF'] > force_zero['rh_RF']:
        print 'Ring finger contact'
        sequence_rf()
        print 'RF demo completed'
        zero_tactile_sensors()
    if tactile_values['rh_LF'] > force_zero['rh_LF']:
        print 'Little finger contact'
        sequence_lf()
        print 'LF demo completed'
        zero_tactile_sensors()
    if tactile_values['rh_TH'] > force_zero['rh_TH']:
        print 'Thumb finger contact'
        sequence_th()
        print 'TH demo completed'
        zero_tactile_sensors()
