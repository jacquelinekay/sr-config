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
from sr_robot_commander import HandCommander
from sr_utilities.hand_finder import HandFinder

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

hand_mapping = hand_parameters.mapping[hand_serial]


rospy.init_node('receiver_example')

to_rad = hand_commander.convert_to_radians

##########
# RANGES #
##########

# Minimum alllowed range for the joints in this particular script
min_range = to_rad({"THJ2": -40, "THJ3": -12, "THJ4": 0, "THJ5": -55,
	     "FFJ1": 10, "FFJ2": 10, "FFJ3": 0, "FFJ4": -20,
	     "MFJ1": 10, "MFJ2": 10, "MFJ3": 0, "MFJ4": -10,
	     "RFJ1": 10, "RFJ2": 10, "RFJ3": 0, "RFJ4": -10,
	     "LFJ1": 10, "LFJ2": 10, "LFJ3": 0, "LFJ4": -20, "LFJ5": 0,
	     "WRJ1": -20, "WRJ2": -10})
	     # "interpolation_time": 0.0}

# Maximum alllowed range for the joints in this particular script
max_range = to_rad({"THJ2": 20, "THJ3": 12, "THJ4": 70, "THJ5": 0,
	     "FFJ1": 55, "FFJ2": 55, "FFJ3": 90, "FFJ4": 0,
	     "MFJ1": 55, "MFJ2": 55, "MFJ3": 90, "MFJ4": 0,
	     "RFJ1": 55, "RFJ2": 55, "RFJ3": 90, "RFJ4": 0,
	     "LFJ1": 55, "LFJ2": 55, "LFJ3": 90, "LFJ4": 0, "LFJ5": 1,
	     "WRJ1": 10, "WRJ2": 5})
	     # "interpolation_time": 4.0}



####################
# Pose DEFINITIONS #
####################

# starting position for the hand
start_pos = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
		  "FFJ1": 0, "FFJ2": 0, "FFJ3": 0, "FFJ4": 0,
		  "MFJ1": 0, "MFJ2": 0, "MFJ3": 0, "MFJ4": 0,
		  "RFJ1": 0, "RFJ2": 0, "RFJ3": 0, "RFJ4": 0,
		  "LFJ1": 0, "LFJ2": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
		  "WRJ1": 0, "WRJ2": 0 })
# Start position for the Hand
pregrasp_pos = to_rad({"THJ2": 12, "THJ3": 15, "THJ4": 69, "THJ5": -23,
	        "FFJ1": 20, "FFJ2": 20, "FFJ3": 21, "FFJ4": -15,
	        "MFJ1": 20, "MFJ2": 20, "MFJ3": 21, "MFJ4": 0,
	        "RFJ1": 20, "RFJ2": 20, "RFJ3": 21, "RFJ4": -7,
	        "LFJ1": 20, "LFJ2": 20, "LFJ3": 21, "LFJ4": -10, "LFJ5": 0,
	        "WRJ1": 0, "WRJ2": 0})
# Close position for the Hand
grasp_pos = to_rad({"THJ2": 30, "THJ3": 15, "THJ4": 69, "THJ5": -3,
	     "FFJ1": 38, "FFJ2": 38, "FFJ3": 67, "FFJ4": -19,
	     "MFJ1": 41, "MFJ2": 41, "MFJ3": 62, "MFJ4": 0,
	     "RFJ1": 45, "RFJ2": 45, "RFJ3": 64, "RFJ4": -18,
	     "LFJ1": 48, "LFJ2": 48, "LFJ3": 64, "LFJ4": -19, "LFJ5": 0,
	     "WRJ1": 0, "WRJ2": 0})

# Random position for the Hand (initialied at 0)
rand_pos = to_rad({"THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
	    "FFJ1": 0, "FFJ2": 0, "FFJ3": 0, "FFJ4": 0,
	    "MFJ1": 0, "MFJ2": 0, "MFJ3": 0, "MFJ4": 0,
	    "RFJ1": 0, "RFJ2": 0, "RFJ3": 0, "RFJ4": 0,
	    "LFJ1": 0, "LFJ2": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
            "WRJ1": 0, "WRJ2": 0})

# flex first finger
flex_ff = to_rad({"FFJ1": 90, "FFJ2": 90, "FFJ3": 90, "FFJ4": 0 })
# extend first finger
ext_ff = to_rad({"FFJ1": 0, "FFJ2": 0, "FFJ3": 0, "FFJ4": 0 })
# flex middle finger
flex_mf = to_rad({"MFJ1": 90, "MFJ2": 90, "MFJ3": 90, "MFJ4": 0 })
# extend middle finger
ext_mf = to_rad({"MFJ1": 0, "MFJ2": 0, "MFJ3": 0, "MFJ4": 0 })
# flex ring finger
flex_rf = to_rad({"RFJ1": 90, "RFJ2": 90, "RFJ3": 90, "RFJ4": 0 })
# extend ring finger
ext_rf = to_rad({"RFJ1": 0, "RFJ2": 0, "RFJ3": 0, "RFJ4": 0 })
# flex little finger
flex_lf = to_rad({"LFJ1": 90, "LFJ2": 90, "LFJ3": 90, "LFJ4": 0 })
# extend middle finger
ext_lf = to_rad({"LFJ1": 0, "LFJ2": 0, "LFJ3": 0, "LFJ4": 0 })
# flex thumb step 1
flex_th_1 = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 70, "THJ5": 0 })
# flex thumb step 2
flex_th_2 = to_rad({"THJ1": 35, "THJ2": 38, "THJ3": 10, "THJ4": 70, "THJ5": 58 })
# extend thumb step 1
ext_th_1 = to_rad({"THJ1": 10, "THJ2": 20, "THJ3": 5, "THJ4": 35, "THJ5": 25 })
# extend thumb step 2
ext_th_2 =to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0 })
# zero thumb
zero_th = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0 })
# Pre O.K. with first finger
pre_ff_ok = to_rad({"THJ4": 70 })
# O.K. with first finger
ff_ok = to_rad({"THJ1": 15, "THJ2": 20, "THJ3": 0, "THJ4": 56, "THJ5": 20,
	 "FFJ0": 75, "FFJ3": 45, "FFJ4": -0.2,
	 "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
	 "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
	 "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7 })
# O.K. transition from first finger to middle finger
ff2mf_ok = to_rad({"THJ1": 5, "THJ2": 12, "THJ3": 4, "THJ4": 60, "THJ5": 2,
	    "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	    "MFJ0": 42, "MFJ3": 33, "MFJ4": -3,
	    "RFJ0": 50, "RFJ3": 18, "RFJ4": 0.5,
	    "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7 })
# O.K. with middle finger
mf_ok = to_rad({"THJ1": 15, "THJ2": 18, "THJ3": 7, "THJ4": 66, "THJ5": 30,
	 "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	 "MFJ0": 71, "MFJ3": 54, "MFJ4": 11,
	 "RFJ0": 50, "RFJ3": 18, "RFJ4": -10,
	 "LFJ0": 30, "LFJ3": 0, "LFJ4": -6, "LFJ5": 7 })
# O.K. transition from middle finger to ring finger
mf2rf_ok = to_rad({"THJ1": 5, "THJ2": -5, "THJ3": 15, "THJ4": 70, "THJ5": 19,
	    "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	    "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
	    "RFJ0": 50, "RFJ3": 18, "RFJ4": -19,
	    "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7 })
# O.K. with ring finger
rf_ok = to_rad({"THJ1": 6, "THJ2": 15, "THJ3": 15, "THJ4": 70, "THJ5": 45,
	 "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	 "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
	 "RFJ0": 93, "RFJ3": 42, "RFJ4": -19,
	 "LFJ0": 30, "LFJ3": 0, "LFJ4": -12, "LFJ5": 7 })
# O.K. transition from ring finger to little finger
rf2lf_ok = to_rad({"THJ1": 5, "THJ2": 4.5, "THJ3": 8, "THJ4": 60, "THJ5": 21,
	    "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	    "MFJ0": 45, "MFJ3": 4, "MFJ4": -1,
	    "RFJ0": 30, "RFJ3": 6, "RFJ4": 0.5,
	    "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 7 })
# O.K. with little finger
lf_ok = to_rad({"THJ1": 21, "THJ2": 11, "THJ3": 10, "THJ4": 69, "THJ5": 26,
	 "FFJ0": 14, "FFJ3": 7, "FFJ4": -0.4,
	 "MFJ0": 15, "MFJ3": 4, "MFJ4": -1,
	 "RFJ0": 15, "RFJ3": 6, "RFJ4": 0.5,
	 "LFJ0": 76, "LFJ3": 26, "LFJ4": 6, "LFJ5": 35 })
# zero wrist
zero_wr = to_rad({"WRJ1": 0, "WRJ2": 0 })
# north wrist
n_wr = to_rad({"WRJ1": 15, "WRJ2": 0 })
# south wrist
s_wr = to_rad({"WRJ1": -20, "WRJ2": 0 })
# east wrist
e_wr = to_rad({"WRJ1": 0, "WRJ2": 8 })
# west wrist
w_wr = to_rad({"WRJ1": 0, "WRJ2": -14 })
# northeast wrist
ne_wr = to_rad({"WRJ1": 15, "WRJ2": 8 })
# northwest wrist
nw_wr = to_rad({"WRJ1": 15, "WRJ2": -14 })
# southweast wrist
sw_wr = to_rad({"WRJ1": -20, "WRJ2": -14 })
# southeast wrist
se_wr = to_rad({"WRJ1": -20, "WRJ2": 8 })
# lateral lf ext side
l_ext_lf = to_rad({"LFJ4": -15 })
# lateral rf ext side
l_ext_rf = to_rad({"RFJ4": -15 })
# lateral mf ext side
l_ext_mf = to_rad({"MFJ4": 15 })
# lateral ff ext side
l_ext_ff = to_rad({"FFJ4": 15 })
# lateral all int side
l_int_all = to_rad({"FFJ4": -15, "MFJ4": -15, "RFJ4": 15, "LFJ4": 15 })
# lateral all ext side
l_ext_all = to_rad({"FFJ4": 15, "MFJ4": 15, "RFJ4": -15, "LFJ4": -15 })
# lateral ff int side
l_int_ff = to_rad({"FFJ4": -15 })
# lateral mf int side
l_int_mf = to_rad({"MFJ4": -15 })
# lateral rf int side
l_int_rf = to_rad({"RFJ4": 15 })
# lateral lf int side
l_int_lf = to_rad({"LFJ4": 15 })
# all zero
l_zero_all = to_rad({"FFJ4": 0, "MFJ4": 0, "RFJ4": 0, "LFJ4": 0 })
# spock
l_spock = to_rad({"FFJ4": -20, "MFJ4": -20, "RFJ4": -20, "LFJ4": -20 })
# grasp for shaking hands step 1
shake_grasp_1 = to_rad({"THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
		 "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
		 "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
		 "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
		 "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10 })
# grasp for shaking hands step 2
shake_grasp_2 = to_rad({"THJ1": 21, "THJ2": 21, "THJ3": 10, "THJ4": 42, "THJ5": 21,
		 "FFJ0": 75, "FFJ3": 29, "FFJ4": 0,
		 "MFJ0": 75, "MFJ3": 41, "MFJ4": 0,
		 "RFJ0": 75, "RFJ3": 41, "RFJ4": 0,
		 "LFJ0": 100, "LFJ3": 41, "LFJ4": 0, "LFJ5": 0 })
# store step 1 PST
store_1_PST = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 60, "THJ5": 0,
	       "FFJ1": 90, "FFJ2": 90, "FFJ3": 90, "FFJ4": 0,
	       "MFJ1": 90, "MFJ2": 90, "MFJ3": 90, "MFJ4": 0,
	       "RFJ1": 90, "RFJ2": 90, "RFJ3": 90, "RFJ4": 0,
	       "LFJ1": 90, "LFJ2": 90, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
	       "WRJ1": 0, "WRJ2": 0 })
# store step 2 PST
store_2_PST = to_rad({"THJ1": 50, "THJ2": 12, "THJ3": 0, "THJ4": 60, "THJ5": 27,
	       "FFJ1": 90, "FFJ2": 90, "FFJ3": 90, "FFJ4": 0,
	       "MFJ1": 90, "MFJ2": 90, "MFJ3": 90, "MFJ4": 0,
	       "RFJ1": 90, "RFJ2": 90, "RFJ3": 90, "RFJ4": 0,
	       "LFJ1": 90, "LFJ2": 90, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
	       "WRJ1": 0, "WRJ2": 0 })
# store step 1 Bio_Tac
store_1_BioTac = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 30, "THJ5": 0,
		  "FFJ1": 90, "FFJ2": 90, "FFJ3": 90, "FFJ4": 0,
		  "MFJ1": 90, "MFJ2": 90, "MFJ3": 90, "MFJ4": 0,
		  "RFJ1": 90, "RFJ2": 90, "RFJ3": 90, "RFJ4": 0,
		  "LFJ1": 90, "LFJ2": 90, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
		  "WRJ1": 0, "WRJ2": 0 })
# store step 2 Bio_Tac
store_2_BioTac = to_rad({"THJ1": 20, "THJ2": 36, "THJ3": 0, "THJ4": 30, "THJ5": -3,
		  "FFJ1": 90, "FFJ2": 90, "FFJ3": 90, "FFJ4": 0,
		  "MFJ1": 90, "MFJ2": 90, "MFJ3": 90, "MFJ4": 0,
		  "RFJ1": 90, "RFJ2": 90, "RFJ3": 90, "RFJ4": 0,
		  "LFJ1": 90, "LFJ2": 90, "LFJ3": 90, "LFJ4": 0, "LFJ5": 0,
		  "WRJ1": 0, "WRJ2": 0 })
# store step 3
store_3 = to_rad({"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 65, "THJ5": 0 })
# business card pre-zero position
bc_pre_zero = to_rad({"THJ1": 15, "THJ2": 7, "THJ3": -4, "THJ4": 50, "THJ5": -17,
		  "FFJ1": 7, "FFJ2": 7, "FFJ3": 7, "FFJ4": -1,
		  "MFJ1": 25.5, "MFJ2": 25.5, "MFJ3": 33, "MFJ4": 20,
		  "RFJ1": 25, "RFJ2": 25, "RFJ3": 18, "RFJ4": -20,
		  "LFJ1": 15, "LFJ2": 15, "LFJ3": 0, "LFJ4": -20, "LFJ5": 7 })
# business card zero position
bc_zero = to_rad({"THJ1": 23, "THJ2": 6, "THJ3": -1, "THJ4": 43, "THJ5": -11,
		  "MFJ1": 31.5, "MFJ2": 31.5, "MFJ3": 24, "MFJ4": 20 })
# business card position 1
bc_1 = to_rad({"FFJ1": 67.5, "FFJ2": 67.5, "FFJ3": 7 })
# business card position 2
bc_2 = to_rad({"FFJ1": 67.5, "FFJ2": 67.5, "FFJ3": 58 })
# business card position 3
bc_3 = to_rad({"FFJ1": 36, "FFJ2": 36, "FFJ3": 62 })
# business card position 4
bc_4 = to_rad({"FFJ1": 90, "FFJ2": 90, "FFJ3": 58 })
# business card position 5
bc_5 = to_rad({"FFJ1": 90, "FFJ2": 90, "FFJ3": 0 })
# business card position 6
bc_6 = to_rad({"FFJ1": 0, "FFJ2": 0, "FFJ3": 0 })
# business card position 7
bc_7 = to_rad({"FFJ1": 67.5, "FFJ2": 67.5, "FFJ3": 15 })
# business card position 8
bc_8 = to_rad({"FFJ1": 67.5, "FFJ2": 67.5, "FFJ3": 58 })
# business card position 9
bc_9 = to_rad({"FFJ1": 40, "FFJ2": 40, "FFJ3": 58 })
# business card position 10
bc_10 = to_rad({"MFJ3": 64, "FFJ4": 20 })
# business card position 11
bc_11 = to_rad({"FFJ1": 40.5, "FFJ2": 40.5, "FFJ3": 50, "FFJ4": 20,
        "THJ4": 57, "THJ5": 25, })
# business card position 12
bc_12 = to_rad({"MFJ1": 10, "MFJ2": 10, "MFJ3": 10, "MFJ4": 0 })


########################
# FUNCTION DEFINITIONS #
########################

def secuence_ff():
   # Start secuence 1
   trajectory = [{'angles': store_3, 'interpolation_time': 1},
                 {'angles': start_pos, 'interpolation_time': 1.2},
                 {'angles': flex_ff, 'interpolation_time': 1},
                 {'angles': ext_ff, 'interpolation_time': 1},
                 {'angles': flex_mf, 'interpolation_time': 1},
                 {'angles': ext_mf, 'interpolation_time': 1},
                 {'angles': flex_rf, 'interpolation_time': 1},
                 {'angles': ext_rf, 'interpolation_time': 1},
                 {'angles': flex_lf, 'interpolation_time': 1},
                 {'angles': ext_lf, 'interpolation_time': 1},
                 {'angles': flex_th_1, 'interpolation_time': 0.7},
                 {'angles': flex_th_2, 'interpolation_time': 1},
                 {'angles': ext_th_2, 'interpolation_time': 0.5},
                 {'angles': l_ext_lf, 'interpolation_time': 0.5},
                 {'angles': l_ext_rf, 'interpolation_time': 0.5},
                 {'angles': l_ext_mf, 'interpolation_time': 0.5},
                 {'angles': l_ext_ff, 'interpolation_time': 0.5},
                 {'angles': l_int_all, 'interpolation_time': 0.5},
                 {'angles': l_ext_all, 'interpolation_time': 0.5},
                 {'angles': l_int_ff, 'interpolation_time': 0.5},
                 {'angles': l_int_mf, 'interpolation_time': 0.5},
                 {'angles': l_int_rf, 'interpolation_time': 0.5},
                 {'angles': l_int_lf, 'interpolation_time': 0.5},
                 {'angles': l_zero_all, 'interpolation_time': 0.5},
                 {'angles': l_spock, 'interpolation_time': 0.5},
                 {'angles': l_zero_all, 'interpolation_time': 0.5},
                 {'angles': pre_ff_ok, 'interpolation_time': 1},
                 {'angles': ff_ok, 'interpolation_time': 1},
                 {'angles': ff2mf_ok, 'interpolation_time': 0.8},
                 {'angles': mf_ok, 'interpolation_time': 1},
                 {'angles': mf2rf_ok, 'interpolation_time': 0.8},
                 {'angles': rf_ok, 'interpolation_time': 1},
                 {'angles': rf2lf_ok, 'interpolation_time': 0.8},
                 {'angles': lf_ok, 'interpolation_time': 1},
                 {'angles': start_pos, 'interpolation_time': 1},
                 {'angles': flex_ff, 'interpolation_time': 0.2},
                 {'angles': flex_mf, 'interpolation_time': 0.2},
                 {'angles': flex_rf, 'interpolation_time': 0.2},
                 {'angles': flex_lf, 'interpolation_time': 0.2},
                 {'angles': ext_ff, 'interpolation_time': 0.2},
                 {'angles': ext_mf, 'interpolation_time': 0.2},
                 {'angles': ext_rf, 'interpolation_time': 0.2},
                 {'angles': ext_lf, 'interpolation_time': 0.2},
                 {'angles': flex_ff, 'interpolation_time': 0.2},
                 {'angles': flex_mf, 'interpolation_time': 0.2},
                 {'angles': flex_rf, 'interpolation_time': 0.2},
                 {'angles': flex_lf, 'interpolation_time': 0.2},
                 {'angles': ext_ff, 'interpolation_time': 0.2},
                 {'angles': ext_mf, 'interpolation_time': 0.2},
                 {'angles': ext_rf, 'interpolation_time': 0.2},
                 {'angles': ext_lf, 'interpolation_time': 0.2},
                 {'angles': flex_ff, 'interpolation_time': 0.2},
                 {'angles': flex_mf, 'interpolation_time': 0.2},
                 {'angles': flex_rf, 'interpolation_time': 0.2},
                 {'angles': flex_lf, 'interpolation_time': 0.2},
                 {'angles': ext_ff, 'interpolation_time': 0.2},
                 {'angles': ext_mf, 'interpolation_time': 0.2},
                 {'angles': ext_rf, 'interpolation_time': 0.2},
                 {'angles': ext_lf, 'interpolation_time': 1},
                 {'angles': pre_ff_ok, 'interpolation_time': 1},
                 {'angles': ff_ok, 'interpolation_time': 1},
                 {'angles': ne_wr, 'interpolation_time': 1.4},
                 {'angles': nw_wr, 'interpolation_time': 1.4},
                 {'angles': sw_wr, 'interpolation_time': 1.4},
                 {'angles': se_wr, 'interpolation_time': 1.4},
                 {'angles': ne_wr, 'interpolation_time': 0.7},
                 {'angles': nw_wr, 'interpolation_time': 0.7},
                 {'angles': sw_wr, 'interpolation_time': 0.7},
                 {'angles': se_wr, 'interpolation_time': 0.7},
                 {'angles': zero_wr, 'interpolation_time': 0.4},
                 {'angles': start_pos, 'interpolation_time': 1.5}]
   hand_commander
   return

def secuence_mf():
   # Start the secuence 2
   rospy.sleep(2.0)
   # Initialize wake time
   wake_time = time.time()

   while True:
      # Check if any of the tactile senors have been triggered
      # If so, send the Hand to its start position
      read_tactile_values()
      if ( tactile_values['FF'] > force_zero['FF'] or
           tactile_values['MF'] > force_zero['MF'] or
           tactile_values['RF'] > force_zero['RF'] or
           tactile_values['LF'] > force_zero['LF'] or
           tactile_values['TH'] > force_zero['TH'] ):

         c.move_hand(start_pos)
         print 'HAND TOUCHED!'
         rospy.sleep(2.0)

         if ( tactile_values['TH'] > force_zero['TH'] ):
            break

      # If the tactile sensors have not been triggered and the Hand
      # is not in the middle of a movement, generate a random position
      # and interpolation time
      else:
         if time.time() > wake_time:
            for i in rand_pos:
               rand_pos[i] = random.randrange(min_range[i],max_range[i])

            rand_pos['FFJ4'] = random.randrange(min_range['FFJ4'],rand_pos['MFJ4'])
            rand_pos['LFJ4'] = random.randrange(min_range['LFJ4'],rand_pos['RFJ4'])
            rand_pos['interpolation_time'] = max_range['interpolation_time'] * random.random()

            c.move_hand(rand_pos)
            wake_time = time.time() + rand_pos['interpolation_time']*0.9
   return

def secuence_rf():
   # Start the secuence 3
#   rospy.sleep(0.5)
#   c.move_hand(start_pos)
#   rospy.sleep(1.5)
#   c.move_hand(shake_grasp_1)
#   rospy.sleep(2.5)
#   c.move_hand(shake_grasp_2)
#   rospy.sleep(1)
#   c.move_hand(e_wr)
#   rospy.sleep(0.4)
#   c.move_hand(w_wr)
#   rospy.sleep(0.4)
#   c.move_hand(zero_wr)
#   rospy.sleep(0.8)
#   c.move_hand(shake_grasp_1)
#   rospy.sleep(1.5)
#   c.move_hand(start_pos)
#   rospy.sleep(1.5)

   rospy.sleep(0.5)
{'angles': start_pos, 'interpolation_time': 1},
{'angles': bc_pre_zero, 'interpolation_time': 2},
{'angles': bc_zero, 'interpolation_time': 4},
{'angles': bc_1, 'interpolation_time': 1},
{'angles': bc_2, 'interpolation_time': 1},
{'angles': bc_3, 'interpolation_time': 1},
{'angles': bc_4, 'interpolation_time': 1},
{'angles': bc_5, 'interpolation_time': 1},
{'angles': bc_6, 'interpolation_time': 1},
{'angles': bc_7, 'interpolation_time': 1},
{'angles': bc_8, 'interpolation_time': 1},
{'angles': bc_9, 'interpolation_time': 1},
{'angles': bc_11, 'interpolation_time': 1},
{'angles': bc_12, 'interpolation_time': 3},
{'angles': start_pos, 'interpolation_time': 1.5},

   return

def secuence_lf():
   # Start the secuence 4
   # Trigger flag array
   trigger = [0,0,0,0,0]

   # Move Hand to zero position
{'angles': start_pos, 'interpolation_time': 2.0},

   # Move Hand to starting position
{'angles': pregrasp_pos, 'interpolation_time': 2.0},

   # Move Hand to close position
   c.move_hand(grasp_pos)
   offset1 = 3

   # Initialize end time
   end_time = time.time() + 11

   while True:
      # Check  the state of the tactile senors
      read_tactile_values()

      # Record current joint positions
      hand_pos = c.get_hand_position()

      # If any tacticle sensor has been triggered, send
      # the corresponding digit to its current position
      if ( tactile_values['FF'] > force_zero['FF'] and trigger[0] == 0 ):
         c.move_hand({"FFJ0": hand_pos['FFJ0'] + offset1, "FFJ3": hand_pos['FFJ3'] + offset1})
         print 'First finger contact'
         trigger[0] = 1

      if ( tactile_values['MF'] > force_zero['MF'] and trigger[1] == 0 ):
         c.move_hand({"MFJ0": hand_pos['MFJ0'] + offset1, "MFJ3": hand_pos['MFJ3'] + offset1})
         print 'Middle finger contact'
         trigger[1] = 1

      if ( tactile_values['RF'] > force_zero['RF'] and trigger[2] == 0 ):
         c.move_hand({"RFJ0": hand_pos['RFJ0'] + offset1, "RFJ3": hand_pos['RFJ3'] + offset1})
         print 'Ring finger contact'
         trigger[2] = 1

      if ( tactile_values['LF'] > force_zero['LF'] and trigger[3] == 0 ):
         c.move_hand({"LFJ0": hand_pos['LFJ0'] + offset1, "LFJ3": hand_pos['LFJ3'] + offset1})
         print 'Little finger contact'
         trigger[3] = 1

      if ( tactile_values['TH'] > force_zero['TH'] and trigger[4] == 0 ):
         c.move_hand({"THJ2": hand_pos['THJ2'] + offset1, "THJ5": hand_pos['THJ5'] + offset1 })
         print 'Thumb contact'
         trigger[4] = 1

      if ( trigger[0] == 1 and
	   trigger[1] == 1 and
	   trigger[2] == 1 and
	   trigger[3] == 1 and
	   trigger[4] == 1 ):
         break

      if time.time() > end_time:
         break

   # Send all joints to current position to compensate
   # for minor offsets created in the previous loop
   hand_pos = c.get_hand_position()
{'angles': hand_pos, 'interpolation_time': 2.0},

   # Generate new values to squeeze object slightly
   offset2 = 3
   squeeze = hand_pos.copy()
   squeeze.update({"THJ5": hand_pos['THJ5'] + offset2, "THJ2": hand_pos['THJ2'] + offset2,
		   "FFJ3": hand_pos['FFJ3'] + offset2, "FFJ0": hand_pos['FFJ0'] + offset2,
   		   "MFJ3": hand_pos['MFJ3'] + offset2, "MFJ0": hand_pos['MFJ0'] + offset2,
		   "RFJ3": hand_pos['RFJ3'] + offset2, "RFJ0": hand_pos['RFJ0'] + offset2,
		   "LFJ3": hand_pos['LFJ3'] + offset2, "LFJ0": hand_pos['LFJ0'] + offset2})

   # Squeeze object gently
{'angles': squeeze, 'interpolation_time': 0.5},
{'angles': hand_pos, 'interpolation_time': 0.5},
{'angles': squeeze, 'interpolation_time': 0.5},
{'angles': hand_pos, 'interpolation_time': 2.0},
{'angles': pregrasp_pos, 'interpolation_time': 2.0},
{'angles': start_pos, 'interpolation_time': 2.0},

   return

def secuence_th():
   # Start the secuence 5
   rospy.sleep(0.5)
{'angles': start_pos, 'interpolation_time': 1.5},
   return

def zero_tactile_sensors():
   # Move Hand to zero position
   rospy.sleep(0.5)
   c.move_hand(start_pos)

   print '\n\nPLEASE ENSURE THAT THE TACTILE SENSORS ARE NOT PRESSED\n'
   #raw_input ('Press ENTER to continue')
   rospy.sleep(1.0)

   for x in xrange (1,1000):
      # Read current state of tactile sensors to zero them
      read_tactile_values()

      if tactile_values['FF'] > force_zero['FF']:
         force_zero['FF'] = tactile_values['FF']
      if tactile_values['MF'] > force_zero['MF']:
         force_zero['MF'] = tactile_values['MF']
      if tactile_values['RF'] > force_zero['RF']:
         force_zero['RF'] = tactile_values['RF']
      if tactile_values['LF'] > force_zero['LF']:
         force_zero['LF'] = tactile_values['LF']
      if tactile_values['TH'] > force_zero['TH']:
         force_zero['TH'] = tactile_values['TH']

   force_zero['FF'] = force_zero['FF'] + 3
   force_zero['MF'] = force_zero['MF'] + 3
   force_zero['RF'] = force_zero['RF'] + 3
   force_zero['LF'] = force_zero['LF'] + 3
   force_zero['TH'] = force_zero['TH'] + 3

   print 'Force Zero', force_zero

   rospy.loginfo("\n\nOK, ready for the demo")

   print "\nPRESS ONE OF THE TACTILES TO START A DEMO"
   print "   FF: Standard Demo"
   print "   MF: Shy Hand Demo"
   print "   RF: Card Trick Demo"
   print "   LF: Grasp Demo"
   print "   TH: Open Hand"

   return

def read_tactile_values():
   # Read tactile type
   tactile_type = c.get_tactile_type()
   # Read current state of tactile sensors
   tactile_state = c.get_tactile_state()

   if tactile_type == "biotac":
      tactile_values['FF'] = tactile_state.tactiles[0].pdc
      tactile_values['MF'] = tactile_state.tactiles[1].pdc
      tactile_values['RF'] = tactile_state.tactiles[2].pdc
      tactile_values['LF'] = tactile_state.tactiles[3].pdc
      tactile_values['TH'] = tactile_state.tactiles[4].pdc

   elif tactile_type == "PST":
      tactile_values['FF'] = tactile_state.pressure[0]
      tactile_values['MF'] = tactile_state.pressure[1]
      tactile_values['RF'] = tactile_state.pressure[2]
      tactile_values['LF'] = tactile_state.pressure[3]
      tactile_values['TH'] = tactile_state.pressure[4]

   elif tactile_type == None:
      print "You don't have tactile sensors. Talk to your Shadow representative to purchase some"

   return

########
# MAIN #
########

# Zero values in dictionary for tactile sensors (initialized at 0)
force_zero = {"FF": 0,"MF": 0,"RF": 0,"LF": 0,"TH": 0}
# Initialize values for current tactile values
tactile_values = {"FF": 0,"MF": 0,"RF": 0,"LF": 0,"TH": 0}
# Zero tactile sensors
zero_tactile_sensors()

while not rospy.is_shutdown():
   # Check the state of the tactile senors
   read_tactile_values()

   # If the tactile is touched, trigger the corresponding function
   if (tactile_values['FF'] > force_zero['FF']):
      print 'First finger contact'
      secuence_ff()
      print 'FF demo completed'
      zero_tactile_sensors()
   if (tactile_values['MF'] > force_zero['MF']):
      print 'Middle finger contact'
      secuence_mf()
      print 'MF demo completed'
      zero_tactile_sensors()
   if (tactile_values['RF'] > force_zero['RF']):
      print 'Ring finger contact'
      secuence_rf()
      print 'RF demo completed'
      zero_tactile_sensors()
   if (tactile_values['LF'] > force_zero['LF']):
      print 'Little finger contact'
      secuence_lf()
      print 'LF demo completed'
      zero_tactile_sensors()
   if (tactile_values['TH'] > force_zero['TH']):
      print 'Thumb finger contact'
      secuence_th()
      print 'TH demo completed'
      zero_tactile_sensors()
