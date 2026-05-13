#!/bin/bash
pkill -9 -f "gz sim"                       2>/dev/null
pkill -9 -f gz_to_uds_bridge               2>/dev/null
pkill -9 -f aruco_to_vision                2>/dev/null
pkill -9 -f offboard_hover                 2>/dev/null
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f sentai_sim                     2>/dev/null
true
