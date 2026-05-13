#!/bin/bash
export GZ_SIM_RESOURCE_PATH="/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds:/home/bogdan/work/px4/PX4-Autopilot/Tools/simulation/gz/models"
export PX4_GZ_MODELS="/home/bogdan/work/px4/PX4-Autopilot/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds"
cd "/home/bogdan/work/px4/PX4-Autopilot"
nohup env HEADLESS=1 PX4_SYS_AUTOSTART=4043 PX4_SIMULATOR=gz     PX4_GZ_MODEL=x500_sentai PX4_GZ_WORLD=sentai_crazysim     PX4_GZ_MODEL_POSE='0,0,1.0,0,0,0'     "/home/bogdan/work/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4" -i 0 -d "/home/bogdan/work/px4/PX4-Autopilot/build/px4_sitl_default/etc" > "/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s112_x86_anchor_shim/run_20260512_221909/px4.log" 2>&1 &
disown
