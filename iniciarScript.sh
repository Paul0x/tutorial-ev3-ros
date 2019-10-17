#!/bin/bash
echo "Inicializando ambiente catkin"
cd ~/ctk_ws
source devel/setup.bash
echo "Inicializando script"
rosrun robotica_tp_py start.py

