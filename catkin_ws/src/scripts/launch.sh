#!/bin/sh

xterm -e " roslaunch levelManager lego_world.launch " &
sleep 10
xterm -e " rosrun levelManager levelManager.py -l 1 " &
sleep 5
xterm -e " rosrun motion_planning motion_planning.py " &
sleep 5
xterm -e " rosrun vision lego-vision.py -show " &
sleep 5
xterm -e " roslaunch voice_commands voice_commands.launch "