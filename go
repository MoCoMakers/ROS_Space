#!/bin/bash
sleep 2
cd ~/tmp
source /opt/nasa/indigo/setup.bash
rosrun srcsimwalk_thru.py
python ~/tmp/look_push_button.py
rosrun srcsimwalk_thru.py
 

