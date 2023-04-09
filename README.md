# FinalYearProject_LuggageBot

## System
> `Ubuntu 22.04 LTS`, `Python 3.6`, `ROS Noetic`

## Illustrations Screenshots
> The GIF below represents the Demonstration of `Robot Following Cylinder`. Complete Video Available [Here](/Screenshots/Robot_Following_Cylinder.mp4)

![Cylinder GIF](/Screenshots/CylinderGIF.gif) 

> The GIF below represents the Demonstration of `Robot Following Person`. Complete Video Available [Here](/Screenshots/Robot_Following_Person.mp4)

![Cylinder GIF](/Screenshots/PersonGIF.gif)

## ROS Setup Process:

Paste this command in `.bashrc` EOD file . This file is located inside home, under hidden files.
> `echo "source /opt/ros/noetic/setup.bash"`

This command is used for linking the `.bashrc` to `ROS Noetic`. If above Instructions is not performed, then when executing ROS commands, we need to execute above command everytime inside terminal before we run ROS commands.

## Instructions

Download https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5 & Move into -> ros_starter_ws\src\Mask_RCNN\scripts Folder

Change the Paths In the mentioned directory below to the path

> `scripts/follow_person.py` located at `line number 16,17 and 18`

> Inside `launchFile` for both **_luggageBot_cylinder_** & **_luggageBot_person_**. Make sure to change the Path In the mentioned directory with your workspace.

> `<arg name="world" default="/home/ronit/ros_starter_ws/src/Mask_RCNN/launchFile/world/luggageBot_cylinder.world"/>`

## Installations of MS COCO
> Terminal Command: `cd scripts & git clone https://github.com/cocodataset/cocoapi.git`

> Terminal Command: `cd scripts/cocoapi & python PythonAPI/setup.py build_ext install`

## Libraries Installations requirements for cocodataset & maskrcnn_node.py
> Terminal Command: `pip install -r requirements.txt`

## Launch LuggageBot - Robot with cylinder [1]
> Terminal Command: `roslaunch launchFile/luggageBot_cylinder.launch`

## Run green cylinder follower node
> Terminal Command: `python3.6 scripts/follow_cylinder.py`

Terminal Output: `Following Green Cylinder!` **OR** `Moving back! in terminal`

## To animate the green cylinder to move randomly
> Terminal Command: `python3.6 scripts/animate_cylinder.py`

The cylinder will animate/make a move in random direction and luggageBot will follow it`

## Rviz view
> Terminal Command: `rviz`

This command will give you `camera view` before and after image segmentation

Inside RViz: `Bottom left cornor -> Option Add -> by_topic -> camera [OR] camera_result`

![Cylinder RViz View](/Screenshots/CylinderRvizView.jpg) 

## Launch LuggageBot - Robot following person [2]
> Terminal Command: `roslaunch launchFile/luggageBot_person.launch`

## Run person follower node
> Terminal Command: `python3.6 scripts/follow_person.py`

This script might take some time to run depending upon CPU/GPU power. 

After the script is executed, Terminal Output: `Person not detected!` **OR** `Received Histogram!`

After the person is detected, Terminal Output: `Following person!` **OR** `Moving back!`

## To animate the user to move randomly
> Terminal Command: `python3.6 scripts/animate_person.py`

The person/human will animate/make a move and luggageBot will follow it`

Please use `person_walking` model under `launchFile/models`. Other model might have missing `mesh/texture` file that might cause it behave unexpectedly

## Rviz view
> Terminal Command: `rviz`

This command will give you `camera view` before and after image segmentation

Inside RViz: `Bottom left cornor -> Option Add -> by_topic -> camera [OR] camera_result`

![Person RViz View](/Screenshots/RvizFinal.jpg) 

## Histogram [3]
Histogram is only saved only for person following version because saving histogram for green cylinder does not make sense.

After the excution of `follow_person.py` script is stopped `(press Ctrl+C )`, `user_histogram.png` is saved in script folder with the message Terminal Output: `Histogram saved!`

![Color Histogram](/Screenshots/user_histogram.png) 
