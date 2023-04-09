# Instructions
`Change the Path In the mentioned directory below to the path`
`scripts/follow_person.py`
`line number 16,17 and 18`

`launchFile for BOTH luggageBot_cylinder & luggageBot_person`
`Make sure to change the Path In the mentioned directory to your workspace`
`<arg name="world" default="/home/ronit/ros_starter_ws/src/Mask_RCNN/launchFile/world/luggageBot_cylinder.world"/>`

## Installations
`cd scripts & git clone https://github.com/cocodataset/cocoapi.git`
`cd scripts/cocoapi & python PythonAPI/setup.py build_ext install`

## Installations requirements for cocodataset & maskrcnn_node.py
`pip install -r requirements.txt`

## Launch LuggageBot - Robot with cylinder [1]
`roslaunch launchFile/luggageBot_cylinder.launch`

## New Terminal: Run green cylinder follower node
`python3.6 scripts/follow_cylinder.py`
`Then you will be able to see the  Following Green Cylinder! [OR] Moving back! in terminal`

## New Terminal: To animate the green cylinder to move randomly
`python3.6 scripts/animate_cylinder.py`
`The cylinder will animate and luggageBot will follow it`

## New Terminal: Rviz view
`rviz`
`This command will give you camera view before and after image segmentation`
`Inside rviz: Bottom left cornor Add -> by_topic -> camera [OR] camera_result`

## Launch LuggageBot - Robot following person [2]
`roslaunch launchFile/luggageBot_person.launch`

## New Terminal: Run person follower node
`python3.6 scripts/follow_person.py`
`This script might take some time to run`
`After the script is executed, following statement will be printed in the terminal: Person not detected! [OR] Received Histogram!`
`After the person is detected, Then you will be able to see the Following person! [OR] Moving back! in terminal`

## New Terminal: To animate the user to move randomly
`python3.6 scripts/animate_person.py`
`The person/human will animate and luggageBot will follow it`
`Please use walking_person model under launchFile/model. Other model might have missing mesh/texture file that might cause it behave unexpectedly`

## New Terminal: Rviz view
`rviz`
`This command will give you camera view before and after image segmentation`
`Inside rviz: Bottom left cornor Add -> by_topic -> camera [OR] camera_result`

## Histogram [3]
`Histogram is only saved only for person following version because saving histogram for green cylinder does not make sense.`
`After the excution of follow_person.py script is stopped (press Ctrl+C ), user_histogram.png is saved in scripts folder with the message Histogram saved! in terminal.`