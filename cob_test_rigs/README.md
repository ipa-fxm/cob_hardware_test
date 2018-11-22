# ROS MASTER URI exportieren

auf den gewünschten Roboter cob4-<roboter_nummer> setzen: 

`export ROS_MASTER_URI=http://10.4.<roboter_nummer>.11:11311`

eigene IP Adresse aus eth0:

`ifconfig`

eigene IP Adresse setzen:

`export ROS_IP=<ip_address>`

# FDM Teststand
FDM auf Vorrichtung fixieren und auf korrekten Anschluss der Versorgungsleitungen prüfen. Violette Leitungen = 48 V; blaue Leitungen = 24 V.
Start bringup:
`roslaunch cob_test_rigs fdm.launch can_id_steer:=3 can_id_drive:=4`

Testscript:
`rosrun cob_test_rigs test_fdm.py`

# Elmo console

*Elmo console* ausführen:
`rosrun canopen_test_utils canopen_elmo_console <can-device> <can-ID>`

#Bsp.: `rosrun canopen_test_utils canopen_elmo_console can0 1`

Falls ROS die `canopen_test_utils` nicht findet, call:
`source ~/git/robot_ws/devel/setup.bash`

Um die FDMs zu initialisieren, call:
`rosservice call /fdm/driver/init`

Um die FDMs zu recovern, call:
`rosservice call /fdm/driver/recover`

--------------------------------------------

# Komponenten

Hiermit können mehrere Achsen getestet werden.

CAN_DEVICE: e.g. can0, can1,...

COMPONENT: e.g. torso2, torso3, head2,...(NO FDM)

Start bringup:
`roslaunch cob_test_rigs COMPONENT.launch [can_device:=can1]`

initialize:
`rosservice call /CAN_DEVICE/COMPONENT/driver/[init/recover]`

Testscript:
`rosrun cob_test_rigs test_components.py -c COMPONENT -d CAN_DEVICE -r 1 -v 0.4`

>Options:
>
>  -h, --help           show this help message and exit
>
>  -c COMPONENT, --component=COMPONENT
>                        Component that is going to be tested
>
>  -d CAN_DEVICE, --can_device=CAN_DEVICE
>                        CAN device the component is connected to
>
>  -r REPETITIONS, --reps=REPETITIONS
>                        Number of repetitions for each test cycle
>
>  -v DEFAULT_VEL, --default_vel=DEFAULT_VEL
>                        Overwrite default velocity of component

start rqt plugins (for slider):
`rqt __ns:=CAN_DEVICE`

start cob_console:
`rosrun cob_script_server cob_console`

use in cob_console:
`sss.move("CAN_DEVICE/COMPONENT","test/CONFIG")`
`sss.init("CAN_DEVICE/COMPONENT")`

exit cob_console:
STRG + D, then ENTER

get available configs/poses:
`rosparam get /script_server/CAN_DEVICE/COMPONENT`

# SINGLE JOINT TESTING

joints can only be moved one at a time.

Start bringup:
`roslaunch cob_test_rigs single_[elmo/schunk/nanotec].launch can_device:=can0 can_id:=XX`

can_ids:
 - torso: 31, 32, 33
 - head: 70, 71, 72
 - sensorring: 73
 - arm: 61, 62, 63, 64, 65, 66, 67

initialize/recover:
`rosservice call /single_[elmo/schunk/nanotec]/driver/[init/recover]`

start graphical tools:
`rqt`

Test JointTrajectoryController:
 1. start controller (in rqt window)
   - go to tab controller manager
   - add joint_trajectory_controller from drop-down menu
   - right-click on controller name -> press start (needs to be "running" afterwards)

 2. move single joint
   - go to tab joint trajectory controller
   - select joint_trajectory_controller from drop down
   - press red button to activate slider (button turns green)
   - move the slider or enter desired joint position [rad] directly

Test JointPositionController/JointVelocityController
 1. start controller (in rqt window)
   - go to tab controller manager
   - add [single_joint_position_controller/single_joint_velocity_controller] from drop-down menu
   - righ-click on respective controller -> press start (needs to be "running" afterwards)
   - make sure no other "commanding" controller is running at the same time ("joint_state_controller" can be "running")

 2. command sine function
   - rosrun cob_test_rigs test_single_sine.py -a 0.1 -b 0.1 -c 0.0 -d 0.0 -n 0.0 __ns:=single_[elmo/schunk/nanotec]/single_joint_[position/velocity]_controller
   - set parameters a, b, c, d, n with caution!!!
   - publishes a sine function of the form: `y=a*sin(b(x+c))+d`
   - `a`: amplitude
   - `b`: period
   - `c`: phase shift
   - `d`: vertical shift
   - adjust value of `__ns` according to loaded controller


