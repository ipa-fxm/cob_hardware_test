# cob4 test-rigs

## Contents

- [Umgebungsvariablen](#env_var)
- [ElmoConsole](#elmo_console)

Teststände
- [FDM Teststand](#fdm_test)
- [Multi Joint Teststand](#comp_test)
- [Single Joint Teststand](#single_test)
- [Hand Teststand](#hand_test)
- [Cam3d Teststand](#cam3d_test)


## Umgebungsvariablen <a name="env_var"></a>

### `ROS_MASTER_URI` exportieren

 - auf den gewünschten Roboter cob4-<roboter_nummer> setzen:  
    `export ROS_MASTER_URI=http://10.4.<roboter_nummer>.11:11311`

### `ROS_IP` exportieren

 - eigene IP Adresse rausfinden, z.B. eth0 aus:  
    `ifconfig`

 - eigene IP Adresse setzen:  
    `export ROS_IP=<ip_address>`
    
## ElmoConsole <a name="elmo_console"></a>

 Achtung: ElmoConsole kann nicht zusammen mit ROS-Treiber verwendet werden.  
 Weitere Infos zu ElmoConsole gibt es unter https://github.com/mojin-robotics/cob4/blob/groovy_dev/ELMO_adjust_offset_remote.md  

 - ElmoConsole ausführen:  
   `rosrun canopen_test_utils canopen_elmo_console <can-device> <can-ID>`
   
   (Bsp.: `rosrun canopen_test_utils canopen_elmo_console can0 1`)

--------------------------------------------
## Teststände

### FDM Teststand <a name="fdm_test"></a>

 - FDM auf Vorrichtung fixieren und auf korrekten Anschluss der Versorgungsleitungen prüfen.  
   Violette Leitungen = 48 V; blaue Leitungen = 24 V.

 - Start bringup:  
    `roslaunch cob_test_rigs fdm.launch can_id_steer:=3 can_id_drive:=4`

 - FDM initialisieren/recovern (in neuem Terminal):  
    `rosservice call /fdm/driver/[init/recover]`

 - Testscript (in neuem Terminal):  
    `rosrun cob_test_rigs test_fdm.py`

 - Wichtig: Auf korrekte Kommutierung/Drehrichtung der einzelnen Motoren achten!

--------------------------------------------

### Multi Joint Teststand <a name="comp_test"></a>

Hiermit können mehrere Achsen getestet werden.

#### Variablen
 Die folgenden Variablen müssen passend zum aktuellen Teststand gewählt werden.  
 `CAN_DEVICE` ist abhängig vom mapping des USB-Dongles in Linux (`ifconfig`).  
 `COMPONENT` ist die zu testende Hardwarekomponente.  
 Achtung: FDM und Basis können nicht mit dieser Methode getested werden.  

 - `CAN_DEVICE`: e.g. `can0`, `can1`,...

 - `COMPONENT`: e.g. `torso2` (2DoF), `head3` (3DoF), `sensorring`,...  

#### Starten
 - Start bringup:  
    `roslaunch cob_test_rigs COMPONENT.launch [can_device:=can1]`

 - Initialisieren/Recovern:  
    `rosservice call /CAN_DEVICE/COMPONENT/driver/[init/recover]`

 - Testscript starten:  
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
    >                        Number of repetitions for each test cycle, default: 5
    >
    >  -v DEFAULT_VEL, --default_vel=DEFAULT_VEL
    >                        Overwrite default velocity of component, default: 0.2
    >
    >  -a DEFAULT_ACC, --default_acc=DEFAULT_ACC
    >                        Overwrite default acceleration of component, default: 1.0

 - Start `rqt`:  
    `rqt __ns:=CAN_DEVICE`
    
    Plugins of Interest: 
    - `Controller Manager`: Plugins > Robot Tools > Controller Manager
    - `Joint Trajectory Controller` (Slider) : Plugins > Robot Tools > Joint Trajectory Controller
    - `Diagnostics Viewer`: Plugins > Robot Tools > Diagnostics Viewer
    - `Plot`: Plugins > Visualization > Plot

 - Start `cob_console`:  
    `rosrun cob_script_server cob_console`

 - Usage `cob_console`:  
    `sss.move("CAN_DEVICE/COMPONENT","test/CONFIG")`  
    `sss.init("CAN_DEVICE/COMPONENT")`

 - Exit `cob_console`:  
    `STRG + D`, then `ENTER`

 - Verfügbare configs/poses:  
    `rosparam get /script_server/CAN_DEVICE/COMPONENT`

--------------------------------------------

### Single Joint Teststand <a name="single_test"></a>

joints can only be moved one at a time.

 - Start bringup:  
    `roslaunch cob_test_rigs single_[elmo/schunk].launch can_device:=can0 can_id:=XX`
    
    > can_ids:
    > - torso: 31, 32, 33
    > - head: 70, 71, 72
    > - sensorring: 73
    > - arm: 61, 62, 63, 64, 65, 66, 67

 - Initialisieren/Recovern:  
    `rosservice call /single_[elmo/schunk]/driver/[init/recover]`

 - Start `rqt` (siehe auch Info oben):  
    `rqt`

##### Test JointTrajectoryController:
 1. start controller (in `rqt` window)
    - go to tab `ControllerManager`
    - add `joint_trajectory_controller` from drop-down menu
    - right-click on controller name -> press start (needs to be "running" afterwards)
   
 2. move single joint
    - go to tab `JointTrajectoryController`
    - select `joint_trajectory_controller` from drop down
    - press red button to activate slider (button turns green)
    - move the slider or enter desired joint position [rad] directly

##### Test JointPositionController/JointVelocityController

 `namespace`: `single_[elmo/schunk]`  
 `controller_name`: `single_joint_[position/velocity]_controller`

 1. start controller (in `rqt` window)
    - go to tab `ControllerManager`
    - add `controller_name` (s.o.) from drop-down menu
    - righ-click on respective controller -> press start (needs to be "running" afterwards)
    - make sure no other "commanding" controller is running at the same time ("joint_state_controller" can be "running")
   
 2. command sine function (`y = a*sin(b*(x-c))+d`)  
    - `rosrun cob_test_rigs test_single_sine.py -a 0.1 -b 0.1 -c 0.0 -d 0.0 -n 0.0 __ns:=<namespace>/<controller_name>`  
    
      > `a`: amplitude  
      > `b`: frequency  
      > `c`: phase  
      > `d`: offset  
      > `n`: repetitions  
      > 
      > set parameters a, b, c, d, n with caution!!!

--------------------------------------------

### Hand Teststand <a name="hand_test"></a>

 1. Configure router to hands network address (`10.4.x.1`)
 2. Configure router's WLAN to hands WLAN (`cob4-x-direct`)
 3. Connect hand to power
 4. Connect laptop with router's LAN interface
 5. On laptop: 
     - `sudo ip addr add 10.4.x.11/24 broadcast 10.4.x.255 dev eth0`
     - `export ROS_IP=10.4.x.11`
     - `roslaunch cob_default_robot_config upload_param.launch robot:=cob4-x`
     - `roslaunch cob_hardware_config upload_robot.launch robot:=cob4-16`
     - `roslaunch cob_bringup dashboard.launch robot:=cob4-16 robot_env:=ipa-apartment`
 6. On laptop, init, recover and move via dashboard

--------------------------------------------

### Cam3d Teststand <a name="cam3d_test"></a>

#### Intel Realsense D435

 1. Connect the camera to a USB3.0 port
 2. Query the serial number of the camera
    `rs-enumerate-devices |grep Serial`
 3. Start the camera driver
    `roslaunch mojin_bringup cam3d_d435_rgbd.launch robot:=jan name:=sensorring_cam3d serial_no:=SERIALNUMBER`
 4. Verify Image and PointCloud2 in rviz
    `roslaunch cob_test_rigs cam3d_d435_rviz.launch`
