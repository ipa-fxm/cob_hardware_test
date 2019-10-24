cob_devices_test
=================

## Content

- [Base](#base)
- [Torso](#torso)
- [Head](#head)
- [NUC](#nuc)


## BASE <a name="base"></a>

1. Connect the USB-Hub (for laser scanner, light, joystick) and CAN-Dongle (for bms) to the computer via two separate USB cables. 
2. Setup udev rules
   ```
   su robot
   sudo /home/robot/git/robot_ws/src/cob_hardware_test/cob_devices_test/scripts/udev_joy.sh
   sudo /home/robot/git/robot_ws/src/cob_hardware_test/cob_devices_test/scripts/udev_s300.sh
   ```
3. Start launchfile in a new window
   ```
   roslaunch cob_devices_test base.launch flexisoft_host:=XX.XX.XX.99
   ```

### Light:
 - [ ] The LEDs of the wheel covers will light up one-by-one (CIRCLE_COLOR) red-green-blue.
Check all wheel-colors.

### Laserscanner:
 - [ ] Select **Base_Test** in RVIZ, the laserscanner points should be visible in RVIZ (front-green, left-white, right-yellow).
Check the correct assignment.

### Fall-Sensors:
 - [ ] Test Fall-Sensors with magnet.

### BMS:
 - [ ] Check diagnostics in the `rqt_robot_monitor`

### Joystick:
 - [ ] Check diagnostics in the `rqt_robot_monitor` and/or execute `rostopic echo /joy` and use the joystick.


## TORSO  <a name="torso"></a>

1. Connect the light to the computer
2. Start launchfile
   ```
   roslaunch cob_devices_test torso.launch
   ```

### Light:
 - [ ] The LEDs of the LED ring will light up one-by-one (CIRCLE_COLOR) red-green-blue.


## HEAD  <a name="head"></a>
1. Connect the head to the power supply (first, connect before switching on the supply).
2. Connect the computer to the head display
3. Connect the camera, touchscreen and soundcard to the computer.
4. Start launchfile
   ```
   roslaunch cob_devices_test head.launch
   ```

### Camera:
 - [ ] Select **Head_Test** in RVIZ, the Camera Image of the head camera should be visible

### Sound: 
 - [ ] Select the correct Input and Output devices **(Sound Blaster Play! 2)** at the Sound Settings (set Input Volume to 100%)
and record sound with ```arecord -C test.wav```
 - [ ] See if sound has been recorded successfully with ```aplay test.wav```
 - [ ] Test Speakers with **Test Speakers** within the Sound Settings (check if left and right is correct

### Touchscreen:
 - [ ] Touch the display of the robot

## Intel NUC  <a name="nuc"></a>

### Temperature Test

1. Run the Test:  
   ```
   rosrun cob_devices_test run_thermal_test.py
   ``` 
   
   The results of the test where written in an *.tar.gz File where the skript was started.  
   optional arguments are possible and show with argument `--help`  
   
    >full signature:   
    >`rosrun cob_devices_test run_thermal_test.py [-h] [--outputdir OUTPUTDIR] [--testtime TESTTIME]`  

2. Generate PNG-Image for the test-results:
   ```
   rosrun cob_devices_test plot_thermal_test.py some_result_file.tar.gz
   ```

    >full signature:  
    >`rosrun cob_devices_test plot_thermal_test.py [--help] [--show] [--show-only] [--pdf] RESULT_FILE_1 [RESULT_FILE_N]` 

Alternative method via USB live stick:

1. preparations:
   - connect service cart to PC (Display, USB-hub with live stick and power supply)
   - power on the PC
   - press F10 and choose the usb live stick
   - choose persistent live
   - make a directory on the live system usbdata partition (e.g. thermal-test-cob4-XX)
   - copy the `plot_thermal_test.py` and `run_thermal_test.py` to the created directory

1. Run the test:
   - open a terminal and navigate to the created directory
   - execute the test: `./run_thermal_test.py`

2. plot the data (creates a png image):
   - execute the plot file: `./plot_thermal_test.py thermal_test-file.tar.gz`
