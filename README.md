# Prerequisites

* Install the MTi USB Serial Driver
  ```sh
  $ git clone https://github.com/xsens/xsens_mt.git
  $ cd ~/xsens_mt
  $ make
  $ sudo modprobe usbserial
  $ sudo insmod ./xsens_mt.ko
  ```

* Install gps_common
  ```sh
  $ sudo apt-get install ros-distro-gps-common
  ```

# Running the Xsens MTi ROS Node
1. Copy the contents of the src folder into your catkin workspace 'src' folder.
   Make sure the permissions are set to _o+rw_ on your files and directories.
   For details on creating a catkin workspace environment refer to [Creating a catkin ws](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)

2. in your catkin_ws ($CATKIN) folder, execute
   ```sh
   $ catkin_make
   ```

3. Source the environment for each terminal you work in. If necessary, add the
   line to your .bashrc
   ```sh
   . $CATKIN/devel/setup.bash
   ```

4. Initiate the ros core
   ```sh
   $ roscore
   ```

5. Open a new terminal, type
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rosrun xsens_driver mtdevice.py -m $sm -f $fs # publish sensor data
   ```
   where $fs can be 1,5,10,20,40,50,80,100,200 or 400Hz. This configures the MTi
   to output inertial data and magnetometer data at the set ODR. The maximum
   supported inertial update rate is 400Hz and for the magnetometer it is 100Hz.
   The $sm can be set to 1,2 or 3. This can be used to set the sensor to output
   sensor data or filter outputs.

6. To run the node
   ```sh
   $ rosrun xsens_driver mtnode.py _device:=/dev/ttyUSB0 _baudrate:=115200
   ```
   or
   ```sh
   $ rosrun xsens_driver mtnode.py 
   ```

7. Open a new terminal (do not forget step 3)
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rostopic echo /mti/sensor/sample
   ```
   or
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rostopic echo /mti/sensor/imu
   ```
