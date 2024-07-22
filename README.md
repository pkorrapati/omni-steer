# Instructions about installing the SDK and ROS 1 Packages to interface with Omni 

## References

https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US 

https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/UserGuide/Omni_Device_Guide.pdf 

https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/OpenHaptics_ProgGuide.pdf 

https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/OpenHaptics_RefGuide.pdf 

https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/Installation+Instructions.pdf 

https://www.researchgate.net/figure/SensAble-PHANToM-OMNIR-haptic-device_fig2_273151335 

https://github.com/jhu-cisst/cisst/tree/main 

https://github.com/jhu-saw/sawSensablePhantom 

https://github.com/jhu-saw/sawOpenIGTLink 

https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9324680/ 

## Step 1. Install Open Haptics SDK 

Download Link: 

https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US 

or

```console 
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz 
```


Procedure: 

Dependencies:

```console  
sudo apt install libncurses5-dev freeglut3 build-essential 
```
 

Installation: 

```console 
tar -xf openhaptics_3.4-0-developer-edition-amd64.tar.gz 

cd openhaptics<press tab> 

sudo ./install 
``` 

Note: Files are installed to: /opt/OpenHaptics/Developer/3.4-0/ 

## Step 2. Install Omni Drivers 

Download Link: 

https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2023_11_15.tgz  

or 

```console
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2023_11_15.tgz 

sudo apt install udev 

tar -xf TouchDriver_2023_11_15.tgz 

cd TouchDriver_2023_11_15 

sudo ./install_haptic_driver 

sudo ./ListCOMPortHapticDevices 

sudo chmod 777 /dev/ttyACM0 

sudo mkdir /usr/share/3DSystems 

export GTDD_HOME=/usr/share/3DSystems 

cd bin 

Touch_Setup 

Touch_AdvancedConfig 

Touch_Diagnostic 
``` 

Note: .so file and udev rules are added to the appropriate location by the install file 

 

Enable Access to the Port 

```console
sudo chmod 777 /dev/ttyACM0 
```
 

Note: If ROS is being run on docker, use --privileged to provide access to peripherals on the host computer 

 

Or use –device=/dev/ttyACM0 

 
## Step 3. Get the git package for Omni [ on ROS machine] 

Dependencies:

```console
sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev swig libncurses5 

sudo apt install python3-wstool 

sudo apt install python3-catkin-tools python3-osrf-pycommon 
```

Scripts in the ROS package reference python instead of python3 

This is causing compilation of python libraries to fail 

Create a softlink python that points to python3: 

$ ln -s /usr/bin/python3 /usr/bin/python 
 

Create folder to clone git into: 

```console
cd ~/catkin_ws 

wstool init src  

catkin init  

catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release  

cd src  

touch config.rosinstall 

gedit config.rosinstall 
```
 

[See ref https://github.com/jhu-saw/sawSensablePhantom/raw/main/ros/sensable_phantom.rosinstall ]  

Paste the following: 

- git: 

    local-name: cisst-saw/cisstNetlib 

    uri: https://github.com/jhu-cisst/cisstNetlib 

    version: main 

- git: 

    local-name: cisst-saw/cisst 

    uri: https://github.com/jhu-cisst/cisst 

    version: 1.2.1 

- git: 

    local-name: crtk/crtk_msgs 

    uri: https://github.com/collaborative-robotics/crtk_msgs 

    version: 1.1.0 

- git: 

    local-name: crtk/crtk_python_client 

    uri: https://github.com/collaborative-robotics/crtk_python_client 

    version: 1.0.0 

- git: 

    local-name: cisst-saw/cisst-ros 

    uri: https://github.com/jhu-cisst/cisst-ros 

    version: 2.2.0 

- git: 

    local-name: cisst-saw/sawSensablePhantom 

    uri: https://github.com/jhu-saw/sawSensablePhantom 

    version: 1.0.0 

 

$ wstool merge config.rosinstall 

$ wstool up  

$ cd .. 

 

Note: in the current version, #include <ros/ros.h> is missing in a file 

 

$ gedit src/cisst-saw/sawSensablePhantom/ros/src/sensable_phantom.cpp 

Add the following line if it doesn't already exist:  

#include <ros/ros.h> 

Save and close 

 

$ catkin build 

$ source devel/setup.bash 

 

 

sudo cp usr/lib/libPhantomIOLib42.so /usr/lib/libPhantomIOLib42.so 

  

sudo cp rules.d/*.rules /etc/udev/rules.d/ 

  

sudo udevadm control --reload 