# README #

## I- Installation ##

### Dependancy ###
* ubuntu 16.04
* libboost 1.58
* libboost BGL 1.58
* libxerces 3.1
* libeigen 3.3
* ROS Lunar 
* hieroglyph

### Install dependancies ###
* On ubuntu 16.04 : 

__apt-get update__

__apt-get install libeigen3-dev libxerces-c-dev libxerces-c3.1 libboost-graph-dev libboost-graph1.58.0 libboost-all-dev libboost-all-dev libboost1.58-dev__

* For __ROS Lunar__ installation, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu
* For __ROS Lunar__ workspace configuration, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu

* For the next section, we admit you have install __ROS Luna__r and configure a workspace using catkin_make
* The workspace directory will be :

__/home/johndoe/catkin_workspace__

* For __hieroglyph__ installation, please go to : https://git.instar-robotics.com/software/NeuralNetwork/hieroglyph
* For the next step, we assume you have __hieroglyph__ install in your catkin\_workspace (home/johndoe/catkin\_workspace/src/hieroglyph)

### Install Kheops ###
* clone the repository in your catkin workspace :

__cd /home/johndoe/catkin_workspace/src__

__git clone http://wiki.ros.org/lunar/Installation/Ubuntu__

* go to your root catkin workspace :

__cd /home/johndoe/catkin_workspace__

* And run __catkin_make__

### Building User functions libraries

* For example, we create user_src directory with a demofct inside 
* Users who want to build could reuse the example

* To build __demofct__, go to the demofct directory :

__cd /home/johndoe/catkin_workspace/src/kheops/user_src/demofct__

* then, run the classical cmake/make command : 

__cmake \.__

__make__

* After the compilation, you get a __libdeomfct.so__
* You can copy this file in your library directory and run kheops with -l option (see __Run__ section)

## Run kheops ##

### Run Kheops : quick version ###

* Kheops is builded on top of ROS system. (In futur version, kheops will support other bus like dbus/yarp) 
* Kheops need roscore to run properly. 
* Launch roscore with :

__roscore__

* To run kheops you can use __rosrun__ command :

__rosrun kheops kheops -s path-to-script-file__

* print Help menu 

__rosrun kheops kheops -h__

* Launch script and load weight from weight_file to the neural network

__rosrun kheops kheops -s path-to-script-file -w path-to-weight-file__

* By default, kheops start in __pause__ mode
* To start kheops in __resume__ mode, run : 

__rosrun kheops kheops -r__

### Load user functions libraries ###

* Kheops is build to be modular. This means you can load function at the runtime like import lib in python.
* When you launch __kheops__ you can specify the directory where __kheops__ will search the library : 

__rosrun kheops kheops -l /path/to/lib/directory__

### Information about kheops services/messages ### 

* All messages and services are defined in the __hieroglyph project__
* To list __hieroglyph__ messages , run : 

__rosmsg package hieroglyph__

* To list __hieroglyph__ services, run :

__rossrv package hieroglyph__

### Name convention ###

### Command kheops with rosservice ### 

* We assume you launch a kheops script with rosrun 


