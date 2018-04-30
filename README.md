# README #

This README would normally document whatever steps are necessary to get your application up and running.

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

* For __ROS__ installation, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu
* For __ROS__ workspace configuration, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu

* For the next section, we admit you have install ROS Lunar and configure a workspace using catkin_make
* The workspace directory will be :

__/home/johndoe/catkin_workspace__

* For __hieroglyph__ installation, please go : https://git.instar-robotics.com/software/NeuralNetwork/hieroglyph

### Install Kheops ###
* clone the repository in your catkin workspace :

__cd /home/johndoe/catkin_workspace/src__

__git clone http://wiki.ros.org/lunar/Installation/Ubuntu__

* go to your root catkin workspace :

__cd /home/johndoe/catkin_workspace__

* And run catkin_make

### Help menu ###
* ./kheops -h 

### Run Kheops ###
* ./kheops -s xml_path 

### User function exampe ###

* Building demofct by hand :
mkdir lib
g++ -c user_src/demofct.cpp -o user_src/demofct.o -I include -I user_src -I /usr/include/eigen3 -fpic -std=c++17
gcc -shared -o  lib/demofct.so  user_src/demofct.o

