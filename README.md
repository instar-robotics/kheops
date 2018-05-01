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

**__apt-get update__**

**__apt-get install libeigen3-dev libxerces-c-dev libxerces-c3.1 libboost-graph-dev libboost-graph1.58.0 libboost-all-dev libboost-all-dev libboost1.58-dev__**

* For __ROS Lunar__ installation, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu
* For __ROS Lunar__ workspace configuration, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu

* For the next section, we admit you have install __ROS Luna__r and configure a workspace using catkin_make
* The workspace directory will be :

**__/home/johndoe/catkin_workspace__**

* For __hieroglyph__ installation, please go to : https://git.instar-robotics.com/software/NeuralNetwork/hieroglyph
* For the next step, we assume you have __hieroglyph__ install in your catkin\_workspace (home/johndoe/catkin\_workspace/src/hieroglyph)

### Install Kheops ###
* clone the repository in your catkin workspace :

**__cd /home/johndoe/catkin_workspace/src__**

**__git clone http://wiki.ros.org/lunar/Installation/Ubuntu__**

* go to your root catkin workspace :

**__cd /home/johndoe/catkin_workspace__**

* And run **__catkin_make__**

### Building User functions libraries

* For example, we create user_src directory with a demofct inside 
* Users who want to build could reuse the example

* To build __demofct__, go to the demofct directory :

**__cd /home/johndoe/catkin_workspace/src/kheops/user_src/demofct__**

* then, run the classical cmake/make command : 

**__cmake \.__**

**__make__**

* After the compilation, you get a __libdeomfct.so__
* You can copy this file in your library directory and run kheops with -l option (see __Run__ section)

## Run kheops ##

### Run Kheops : quick version ###

* Kheops is builded on top of ROS system. (In futur version, kheops will support other bus like dbus/yarp) 
* Kheops need roscore to run properly. 
* Launch roscore with :

**__roscore__**

* To run kheops you can use __rosrun__ command :

**__rosrun kheops kheops -s path-to-script-file__**

* print Help menu 

**__rosrun kheops kheops -h__**

* Launch script and load weight from weight_file to the neural network

**__rosrun kheops kheops -s path-to-script-file -w path-to-weight-file__**

* By default, kheops start in __pause__ mode
* To start kheops in __resume__ mode, run : 

**__rosrun kheops kheops -r__**

### Load user functions libraries ###

* Kheops is build to be modular. This means you can load function at the runtime like import lib in python.
* When you launch __kheops__ you can specify the directory where __kheops__ will search the library : 

**__rosrun kheops kheops -l /path/to/lib/directory__**

### Information about kheops services/messages ### 

* All messages and services are defined in the __hieroglyph project__
* To list __hieroglyph__ messages , run : 

**__rosmsg package hieroglyph__**

* To list __hieroglyph__ services, run :

**__rossrv package hieroglyph__**

* __rosmsg__ and __rossrv__ command can print details of messages and services

### Name convention ###

* The ROS name convention are defined here : http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources
* We follow the same convention with adding the following rule : 
..1. Each node have "__kheops__" as prefix 
..2. We had the neural script name as suffix
..3. We use __underscore__ (\_) as separator

* For example, a kheops script name "action.script" will have "kheops_action" as node name

* To list the ROS node, use rosnode command :  

**__rosnode list__**

### Command kheops with rosservice ### 

* We assume you launch a kheops script "__action.script__" with rosrun, like this : 

**__ rosrun kheops kheops -s action.script__**

* At launch, kheops register some services : 
..1. help : print help message, list of services and arguments
..2. control : basic command, "resume", "pause", "quit"
..3. oscillo : create oscillo rostopic
..4. output : create data rostopic for both links and functions
..5. objects : print list of the objects (functions, links, inputs, rt\_token)
..6. weight : load/save neural network weight
..7. rt\_token : create data topic for rt\_token
..8. rt\_stat : get rt\_token stat


* To run services, you can use ROS command __rosservice__ 
* To list services : 

**__rosservice list__**

* To call a service : 

**__rosservice call /node\_name/service\_name  args1 args2 ... argsN__**

* For example, to call help service for action srcipt: 

**__rosservice call /kheops\_action/help__**

#### Control service ####

* Control service add 3 arguments : 
..1. "quit" : stop the program
..2. "resume" : start node execution
..3. "pause" : stop node execution until resave a "quit" or "resume" command

* For example, to shutdown action script : 

**__rosservice call /kheops\_action/control quit__**

#### Oscillo service ####

* Start and stop the oscillo rostopic
* This topic contains all runners informations. For details about the topic, go to "rostopic section"

* For example, to create and publish on oscillo rostopic, run : 

**__rosservice call /kheops\_action/oscillo start__**

#### Objects service ###

* Each object (link, function, rt_token, input) are identified by an UUID
* The object service provide the list of all the object of a script with associated UUID
* The argument of the command are :
..1. "all" : provide all the object
..2. "rt\_token" : provide rt\_token UUID
..3. "functions" : provide functions UUID
..4. "links" : provide links UUID
..5. "inputs" : provide inputs UUID

* For example, to list all functions UUID, run :

**__rosservice call /kheops\_action/objects functions__**

#### Output service ####

* Start and stop output rostopic
* Each topic contains output for the desire object (link or function). For details about the topic, go to "rostopic section"

* For example, to create an output for object with UUID {70a19c5c-fc60-4275-bbd6-aac857190b3d} : 

**__rosservice call /kheops\_action/output start {70a19c5c-fc60-4275-bbd6-aac857190b3d}__**

#### Weight service ####

* To load, neural network weight, you can use -w option with file path
* But it can be useful to reload an other weight file at runtime or be able to save the weight before exit the script
* The weight service provide save/load command. 
* When save or load are executed, the script is automatically paused. 
* Then, the operation is executed. 
* After the operation, the script is automatically resumed 

* For example, to load a weight file : 

**__rosservice call /kheops/\weight load path\_to\_weight\_file__**

### Rostopic section ###
