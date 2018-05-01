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

**_apt-get update_**

**_apt-get install libeigen3-dev libxerces-c-dev libxerces-c3.1 libboost-graph-dev libboost-graph1.58.0 libboost-all-dev libboost-all-dev libboost1.58-dev_**

* For __ROS Lunar__ installation, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu
* For __ROS Lunar__ workspace configuration, please refer to : http://wiki.ros.org/lunar/Installation/Ubuntu

* For the next section, we admit you have install __ROS Luna__r and configure a workspace using catkin_make
* The workspace directory will be :

**_/home/johndoe/catkin_workspace_**

* For __hieroglyph__ installation, please go to : https://git.instar-robotics.com/software/NeuralNetwork/hieroglyph
* For the next step, we assume you have __hieroglyph__ install in your catkin\_workspace (home/johndoe/catkin\_workspace/src/hieroglyph)

### Install Kheops ###
* clone the repository in your catkin workspace :

**_cd /home/johndoe/catkin_workspace/src_**

**_git clone http://wiki.ros.org/lunar/Installation/Ubuntu_**

* go to your root catkin workspace :

**_cd /home/johndoe/catkin_workspace_**

* And run **_catkin_make_**

### Building User functions libraries

* For example, we create user_src directory with a demofct inside 
* Users who want to build could reuse the example

* To build __demofct__, go to the demofct directory :

**_cd /home/johndoe/catkin_workspace/src/kheops/user_src/demofct_**

* then, run the classical cmake/make command : 

**_cmake \._**

**_make_**

* After the compilation, you get a __libdeomfct.so__
* You can copy this file in your library directory and run kheops with -l option (see __Run__ section)

## Run kheops ##

### Run Kheops : quick version ###

* Kheops is builded on top of ROS system. (In futur version, kheops will support other bus like dbus/yarp) 
* Kheops need roscore to run properly. 
* Launch roscore with :

**_roscore_**

* To run kheops you can use __rosrun__ command :

**_rosrun kheops kheops -s path-to-script-file_**

* print Help menu 

**_rosrun kheops kheops -h_**

* Launch script and load weight from weight_file to the neural network

**_rosrun kheops kheops -s path-to-script-file -w path-to-weight-file_**

* By default, kheops start in __pause__ mode
* To start kheops in __resume__ mode, run : 

**_rosrun kheops kheops -r_**

### Load user functions libraries ###

* Kheops is build to be modular. This means you can load function at the runtime like import lib in python.
* When you launch __kheops__ you can specify the directory where __kheops__ will search the library : 

**_rosrun kheops kheops -l /path/to/lib/directory_**

### Kheops services and messages ###

* All messages and services are defined in the __hieroglyph project__
* To list __hieroglyph__ messages , run : 

**_rosmsg package hieroglyph_**

* To list __hieroglyph__ services, run :

**_rossrv package hieroglyph_**

* __rosmsg__ and __rossrv__ command can print details of messages and services

### Name convention ###

* The ROS name convention are defined here : http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources
* We follow the same convention with adding the following rule : 
..1. Each node have "__kheops__" as prefix 
..2. We had the neural script name as suffix
..3. We use __underscore__ (\_) as separator

* For example, a kheops script name "action.script" will have "kheops_action" as node name

* To list the ROS node, use rosnode command :  

**_rosnode list_**

### Command kheops with rosservice

* We assume you launch a kheops script "__action.script__" with rosrun, like this : 

**_rosrun kheops kheops -s action.script_**

1. At launch, kheops register some services : 
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

**_rosservice list_**

* To call a service : 

**_rosservice call /node\_name/service\_name  args1 args2 ... argsN_**

* For example, to call help service for action srcipt: 

**_rosservice call /kheops\_action/help_**

#### Control service ####

* Control service add 3 arguments : 
..1. "quit" : stop the program
..2. "resume" : start node execution
..3. "pause" : stop node execution until resave a "quit" or "resume" command

* For example, to shutdown action script : 

**_rosservice call /kheops\_action/control quit_**

#### Oscillo service ####

* Start and stop the oscillo rostopic
* This topic contains all runners informations. For details about the topic, go to "rostopic section"

* For example, to create and publish on oscillo rostopic, run : 

**_rosservice call /kheops\_action/oscillo start_**

#### Objects service ####

* Each object (link, function, rt_token, input) are identified by an UUID
* The object service provide the list of all the object of a script with associated UUID
* The argument of the command are :
..1. "all" : provide all the object
..2. "rt\_token" : provide rt\_token UUID
..3. "functions" : provide functions UUID
..4. "links" : provide links UUID
..5. "inputs" : provide inputs UUID

* For example, to list all functions UUID, run :

**_rosservice call /kheops\_action/objects functions_**

#### Output service ####

* Start and stop output rostopic
* Each topic contains output for the desire object (link or function). For details about the topic, go to "rostopic section"

* For example, to create an output for object with UUID {70a19c5c-fc60-4275-bbd6-aac857190b3d} : 

**_rosservice call /kheops\_action/output start {70a19c5c-fc60-4275-bbd6-aac857190b3d}_**

#### Weight service ####

* To load, neural network weight, you can use -w option with file path
* But it can be useful to reload an other weight file at runtime or be able to save the weight before exit the script
* The weight service provide save/load command. 
* When save or load are executed, the script is automatically paused. 
* Then, the operation is executed. 
* After the operation, the script is automatically resumed 

* For example, to load a weight file : 

**_rosservice call /kheops\_action/weight load path\_to\_weight\_file_**

#### Rt\_token service ####

* The oscillo rostopic contains data for all runners. This could be represent a huge amont of data.
* Some times, we just want monitor script perfomance without all the details
* THe rt\_token service can create a rostopic where only global performance are printed. 
* The argument of the service are simply start or stop.

* For example to create and publish into the rt\_token topic, run : 

**_rosservice call /kheops\_action/rt\_token start_**

#### Rt\_stat service ####

* Sometimes we just want to read instant information about the script without stream the information.
* The rt\_stat service provide instant information about the running script.

* For example, to read global information for action script, run : 

**__rosservice call /kheops\_action/rt\_stat_**

### Rostopic section ###

* kheops use rostopic to stream data accros script and/or to the IHM Papyrus
* Rostopic could be data send accros script or script management information send Papyrus 
* Each topic are stored in the script rosnode tree.
* For example, each topic of the action script will be inside : 

**_/kheops\_action/_**

* To display the list of topic, you can use __rostopic__ command :

**_rostopic list_**


#### Output Topic ####

* Data strucutre inside kheops functions and links are strongly typed. They could be Scalar (double) or Matrix (Using Eigen Matrix)
* So, each function or link could output a Scalar Rostopic or a Matrix Rostopic depand of the type of the object.

* Rostopic output are created by default using XML script file (See Script developpers guide) or using the output service 
* When a topic is created, the name conventions are :
..1. 


#### Oscillo and Rt_Token Topic ####


## kheops and functions developpers guide ##


## Neural Script developpers guide ##
