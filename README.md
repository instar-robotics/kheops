![Alt text](icon/kheops_icon.png?raw=true "Title")

# README #

## I Description ##

* Kheops is a Neural Network and Dynamical function simulator
  1. A neural network is reprented by a graph strucutre described in an XML file called a neural script
  2. Each neurons layer or group are graph vertex and are connected to other layer by link (edge of the graph).
  3. The weight of the neurons are contains into the link.
  4. And neural activities are propagated accros the link.
  5. Connection could be feedforward or recurent  
  6. A neural layer or group is called a function. 
  7. Each function is executed by a separte thread called a Runner

* Kheops uses dataflow paradigm to process the neural network :
  1. A neural script had a defined period 
  2. At the begenning of each period, kheops launch a wave into the graph
  3. A function can be executed only if all this input link are available or if the function doesn't havce anny input (first layer)
  4. Each function is exectued by his runner and spread the wave accros output link.
  5. When the last layer is executed, kheops sleeps until the new period.

* Kheops is designed to be used in online environnement, mainly to design robotics controller.

* Kheops are based on [ROS](http://www.ros.org/):
  1. scripts communicate with one another through ROS topics   
  2. live visualization is achieved by having functions publish their outputs in a ROS topic  
  3. ROS services are used to control scripts execution

* Kheops should be use in interaction with [Papyrus](https://github.com/instar-robotics/papyrus) GUI to create, edit and debug neural script 


## II- Installation ##

Kheops is a part of the Kheops/Papyrus software containing 4 components : Kheops, [Papyrus](https://github.com/instar-robotics/papyrus), [Alexandria](https://github.com/instar-robotics/alexandria) and [Hieroglyph](https://github.com/instar-robotics/hieroglyph)

Please see [Papyrus's how-to-install](https://github.com/instar-robotics/papyrus/blob/master/README.org#how-to-install) tutorial for a Kheops/Papyrus full installation.

or See below for a Kheops standlone installation.


## III- Standalone Installation ##

Standalone installation is useful to install Kheops without Papyrus GUI on devices like mobile robots.

### Dependancy ###
* cmake 3.5
* libboost (BGL, SYSTEM, FILESYSTEM, SERIALIZATION) at least 1.58
* libeigen 3.3
* ROS Melodic 

### Install dependancies ###
* Installation has been tested on __ubuntu 18.04__  
* For __ROS Melodic__ installation and workspace configuration, please refer to : http://wiki.ros.org/melodic/Installation/Ubuntu or to [Papyrus's how-to-install](https://github.com/instar-robotics/papyrus/blob/master/README.org#how-to-install) tutorial.

* Kheops dependancies : 

```console
$> apt-get update
$> apt-get install cmake libeigen3-dev libboost-graph-dev libboost-graph1.65.1 libboost-all-dev libboost-all-dev libboost1.65-dev ros-melodic-diagnostics
```

* Alexandria dependancies : 

```console
$> apt-get install ros-melodic-joy ros-melodic-tf2 ros-melodic-nav-msgs
```

### Install Kheops and Alexandria ###

* For the next section, we admit you :
  1. have install __ROS__ and configure a workspace using __catkin_make__
  2. Kheops and Alexandria dependancies are installed

* To fetch the sources:

```console
$> cd ~/CATKIN_WS/PATH/src
$> git clone https://github.com/instar-robotics/kheops.git
$> git clone https://github.com/instar-robotics/alexandria.git
$> git clone https://github.com/instar-robotics/hieroglyph.git
```

* Then we build the whole workspace:
```console
$> cd ~/CATKIN_WS/PATH/
$> catkin_make
```

## III Run kheops in standalone mode ##

Note : For beginners, we recommand to start with the full installation and run Kheops using Papyrus GUI ! 

### Run Kheops : quick version ###

* Kheops is builded on top of ROS system.
* Kheops need roscore to run properly. 
* Launch roscore with :

```console
$> roscore
```
* print Kheops help menu 

```console
$> rosrun kheops kheops -h
```

* To run kheops you can use __rosrun__ command :

```console
$> rosrun kheops kheops -s MyScript.xml  -l path/to/lib/alexandria/
```

(See [Alexandria](https://github.com/instar-robotics/alexandria/blob/master/README.md#libraries-path) to get alexandria libraries path)

* if you want launch Kheops without __-l__ option, you can set __KHEOPS_LIB_PATH__ variable into your bashrc : 

```console
$> echo "export KHEOPS_LIB_PATH=\"path/to/lib/alexandria/\"" >> ~/.bashrc
$> source ~/.bashrc
```

* then just run 

```console
$> rosrun kheops kheops -s MyScript.xml 
```

* Launch script and load weight from weight_file to the neural network

```console
$> rosrun kheops kheops -s MyScript.xml -w path-to-weight-file
```

* By default, kheops start in __run__ mode
* To start kheops in __pause__ mode, run : 

```console
$> rosrun kheops kheops -p
```

### Kheops services and messages ###

* All messages and services are defined in the __hieroglyph project__
* To list __hieroglyph__ messages , run : 

```console
$> rosmsg package hieroglyph
```

* To list __hieroglyph__ services, run :

```console
$> rossrv package hieroglyph
```

* __rosmsg__ and __rossrv__ command can print details of messages and services. See ROS wiki for details

### Name convention ###

* The ROS name convention are defined here : http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources
* We follow the same convention with adding the following rule : 
  1. Each node have "__kheops__" as prefix 
  2. We had the neural script name as suffix
  3. We use __underscore__ (\_) as separator

* For example, a script name __"action.script"__ will have __"kheops_action"__ as node name

* To list the ROS node, use rosnode command :  

**_rosnode list_**

### Command kheops with rosservice

* We assume you launch a kheops script "__action.script__" with rosrun, like this : 

**_rosrun kheops kheops -s action.script_**

* At launch, kheops register some services : 
  1. help : print help message, list of services and arguments
  2. control : basic command, "resume", "pause", "quit", "status"
  3. oscillo : create oscillo rostopic
  4. output : create data rostopic for both links and functions
  5. objects : print list of the objects (functions, links, inputs, rt\_token)
  6. weight : load/save neural network weight
  7. rt\_token : create data topic for rt\_token
  8. rt\_stat : get rt\_token stat


* To run services, you can use ROS command __rosservice__ 
* To list services : 

**_rosservice list_**

* To call a service : 

**_rosservice call /node\_name/service\_name  args1 args2 ... argsN_**

* For example, to call help service for action srcipt: 

**_rosservice call /kheops\_action/help_**

#### Control service ####

* Control service add 3 arguments : 
  1. "quit" : stop the program
  2. "resume" : start node execution
  3. "pause" : stop node execution until resave a "quit" or "resume" command

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
  1. "all" : provide all the object
  2. "rt\_token" : provide rt\_token UUID
  3. "functions" : provide functions UUID
  4. "links" : provide links UUID
  5. "inputs" : provide inputs UUID

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

* To stream the contents of a rostopic, run :

**_rostopic echo /kheops\_script\_name/topic\_name_**

#### Output Topic ####

* Data strucutre inside kheops functions and links are strongly typed. They could be Scalar (double) or Matrix (Using Eigen Matrix)
* So, each function or link could output a Scalar Rostopic or a Matrix Rostopic depand of the type of the object.

* Rostopic output are created by default using XML script file (See Script developpers guide) or using the output service 
* When a topic is created, the name conventions are :
  1. The prefix is the type of the object (link or function)
  2. The suffix is the UUID of the object
  3. An underscore is used as a separator

* For example, the output of the link {64a9913c-ac60-4b2a-bbd6-f0c868190b3f} from the action script will be :

**_/kheops\_action/link_{64a9913c-ac60-4b2a-bbd6-f0c868190b3f}_** 

* Scalar topic are simply a ros standard message Float64 
* Matrix topic are a ros standard message Float64MultipleArray

#### Oscillo and Rt_Token Topic ####

* Oscillo and rt\_token topic are created using services.
* This two topîcs are useful for debug information.

* rt\_token topîcs uses __OscilloData__ message defined in hieroglyph.
* __OscilloData__ contains following information :
  1. string uuid 
  2. float64 period : theoric period of the script
  3. float64 means : average period 
  4. float64 sleep : time spend to sleep
  5. float64 duration : execution time
  6. float64 start : date when last run was started
  7. bool warning : real time constraint wasn't respected (duration > period)

* Oscillo topics uses __OscilloArray__ message defined in hieroglyph.
* An __OscilloArray__ message is an array of __OscilloData__ message
* One __OscilloData__ message for each function in the script 

## V Kernel developpers guide ##

* TODO

* Kernel Object : 
1. Runner : 
2. FRunner : 
3. RtToken : 
4. kLink : 
5. iLink (and iLinkBase) : 
6. Input (and inputBase) : 
7. Function : 

* Object : link, input an Function

* Using Input and iLink 

* Sparse Matrix : Connections is define is a Sparse Matrix Filter and we can generate every topology

* Matrix_Matrix have 3 types of connections
  1. One to All connections (ONE_TO_ALL) : Dense connections between input and output 
  2. One to One connections (ONE_TO_ONE) : Sparse conenction between input and output 
  3. One to Neighborhood connections (ONE_TO_NEI) : Sparse conenction between input and output 

* IScalar
* ISMatrix
* IMMatrix
1. Weight
2. Filter

* UUID definition

## VI Functions developpers guide ##

* Main information are provided in the alexandria software description. 

## VII Neurals developpers guide ##

* Main information are provided in the papyrus software description. 






  


