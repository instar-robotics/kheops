/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of kheops <https://github.com/instar-robotics/kheops>.
 
  kheops is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  kheops is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef __COM_INTERFACE_H__
#define __COM_INTERFACE_H__

#include <string>
#include <queue>
#include <vector>

/**************************  List of interfaces *******************************
*  1- help : list all the request supported by kheops
*  2- control : general command
*	a- resume 	
*       b- pause 
*	c- quit
*       d- status
*       e- path
*  3- weight : command relative to the weight file
*	a- save 'path'  (path is not mandatory, take the default weigth path)
*	b- load 'path'  (path is not mandatory, take the default weigth path)
*  4- rt_stat : get rt_token status 
*  5- oscillo : 
* 	a- start  (active oscillo)
*	b- stop   (stop oscillo)
*  6- output :
* 	a- start 'uuid'  (active output for uuid object )
*	b- stop  'uuid'  (stop output for uuid object )
*  7- objects :
*	a- all : get the list of uuid/type of all the objects
*	b- ilink : get list of uuid of ilink
*	c- input : get list of uuid of input 
*	d- function  get list of uuid of function
*	c- rt_token :  get list of uuid of rt_token
*  8- rt_token : 
*	a- start (active rt_token topic)
*	b- stop  (stop rt_token topic)
*  9- save_activity :
*  	a- start 'uuid' (start saving function's activity into SHM)
*  	b- stop 'uuid' (stop saving function's activity into SHM)
*  10- comment : 
*  	a- start 'uuid' (comment the function : stop to run compute and after_compute)  	
*  	b- stop 'uuid' (uncomment the function)
********************************************************************************/

const std::string RETURN[] = {"unknown command","unknown uuid"};
const std::string CMD[] = {"help","control","weight","rt_stat","output","oscillo","objects","rt_token","save_activity","comment"};

const int C_HELP=0; 
const int C_CONTROL=1; 
const int C_WEIGHT=2; 
const int C_RTSTAT=3; 
const int C_OUTPUT=4; 
const int C_OSCILLO=5; 
const int C_OBJECTS=6; 
const int C_RTTOKEN=7; 
const int C_ACTIVITY=8; 
const int C_COMMENT=9; 

const std::string CARG[] = {"resume","quit","pause","status","path" ,"save","load","start","stop","all","rt_token","function","input","ilink","activity"};

const int S_RESUME=0; 
const int S_QUIT=1; 
const int S_PAUSE=2; 
const int S_STATUS=3; 
const int S_PATH=4; 
const int S_SAVE=5;
const int S_LOAD=6; 
const int S_START=7; 
const int S_STOP=8;
const int S_ALL=9;
const int S_RTTOKEN=10;
const int S_FUNCTIONS=11;
const int S_INPUTS=12;
const int S_ILINKS=13;
const int S_ACTIVITY=14;


struct Request
{
	int id_cmd;
	int id_arg;
	std::vector<std::string> args;
};

class ComInterface
{
	protected : 

		std::string name;
		std::queue<Request> qrequest;

		static ComInterface* singleton;
		ComInterface(){}

	public : 

		virtual ~ComInterface(){}

		static ComInterface *getInstance(){return singleton;}

		static void init(int argc, char ** argv, std::string prog_name, std::string script_name){singleton->_init(argc, argv,prog_name,script_name);}
		static void setDefaultName(std::string& str) {return singleton->_setDefaultName(str);}
		static std::string getName() {return singleton->name;}

		virtual int enter() = 0;
		virtual void registerListener() = 0;
		virtual void _init(int argc, char ** argv, std::string prog_name, std::string script_name)=0;
		virtual void _setDefaultName(std::string& str) = 0;

		void exec_request();
};

#endif // __COM_INTERFACE_H__

