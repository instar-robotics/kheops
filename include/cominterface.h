/*
Copyright Instar Robotics

Author: Pierre Delarboulas

This software is governed by the CeCILL v2.1 license under French law and abiding by the rules of distribution of free software. 
You can use, modify and/ or redistribute the software under the terms of the CeCILL v2.1 license as circulated by CEA, CNRS and INRIA at the following URL "http://www.cecill.info".
As a counterpart to the access to the source code and  rights to copy, modify and redistribute granted by the license, 
users are provided only with a limited warranty and the software's author, the holder of the economic rights,  and the successive licensors have only limited liability. 
In this respect, the user's attention is drawn to the risks associated with loading, using, modifying and/or developing or reproducing the software by the user in light of its specific status of free software, 
that may mean  that it is complicated to manipulate, and that also therefore means that it is reserved for developers and experienced professionals having in-depth computer knowledge. 
Users are therefore encouraged to load and test the software's suitability as regards their requirements in conditions enabling the security of their systems and/or data to be ensured 
and, more generally, to use and operate it in the same conditions as regards security. 
The fact that you are presently reading this means that you have had knowledge of the CeCILL v2.1 license and that you accept its terms.
*/

#ifndef __COM_INTERFACE_H__
#define __COM_INTERFACE_H__

#include <string>

/******  List of interfaces ******
*  1- help : list all the request supported by kheops
*  2- cmd :  general command
*	a- resume 	
*       b- pause 
*	c- quit
*  3- weight : command relative to the weight file
*	a- save 'path'  (path is not mandatory, take the default weigth path)
*	b- load 'path'  (path is not mandatory, take the default weigth path)
*  4- rt_stat : get rt_token status 
*  5- oscillo : 
* 	a- start  (active oscillo)
*	b- stop   (stop oscillo)
*  6- output :
* 	a- start 'str'  (active oscillo str : can be 'uuid' or 'all' )
*	b- stop  'str'  (stop oscillo str : can be 'uuid' or 'all' )
*********************************/

const std::string UNKNOWN("unknown");
const std::string CMD[] = {"resume", "quit","pause"};
const std::string WEIGHT[] = {"save", "load"};
const std::string OUTPUT[] = {"start", "stop"};
const std::string OSCILLO[] = {"start", "stop"};

class ComInterface{

	public : 

		ComInterface(){}
		~ComInterface(){}

		virtual void init(int argc, char ** argv, std::string prog_name, std::string script_name) = 0;
		virtual void registerListener() = 0;
		virtual void quit() = 0;	
		virtual void enter() = 0;

};

#endif // __COM_INTERFACE_H__

