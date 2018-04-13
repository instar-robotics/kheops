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

#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <vector>
#include "cominterface.h"
#include "kernel.h"
#include "ros/ros.h"
#include "hieroglyph/Help.h"
#include "hieroglyph/Cmd.h"
#include "hieroglyph/Weight.h"

class RosInterface : public ComInterface{
	
	private :

		std::string node_name;

		ros::ServiceServer sHelper;
		ros::ServiceServer sCmd;
		ros::ServiceServer sWeight;
		ros::NodeHandle * n;

	public : 

		RosInterface(){}
		~RosInterface(){ delete(n); }

		virtual void init(int argc, char ** argv, std::string prog_name, std::string script_name);
		virtual void registerListener();
		virtual void quit() {ros::shutdown();}	
		virtual void enter(){ros::spin();}

		bool callback_weight( hieroglyph::Weight::Request& request, hieroglyph::Weight::Response& response);
		bool callback_cmd( hieroglyph::Cmd::Request& request, hieroglyph::Cmd::Response& response);
		bool callback_helper( hieroglyph::Help::Request& request, hieroglyph::Help::Response& response);
};

#endif // __COM_INTERFACE_H__

