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


#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <vector>
#include <ros/ros.h>
#include "kheops/kernel/cominterface.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rosutil.h"
#include "hieroglyph/Help.h"
#include "hieroglyph/RtStat.h"
#include "hieroglyph/SimpleCmd.h"
#include "hieroglyph/ArgCmd.h"
#include "hieroglyph/ArgsCmd.h"
#include "hieroglyph/Objects.h"

const int DEFAULT_RATE = 10; //10 hz

class RosInterface : public ComInterface{
	
	private :

		ros::ServiceServer sHelper;
		ros::ServiceServer sControl;
		ros::ServiceServer sWeight;
		ros::ServiceServer sRtStat;
		ros::ServiceServer sOutput;
		ros::ServiceServer sOscillo;
		ros::ServiceServer sObjects;
		ros::ServiceServer sRtToken;
		ros::ServiceServer sActivity;
		ros::ServiceServer sComment;
		ros::ServiceServer sDebug;
		//ros::ServiceServer sGetControlStatus;
		
		RosInterface(){}

	public : 

		static void build();
		static void destroy();
		static RosInterface *getInstance(){return dynamic_cast<RosInterface*>(singleton);}

		virtual ~RosInterface(){ros::shutdown();}

		virtual int enter();
		virtual void registerListener();
		virtual void _init(int argc, char ** argv, std::string prog_name, std::string script_name, uint32_t options = 0);
		virtual void _setDefaultName(std::string& str);

		bool callback_debug(hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response);
		bool callback_comment(hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response);
		bool callback_activity(hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response);
		bool callback_rt_token(hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response);
		bool callback_objects(hieroglyph::Objects::Request& request, hieroglyph::Objects::Response& response);
		bool callback_oscillo(hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response);
		bool callback_output( hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response);
		bool callback_rt_stat( hieroglyph::RtStat::Request& request, hieroglyph::RtStat::Response& response);
		bool callback_weight( hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response);
		bool callback_control( hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response);
		bool callback_helper( hieroglyph::Help::Request& request, hieroglyph::Help::Response& response);
};

#endif // __ROS_INTERFACE_H__
