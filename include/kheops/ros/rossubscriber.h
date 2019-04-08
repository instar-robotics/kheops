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


#ifndef __ROS_SUBSCRIBER_HPP__
#define __ROS_SUBSCRIBER_HPP__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "kheops/ros/rosinterface.h"

#define TIMESLICE 1000

template<class RosMessage>
class RosSubscriber
{
	protected :

		ros::Subscriber sub;
		ros::CallbackQueue my_queue;
		ros::NodeHandle n;

	public : 
	
		RosSubscriber() {
			n.setCallbackQueue(&my_queue);
		}
		virtual ~RosSubscriber(){}

		void enable(std::string& topic_name,int size_queue ){ 
			my_queue.enable();
			sub = n.subscribe( topic_name, size_queue, &RosSubscriber<RosMessage>::callback, this);
		}

		void disable(){ 
			my_queue.disable();
			n.shutdown();
		}

		virtual void callback(const typename RosMessage::ConstPtr &msg) = 0;
		virtual void callOne(double time)
		{
			if( time < 0 )
        		{
                		ros::CallbackQueue::CallOneResult res = ros::CallbackQueue::TryAgain ;
				while( ros::ok() && 
					res != ros::CallbackQueue::Called &&  
						res != ros::CallbackQueue::Disabled )
				{
					res = my_queue.callOne(ros::WallDuration(TIMESLICE)); 
				}

        		}
        		else{
                		my_queue.callOne(ros::WallDuration(time));
        		}
		}
};

#endif // __ROS_SUBSCRIBER_HPP__
