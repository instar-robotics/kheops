/*
Copyright INSTAR Robotics

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

#ifndef __ROS_SUBSCRIBER_HPP__
#define __ROS_SUBSCRIBER_HPP__

#include "ros/ros.h"
#include "roswrapper.h"
#include <ros/callback_queue.h>


template<class RosMessage>
class RosSubscriber
{
	protected :

		std::string topic_name;

		RosMessage msg;
                int size_queue;

		ros::Subscriber sub;
		ros::CallbackQueue my_queue;
		ros::NodeHandle n;

	public : 
	
		RosSubscriber(int size_queue) : size_queue(size_queue) {}
		RosSubscriber(int size_queue, const std::string& topic) : size_queue(size_queue), topic_name(topic) 
		{
			RosWrapper::clean_topic_name(topic_name);
		}
		virtual ~RosSubscriber(){}

		void subscribe()
		{
			n.setCallbackQueue(&my_queue);
			sub = n.subscribe( topic_name, size_queue, &RosSubscriber<RosMessage>::callback, this);
		}

		virtual void callback(const typename RosMessage::ConstPtr &msg) = 0;
};

#endif // __ROS_SUBSCRIBER_HPP__