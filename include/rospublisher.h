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

#ifndef __ROS_PUBLISHER_H__
#define __ROS_PUBLISHER_H__

#include "publisher.h" 
#include "roswrapper.h"
#include "ros/ros.h"
#include "hieroglyph/OscilloArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::Map;
using Eigen::MatrixXd;

class RosTopic 
{
	protected : 
		
		int size_queue;
		ros::Publisher pub;

	public : 

		RosTopic(){}
		RosTopic(int size) : size_queue(size){}
		virtual ~RosTopic(){}

		void setSize(int size) {size_queue = size;}
		
};

template<class Message>
class RosDataPublisher : public DataPublisher<Message>, public RosTopic
{
	public : 
		RosDataPublisher(int size) : DataPublisher<Message>() ,  RosTopic(size) {}
		virtual ~RosDataPublisher() {}

		virtual void close()
		{ 
			Publisher::state = false;
			pub.shutdown(); 
		}
};

template<class Message>
class RosArrayPublisher : public ArrayPublisher<Message> , public RosTopic
{
	public : 
		RosArrayPublisher(int size) : ArrayPublisher<Message>(), RosTopic(size) {}
		virtual ~RosArrayPublisher() {}

		virtual void close()
		{ 
			Publisher::state = false;
			pub.shutdown(); 
		}
		
};

class RosOscilloPublisher : public RosArrayPublisher<OscilloMessage>
{
	private :

		hieroglyph::OscilloArray msg;  

	public : 
		RosOscilloPublisher(int size) : RosArrayPublisher(size) {}
		virtual ~RosOscilloPublisher(){}

		virtual void add(const OscilloMessage &m) 
		{
			hieroglyph::OscilloData osc_data;	

			osc_data.uuid = m.uuid;
			osc_data.period = m.period;
			osc_data.means = m.means;
			osc_data.sleep = m.sleep;
			osc_data.duration = m.duration;
			osc_data.start = m.start;
			osc_data.warning = m.warning;

			msg.array.push_back(osc_data);		
		}
		
		virtual void clear()
		{
			msg.array.clear();
		}

		virtual void resize(int size)
		{
			msg.array.resize(size);		
		}

		virtual void publish(){	pub.publish(msg);}
		
		virtual void open()
		{
			state = true;
			pub = RosWrapper::getNodeHandle()->advertise<hieroglyph::OscilloArray>( RosWrapper::getNodeName() +"/oscillo", size_queue);
		}
};


class RosRtTokenOutputPublisher : public RosDataPublisher<OscilloMessage>
{
	private : 
		
		hieroglyph::OscilloData msg;	

	public : 

		RosRtTokenOutputPublisher(int size) : RosDataPublisher(size) {}
		virtual ~RosRtTokenOutputPublisher(){}

		virtual void setMessage(const OscilloMessage& m)
                {
			msg.uuid = m.uuid;
			msg.period = m.period;
			msg.means = m.means;
			msg.sleep = m.sleep;
			msg.duration = m.duration;
			msg.start = m.start;
			msg.warning = m.warning;
                }

		virtual void open()
		{
			state = true;
			pub = RosWrapper::getNodeHandle()->advertise<hieroglyph::OscilloData>( RosWrapper::getNodeName() +"/rt_token", size_queue);
		}

		virtual void publish(){	pub.publish(msg);}
};

class RosMatrixPublisher : public RosDataPublisher<MatrixXd>
{
	private : 

		std_msgs::Float64MultiArray msg;
		std::string topic_name;

	public : 

		RosMatrixPublisher(int size, std::string topic_name) : RosDataPublisher(size),topic_name(topic_name){}
		virtual ~RosMatrixPublisher(){}

		virtual void setMessage(const MatrixXd& m)
		{
			Map<MatrixXd> mEnc( msg.data.data() , msg.layout.dim[0].size ,msg.layout.dim[1].size );
			mEnc = m;
		}

		virtual void setSize(int rows, int cols)
		{
			msg.layout.dim.resize(2);
        		msg.layout.dim[0].stride = rows  * cols ;
        		msg.layout.dim[0].size = rows;
        		msg.layout.dim[1].stride = cols;
        		msg.layout.dim[1].size = cols;

			msg.data.resize( rows * cols  );
		}
		
		virtual void open()
		{
			state = true;
			pub = RosWrapper::getNodeHandle()->advertise<std_msgs::Float64MultiArray>( RosWrapper::getNodeName() +topic_name, size_queue);
		}

		virtual void publish(){	pub.publish(msg);}
};

#endif // __ROS_PUBLISHER_H__
