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

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "diagnostic_msgs/KeyValue.h"
#include "kheops/kernel/publisher.h" 
#include "kheops/ros/roswrapper.h"
#include "hieroglyph/OscilloArray.h"
#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::Map;
using Eigen::MatrixXd;


template<class RosMessage> 
class RosTopic 
{
	protected : 

		RosMessage msg;
		
		int size_queue;
		ros::Publisher pub;
		bool latch;

	public : 

		RosTopic() : latch(false) {}
		RosTopic(int size) : size_queue(size), latch(false){}
		virtual ~RosTopic(){}

		void setSize(int size) {size_queue = size;}

		//TODO : for now, i don't implement mutex exclusion on publish/shutdown
		//  a Topic could be closed during a publish call
		// I had some security in rosinterface : Kernel is paused during a close call
		// however, this could had some latency during start/stop call
		// So it should be a better idee to had a Condition_variable in RosTopic class to avoid problem and remove kernel pause during services call execution
		void open(const std::string &topic);
		void close() { pub.shutdown();}
		void publish(){	pub.publish(msg);}

		void setLatch(bool latch) {this->latch = latch;}
};


template<class Message, class RosMessage>
class RosDataPublisher : public DataPublisher<Message>, public RosTopic<RosMessage>
{
	public : 
		RosDataPublisher(int size) : DataPublisher<Message>() ,  RosTopic<RosMessage>(size) {}
		virtual ~RosDataPublisher() {}

		virtual void open();
		virtual void close();
		virtual void publish();

		virtual void setPubName(const std::string & pub_name);

};

template<class Message, class RosMessage>
class RosArrayPublisher : public ArrayPublisher<Message> , public RosTopic<RosMessage>
{
	public : 
		RosArrayPublisher(int size) : ArrayPublisher<Message>(), RosTopic<RosMessage>(size) {}
		virtual ~RosArrayPublisher() {}
		
		virtual void open();
                virtual void close();
                virtual void publish();
		
		virtual void setPubName(const std::string & pub_name);
};

class RosOscilloPublisher : public RosArrayPublisher<OscilloMessage,hieroglyph::OscilloArray>
{
	public : 
		RosOscilloPublisher(int size, const std::string& pub_name) : RosArrayPublisher(size) {
			Publisher::pub_name = pub_name ;
			RosWrapper::clean_topic_name(Publisher::pub_name);
		}
		virtual ~RosOscilloPublisher(){}

		virtual void add(const OscilloMessage &m);
		virtual void clear();
		virtual void resize(int size);
};

class RosRtTokenOutputPublisher : public RosDataPublisher<OscilloMessage, hieroglyph::OscilloData>
{
	public : 

		RosRtTokenOutputPublisher(int size, const std::string& pub_name ) : RosDataPublisher(size) {
			Publisher::pub_name=pub_name; 
			RosWrapper::clean_topic_name(Publisher::pub_name);
		}

		virtual ~RosRtTokenOutputPublisher(){}
		virtual void setMessage(const OscilloMessage& m);
};

class RosScalarPublisher : public RosDataPublisher<double,std_msgs::Float64>
{
	public :
		RosScalarPublisher(int size) : RosDataPublisher(size){}
		virtual ~RosScalarPublisher(){}
		
		virtual void setMessage(const double& m);
};

class RosMatrixPublisher : public RosDataPublisher<MatrixXd, std_msgs::Float64MultiArray>
{
	public : 

		RosMatrixPublisher(int size) : RosDataPublisher(size){ msg.layout.dim.resize(2);}
		virtual ~RosMatrixPublisher(){}

		virtual void setMessage(const MatrixXd& m);
		virtual void setSize(unsigned int rows,unsigned int cols);
		virtual bool checkSize( unsigned int rows, unsigned int cols );
};

class RosStatusPublisher : public RosDataPublisher<StatusMessage,diagnostic_msgs::KeyValue>
{
	public : 
		RosStatusPublisher(int size, const std::string& pub_name ) : RosDataPublisher(size)
		{
			RosTopic::setLatch(true);
			Publisher::pub_name = pub_name ;
			RosWrapper::clean_topic_name(Publisher::pub_name);
		}
                virtual ~RosStatusPublisher(){}

                virtual void setMessage(const StatusMessage& m);
};

#endif // __ROS_PUBLISHER_H__
