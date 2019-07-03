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


#ifndef __ROS_PUBLISHER_H__
#define __ROS_PUBLISHER_H__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "diagnostic_msgs/KeyValue.h"
#include "kheops/kernel/publisher.h" 
#include "kheops/ros/rosinterface.h"
#include "hieroglyph/OscilloArray.h"
#include "hieroglyph/RtToken.h"

using Eigen::Map;

template<class RosMessage> 
class RosTopic 
{
	protected : 

		bool state;
		RosMessage msg;
		
		int size_queue;
		ros::Publisher pub;
		bool latch;

	public : 

		RosTopic() : state(false), latch(false) {}
		RosTopic(int size) : state(false), size_queue(size), latch(false){}
		virtual ~RosTopic(){}

		void setSize(int size) {size_queue = size;}

		//TODO : for now, i don't implement mutex exclusion on publish/shutdown
		//  a Topic could be closed during a publish call
		// I had some security in rosinterface : Kernel is paused during a close call
		// however, this could had some latency during start/stop call
		// So it should be a better idee to had a Condition_variable in RosTopic class to avoid problem and remove kernel pause during services call execution
		void open(const std::string &topic);
		void close();
		void publish(){	pub.publish(msg);}
		bool is_open() {return state;}

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
		virtual bool is_open() { return RosTopic<RosMessage>::is_open();}
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
		virtual bool is_open() { return  RosTopic<RosMessage>::is_open();}
};

class RosOscilloPublisher : public OscilloPublisher,  public RosTopic<hieroglyph::OscilloArray>
{
	public : 
		RosOscilloPublisher(int size) : OscilloPublisher(),RosTopic<hieroglyph::OscilloArray>(size) {}
		virtual ~RosOscilloPublisher(){}

		virtual void add(const OscilloMessage &m);
		virtual void addRt(const RtTokenMessage& m);
		virtual void clear();
		virtual void resize(int size);
		virtual void open();
                virtual void close();
                virtual void publish();
		virtual bool is_open() { return  RosTopic<hieroglyph::OscilloArray>::is_open();}
};

class RosRtTokenOutputPublisher : public RosDataPublisher<RtTokenMessage, hieroglyph::RtToken>
{
	public : 

		RosRtTokenOutputPublisher(int size) : RosDataPublisher(size) {}

		virtual ~RosRtTokenOutputPublisher(){}
		virtual void setMessage(const RtTokenMessage& m);
};

class RosScalarPublisher : public RosDataPublisher<SCALAR,std_msgs::Float64>
{
	public :
		RosScalarPublisher(int size) : RosDataPublisher(size){}
		virtual ~RosScalarPublisher(){}
		
		virtual void setMessage(const SCALAR& m);
};

class RosMatrixPublisher : public RosDataPublisher<MATRIX, std_msgs::Float64MultiArray>
{
	public : 

		RosMatrixPublisher(int size) : RosDataPublisher(size){ msg.layout.dim.resize(2);}
		virtual ~RosMatrixPublisher(){}

		virtual void setMessage(const MATRIX& m);
		virtual void setSize(unsigned int rows,unsigned int cols);
		virtual bool checkSize( unsigned int rows, unsigned int cols );
};

class RosStatusPublisher : public RosDataPublisher<StatusMessage,diagnostic_msgs::KeyValue>
{
	public : 
		RosStatusPublisher(int size) : RosDataPublisher(size)
		{
			RosTopic::setLatch(true);
		}
                virtual ~RosStatusPublisher(){}

                virtual void setMessage(const StatusMessage& m);
};

#endif // __ROS_PUBLISHER_H__
