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
#include "std_msgs/Float64.h"
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

	public : 

		RosTopic(){}
		RosTopic(int size) : size_queue(size){}
		virtual ~RosTopic(){}

		void setSize(int size) {size_queue = size;}
		
		void open(const std::string &topic)
		{
			pub = RosWrapper::getNodeHandle()->advertise<RosMessage>( topic , size_queue);
		}
		
		//TODO : for now, i don't implement mutex exclusion on publish/shutdown
		//  a Topic could be closed during a publish call
		// I had some security in rosinterface : Kernel is paused during a close call
		// however, this could had some latency during start/stop call
		// So it should be a better idee to had a Condition_variable in RosTopic class to avoid problem and remove kernel pause during services call execution
		void close() { pub.shutdown();}
		void publish(){	pub.publish(msg);}
};


template<class Message, class RosMessage>
class RosDataPublisher : public DataPublisher<Message>, public RosTopic<RosMessage>
{
	public : 
		RosDataPublisher(int size) : DataPublisher<Message>() ,  RosTopic<RosMessage>(size) {}
		virtual ~RosDataPublisher() {}

		virtual void open()
		{ 
			Publisher::state = false;
			RosTopic<RosMessage>::close();
			
			if( ! Publisher::state )
			{  
				RosTopic<RosMessage>::open(  RosWrapper::getNodeName() +"/"+Publisher::pub_name );
			}	
			Publisher::state = true;
		}

		virtual void close()
		{ 
			Publisher::state = false;
			RosTopic<RosMessage>::close();
		}
		
		virtual void publish(){	RosTopic<RosMessage>::publish();}
};

template<class Message, class RosMessage>
class RosArrayPublisher : public ArrayPublisher<Message> , public RosTopic<RosMessage>
{
	public : 
		RosArrayPublisher(int size) : ArrayPublisher<Message>(), RosTopic<RosMessage>(size) {}
		virtual ~RosArrayPublisher() {}
		
		virtual void open()
                {
                        Publisher::state = false;
                        RosTopic<RosMessage>::close();

                        if( !Publisher::state )
                        {
                                RosTopic<RosMessage>::open(  RosWrapper::getNodeName() +"/"+Publisher::pub_name );
                        }
                        Publisher::state = true;
                }

                virtual void close()
                {
                        Publisher::state = false;
                        RosTopic<RosMessage>::close();
                }

                virtual void publish(){ RosTopic<RosMessage>::publish();}
};

class RosOscilloPublisher : public RosArrayPublisher<OscilloMessage,hieroglyph::OscilloArray>
{
	public : 
		RosOscilloPublisher(int size) : RosArrayPublisher(size) {
			pub_name = "oscillo" ;
		}
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
};

class RosRtTokenOutputPublisher : public RosDataPublisher<OscilloMessage, hieroglyph::OscilloData>
{
	public : 

		RosRtTokenOutputPublisher(int size, const std::string& pub_name ) : RosDataPublisher(size) {
			Publisher::pub_name=pub_name; 
		}

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
};

class RosScalarPublisher : public RosDataPublisher<double,std_msgs::Float64>
{
	public :
		RosScalarPublisher(int size) : RosDataPublisher(size){}
		virtual ~RosScalarPublisher(){}
		
		virtual void setMessage(const double& m)
		{
			msg.data = m;
		}
};

class RosMatrixPublisher : public RosDataPublisher<MatrixXd, std_msgs::Float64MultiArray>
{
	public : 

		RosMatrixPublisher(int size) : RosDataPublisher(size){ msg.layout.dim.resize(2);}
		virtual ~RosMatrixPublisher(){}

		virtual void setMessage(const MatrixXd& m)
		{
			if( !checkSize( m.rows(), m.cols() )) setSize(m.rows(), m.cols());	

			Map<MatrixXd> mEnc( msg.data.data() , msg.layout.dim[0].size ,msg.layout.dim[1].size );
			mEnc = m;
		}

		virtual void setSize(unsigned int rows,unsigned int cols)
		{
        		msg.layout.dim[0].stride = rows  * cols ;
        		msg.layout.dim[0].size = rows;
        		msg.layout.dim[1].stride = cols;
        		msg.layout.dim[1].size = cols;

			msg.data.resize( rows * cols );
		}

		virtual bool checkSize( unsigned int rows, unsigned int cols )
		{
			if( rows == msg.layout.dim[0].size  && cols == msg.layout.dim[1].size ) return true;
			else return false;
		}
};

#endif // __ROS_PUBLISHER_H__
