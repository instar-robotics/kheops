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

#include "kheops/ros/rospublisher.h"

/*******************************************************************************************************/
/*****************	                     	ROSTopic                             *******************/
/*******************************************************************************************************/

template<class RosMessage>
void RosTopic<RosMessage>::open(const std::string &topic)
{
	pub = RosWrapper::getNodeHandle()->advertise<RosMessage>( topic , size_queue);
}

/*******************************************************************************************************/
/*****************                         ROSDataPublisher                          *******************/
/*******************************************************************************************************/

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::open()
{
	Publisher::state = false;
	RosTopic<RosMessage>::close();

	if( ! Publisher::state )
	{
		RosTopic<RosMessage>::open( RosWrapper::getNodeName() +"/"+Publisher::pub_name );
	}
	Publisher::state = true;
}

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::close()
{
	Publisher::state = false;
	RosTopic<RosMessage>::close();
}

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::publish()
{
	RosTopic<RosMessage>::publish();
}

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::setPubName(const std::string & pub_name)
{
	Publisher::pub_name = pub_name;	
	RosWrapper::clean_topic_name(Publisher::pub_name);
}

/*******************************************************************************************************/
/*****************                        ROSArrayPublisher                          *******************/
/*******************************************************************************************************/

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::open()
{
	Publisher::state = false;
	RosTopic<RosMessage>::close();

	if( !Publisher::state )
	{
		RosTopic<RosMessage>::open(  RosWrapper::getNodeName() +"/"+Publisher::pub_name );
	}
	Publisher::state = true;
}

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::close()
{
	Publisher::state = false;
	RosTopic<RosMessage>::close();
}


template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::publish()
{
	 RosTopic<RosMessage>::publish();
}

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::setPubName(const std::string & pub_name)
{
	Publisher::pub_name = pub_name;	
	RosWrapper::clean_topic_name(Publisher::pub_name);
}

/*******************************************************************************************************/
/*****************                       ROSOscilloPublisher                         *******************/
/*******************************************************************************************************/

void RosOscilloPublisher::add(const OscilloMessage &m)
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


void RosOscilloPublisher::clear()
{
	msg.array.clear();
}

void RosOscilloPublisher::resize(int size)
{
	msg.array.resize(size);
}


/*******************************************************************************************************/
/*****************                    RosRtTokenOutputPublisher                      *******************/
/*******************************************************************************************************/

void RosRtTokenOutputPublisher::setMessage(const OscilloMessage& m)
{
	msg.uuid = m.uuid;
	msg.period = m.period;
	msg.means = m.means;
	msg.sleep = m.sleep;
	msg.duration = m.duration;
	msg.start = m.start;
	msg.warning = m.warning;
}


/*******************************************************************************************************/
/*****************                        RosScalarPublisher                         *******************/
/*******************************************************************************************************/

void RosScalarPublisher::setMessage(const double& m)
{
	msg.data = m;
}

/*******************************************************************************************************/
/*****************                        RosMatrixPublisher                         *******************/
/*******************************************************************************************************/

void RosMatrixPublisher::setMessage(const MatrixXd& m)
{
	if( !checkSize( m.rows(), m.cols() )) setSize(m.rows(), m.cols());

	Map<MatrixXd> mEnc( msg.data.data() , msg.layout.dim[0].size ,msg.layout.dim[1].size );
	mEnc = m;
}

void RosMatrixPublisher::setSize(unsigned int rows,unsigned int cols)
{
	msg.layout.dim[0].stride = rows  * cols ;
	msg.layout.dim[0].size = rows;
	msg.layout.dim[1].stride = cols;
	msg.layout.dim[1].size = cols;

	msg.data.resize( rows * cols );
}

bool RosMatrixPublisher::checkSize( unsigned int rows, unsigned int cols )
{
	if( rows == msg.layout.dim[0].size  && cols == msg.layout.dim[1].size ) return true;
	else return false;
}

/*******************************************************************************************************/
/*****************                        RosStatusPublisher                         *******************/
/*******************************************************************************************************/

void RosStatusPublisher::setMessage(const StatusMessage& m)
{
	msg.key = m.key;
	msg.value = m.value;	
}
