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


#include "kheops/ros/rospublisher.h"

/*******************************************************************************************************/
/*****************	                     	ROSTopic                             *******************/
/*******************************************************************************************************/

template<class RosMessage>
void RosTopic<RosMessage>::open(const std::string &topic)
{
	if(!state)
	{
		ros::NodeHandle n;
		pub = n.advertise<RosMessage>( topic , size_queue, latch);
		state = true;
	}
}

template<class RosMessage>
void RosTopic<RosMessage>::close()
{
	if(state)
	{
		pub.shutdown();
		state = false;
	}
}

/*******************************************************************************************************/
/*****************                         ROSDataPublisher                          *******************/
/*******************************************************************************************************/

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::open()
{
	RosTopic<RosMessage>::open( Publisher::pub_name );
}

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::close()
{
	RosTopic<RosMessage>::close();
}

template<class Message, class RosMessage>
void RosDataPublisher<Message,RosMessage>::publish()
{
	RosTopic<RosMessage>::publish();
}

/*******************************************************************************************************/
/*****************                        ROSArrayPublisher                          *******************/
/*******************************************************************************************************/

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::open()
{
	RosTopic<RosMessage>::open( Publisher::pub_name );
}

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::close()
{
	RosTopic<RosMessage>::close();
}

template<class Message, class RosMessage>
void RosArrayPublisher<Message,RosMessage>::publish()
{
	 RosTopic<RosMessage>::publish();
}

/*******************************************************************************************************/
/*****************                       ROSOscilloPublisher                         *******************/
/*******************************************************************************************************/

void RosOscilloPublisher::open()
{
        RosTopic<hieroglyph::OscilloArray>::open( Publisher::pub_name );
}

void RosOscilloPublisher::close()
{
        RosTopic<hieroglyph::OscilloArray>::close();
}

void RosOscilloPublisher::publish()
{
         RosTopic<hieroglyph::OscilloArray>::publish();
}


void RosOscilloPublisher::add(const OscilloMessage &m)
{
	hieroglyph::OscilloData osc_data;

	osc_data.uuid = m.uuid;
	osc_data.means = m.means;
	osc_data.duration = m.duration;
	osc_data.start = m.start;
	osc_data.maxDuration = m.maxDuration;
	osc_data.minDuration = m.minDuration;

	msg.array.push_back(osc_data);
}

void RosOscilloPublisher::addRt(const RtTokenMessage &m)
{
	msg.rt_data.uuid = m.uuid;
	msg.rt_data.period = m.period;
	msg.rt_data.means = m.means;
	msg.rt_data.sleep = m.sleep;
	msg.rt_data.duration = m.duration;
	msg.rt_data.start = m.start;
	msg.rt_data.minDuration = m.minDuration;
	msg.rt_data.maxDuration = m.maxDuration;
	msg.rt_data.warning = m.warning;
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

void RosRtTokenOutputPublisher::setMessage(const RtTokenMessage& m)
{
	msg.uuid = m.uuid;
	msg.period = m.period;
	msg.means = m.means;
	msg.sleep = m.sleep;
	msg.duration = m.duration;
	msg.start = m.start;
	msg.warning = m.warning;
	msg.maxDuration = m.maxDuration;
	msg.minDuration = m.minDuration;
}


/*******************************************************************************************************/
/*****************                        RosScalarPublisher                         *******************/
/*******************************************************************************************************/

void RosScalarPublisher::setMessage(const SCALAR& m)
{
	msg.data = m;
}

/*******************************************************************************************************/
/*****************                        RosMatrixPublisher                         *******************/
/*******************************************************************************************************/

void RosMatrixPublisher::setMessage(const MATRIX& m)
{
	if( !checkSize( m.rows(), m.cols() )) setSize(m.rows(), m.cols());

	Map<MATRIX> mEnc( msg.data.data() , msg.layout.dim[0].size ,msg.layout.dim[1].size );
	mEnc = m;
}

void RosMatrixPublisher::setSize(unsigned int rows,unsigned int cols)
{
	msg.layout.dim[0].label = "rows" ;
	msg.layout.dim[0].stride = rows  * cols ;
	msg.layout.dim[0].size = rows;
	msg.layout.dim[1].stride = cols;
	msg.layout.dim[1].size = cols;
	msg.layout.dim[1].label = "cols" ;
	msg.layout.data_offset = 0;

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
