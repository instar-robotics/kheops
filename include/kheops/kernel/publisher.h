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


#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

#include "type.h"

class Publisher
{
	protected : 

		std::string pub_name;

	public : 
		Publisher() {}
		virtual ~Publisher(){}

		virtual void open() = 0;
		virtual void close() = 0;
		virtual void publish() = 0;
		virtual bool is_open() = 0;
	
		virtual void setPubName(const std::string & pub_name) 
		{
			this->pub_name = pub_name;
		}
};

template<class Message>
class ArrayPublisher : public Publisher
{
	public : 
		
		ArrayPublisher() {}
		virtual ~ArrayPublisher(){}	

		virtual void add(const Message& m) = 0; 
		virtual void clear() = 0;
		virtual void resize(int size) = 0;
};

template<class Message>
class DataPublisher : public Publisher
{
	public :
		DataPublisher(){}
		virtual ~DataPublisher(){}

		virtual void setMessage(const Message& m) = 0;
};


class OscilloMessage{

	public : 
		const std::string &uuid;
		double means;
		double duration;
		double start; 
		double minDuration;
		double maxDuration;

		OscilloMessage(const std::string &uuid) : uuid(uuid) {}
};

class RtTokenMessage{

	public : 
		const std::string &uuid;
		double period;
		double means;
		double sleep;
		double duration;
		double start; 
		double minDuration;
		double maxDuration;
		bool warning;

		RtTokenMessage(const std::string &uuid) : uuid(uuid) {}
};

class StatusMessage{
	public : 
		std::string key;
		std::string value;
};


typedef DataPublisher<RtTokenMessage> RtTokenOutputPublisher;
typedef DataPublisher<MATRIX> MatrixPublisher;
typedef DataPublisher<SCALAR> ScalarPublisher;
typedef DataPublisher<StatusMessage> StatusPublisher;

class OscilloPublisher : public Publisher
{
	public : 
		
		OscilloPublisher() {}
		virtual ~OscilloPublisher(){}	

		virtual void add(const OscilloMessage& m) = 0; 
		virtual void addRt(const RtTokenMessage& m) = 0; 
		virtual void clear() = 0;
		virtual void resize(int size) = 0;
};

#endif // __PUBLISHER_H__
