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

#ifndef __PUBLISHER_H__
#define __PUBLISHER_H__

class Publisher
{
	protected : 
		bool state;

	public : 
		Publisher() : state(false) {}
		virtual ~Publisher(){}

		virtual void open() = 0;
		virtual void close() = 0;
		virtual void publish() = 0;
		virtual bool is_open() {return state;}
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
		double period;
		double means;
		double sleep;
		double duration;
		double start; 
		bool warning;

		OscilloMessage(const std::string &uuid) : uuid(uuid) {}
};

typedef ArrayPublisher<OscilloMessage> OscilloPublisher;
typedef DataPublisher<OscilloMessage> RtTokenOutputPublisher;
typedef DataPublisher<MatrixXd> MatrixPublisher;
typedef DataPublisher<double> ScalarPublisher;

#endif // __PUBLISHER_H__
