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

#include <chrono>
#include "kheops/kernel/function.h"
#include "kheops/ros/rospublisher.h"

/********************************************************************************************************/
/********************************************* FUNCTION *************************************************/
/********************************************************************************************************/

Function::~Function()
{
	input.clear();
}

void Function::exec()
{
	try
	{
		compute();
	}
	catch(const std::invalid_argument &e)
	{
		std::cerr << " Function "<< getUuid() << " : "  << e.what() << std::endl;
	}
	catch(...)
	{
		std::cerr << " Function "<< getUuid() << " : unknown execption"  << std::endl;
	}
}

void Function::copy_buffer()
{
        // Buffer input
        // Send debug output [ROS Topic ] ...
        for( auto in = input.begin() ; in != input.end(); in++  )
        {
                for( unsigned int i = 0 ; i < (*in)->size(); i++ )
                {
                        (**in)[i].copyBuffer();
                }
        }
}

void Function::publish_data()
{
	// Buffer input
        // Send debug output [ROS Topic ] ...
        for( auto in = input.begin() ; in != input.end(); in++  )
        {
                for( unsigned int i = 0 ; i < (*in)->size(); i++ )
                {
                        (**in)[i].publish_message();
                }
        }
}

void Function::exec_afterCompute()
{
	// Buffer input
	// Send debug output [ROS Topic ] ...
	copy_buffer();
	publish_data();

	uexec_afterCompute();
}

/********************************************************************************************************/
/********************************************** FTEMPLATE ***********************************************/
/********************************************************************************************************/

template<class T>
FTemplate<T>::~FTemplate()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() ) o_pub->close();
		delete(o_pub);
	}
}

template<class T>
void FTemplate<T>::set_topic_name(const std::string& topic)
{
	std::string name = "function_";
	if(topic.size()==0 )
	{
		name+=getUuid();
	}
	else name+=topic;

	o_pub->setPubName(name);
}


template<class T>
void FTemplate<T>::active_publish(bool state)
{
	if( state )
	{
		if( o_pub != NULL  )
		{
			if( !o_pub->is_open() )  o_pub->open();
		}
		else throw std::invalid_argument("Function : failed to open output publisher");
	}
	else
	{
		if( o_pub != NULL)
		{
			if( o_pub->is_open()) o_pub->close();
		}
	}
	publish = state;
}

template<class T>
void FTemplate<T>::prerun()
{
	if( is_save_active() )
	{
		shm_read( getUuid() , output );
		active_save(true);

		// Allow to get activity for reccurent link
		copy_buffer();
	}

	active_publish(is_publish_active());

	uprerun();
}


template<class T>
void FTemplate<T>::active_save(bool state)
{
	if( state )
	{
		serializer.buildSHM(getUuid(), output );		
	}
	else
	{
		serializer.free();		
	}
	save = state ;
}

template<class T>
void FTemplate<T>::exec_afterCompute()
{
	Function::exec_afterCompute();

	if( is_publish_active() && o_pub != NULL)
	{
		if( o_pub->is_open() )
		{
			o_pub->setMessage(output);
			o_pub->publish();
		}
	}

	// Save Activity
	if( is_save_active() )
	{
		serializer.write(output);		
	}
}

/********************************************************************************************************/
/********************************************** FMATRIX *************************************************/
/********************************************************************************************************/

FMatrix::FMatrix() : FTemplate()  
{
	o_pub = new RosMatrixPublisher(1);
}

FMatrix::FMatrix( int X,int Y) : FTemplate()  
{ 
	output = MatrixXd::Constant( X , Y ,0);
	
	o_pub = new  RosMatrixPublisher(1);
}

FMatrix::FMatrix(int X,int Y, double dvalue) : FTemplate() 
{
	output = MatrixXd::Constant( X , Y ,dvalue);
	
	o_pub = new RosMatrixPublisher(1);
}

/********************************************************************************************************/
/********************************************** FSCALAR *************************************************/
/********************************************************************************************************/

FScalar::FScalar() : FTemplate() {
	output = 0;
	
	o_pub = new RosScalarPublisher(1);
}

FScalar::FScalar( double dvalue) : FTemplate() {
	output = dvalue;
	
	o_pub = new RosScalarPublisher(1);
}

