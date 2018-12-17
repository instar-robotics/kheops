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

#include <sstream>
#include <cxxabi.h>
#include <chrono>
#include "kheops/kernel/function.h"
#include "kheops/ros/rospublisher.h"
#include "kheops/kernel/cominterface.h"

/*******************************************************************************************************/
/******************************************** FUNCTION *************************************************/
/*******************************************************************************************************/

Function::~Function()
{
	input.clear();
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

void Function::ksetparameters()
{
	checkType();
	setparameters();
}

void Function::kexec_afterCompute()
{
	// Buffer input
	// Send debug output [ROS Topic ] ...
	copy_buffer();
	publish_data();
	
	if( is_publish_active() )
	{
		publish_activity();
	}

	// Save Activity
	if( is_save_active() )
	{
		save_activity();
	}
	exec_afterCompute();
}

void Function::kprerun()
{
	// Crapy ! Should be better to create an intermediate class between FTemplate and FMatrix
	checkSize();

	if( is_save_active() )
	{
		load_save();
		active_save(true);

		// Allow to get activity for reccurent link
		copy_buffer();
	}

	active_publish(is_publish_active());

	prerun();
}

/*******************************************************************************************************/
/********************************************* FTEMPLATE ***********************************************/
/*******************************************************************************************************/

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
void FTemplate<T>::set_pub_name(const std::string& pubname)
{
	if(pubname.size()==0 )
	{
		std::string name = "function_";
		name+=getUuid();
		ComInterface::setDefaultName(name);
		o_pub->setPubName(name);
	}
	else 
	{
		o_pub->setPubName(pubname);
	}
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
void FTemplate<T>::checkSize()
{
        for( auto in = input.begin() ; in != input.end(); in++  )
	{
		if( (*in)->isCheckSize() )
		{
			for( unsigned int i = 0 ; i < (*in)->size(); i++ )
			{
				compareSize( (**in)[i] );
			}
		}
	}	
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
void FTemplate<T>::load_save()
{
	shm_read( getUuid() , output );
}

template<class T>
void FTemplate<T>::publish_activity()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() )
		{
			o_pub->setMessage(output);
			o_pub->publish();
		}
	}
}

template<class T>
void FTemplate<T>::save_activity()
{
		serializer.write(output);		
}

/*******************************************************************************************************/
/********************************************* FMATRIX *************************************************/
/*******************************************************************************************************/

FMatrix::FMatrix(unsigned int type) : FTemplate(), type(type)  
{
	o_pub = new RosMatrixPublisher(1);
}

void FMatrix::compareSize(iLinkBase& link)
{
	double row,col;
	if( link.i_type_name() ==  type_name()) 
	{
		if( link.w_type_name() == type_name() )
		{
			row = dynamic_cast<iLink<MatrixXd,MatrixXd>&>(link).i().rows();
			col = dynamic_cast<iLink<MatrixXd,MatrixXd>&>(link).i().cols();
		}
		else
		{
			row = dynamic_cast<iLink<MatrixXd,double>&>(link).i().rows();
			col = dynamic_cast<iLink<MatrixXd,double>&>(link).i().cols();
		}

		if(   output.rows() != 	row || output.cols() !=  col )
		{
			 std::stringstream buf ;
                	 buf << "FATAL : checkSize failed ! Matrix Function [" + getUuid() << "] is (" << output.rows() << "," << output.cols() << ") and iLink [" << link.getUuid() << "] is (" << row << "," <<  col << ")" ; 	
                	 throw std::invalid_argument(buf.str());
		}
	}
}

void FMatrix::checkType()
{
	if( type == POINT && !isPoint() )
	{
		 throw std::invalid_argument(class_name()+" : output must be a Point [(1,1) Matrix]");
	}
	if( type == VECTOR && !isVect() )
	{
		 throw std::invalid_argument(class_name()+" : output must be a Vector [ROW or COL]");
	}
	if( type == RVECTOR && !isRowVect() ) 
	{
		 throw std::invalid_argument(class_name()+" : output must be a ROW Vector");
	}	
	if( type == CVECTOR && !isColVect() ) 
	{
		 throw std::invalid_argument(class_name()+" : output must be a COL Vector");
	}	
}

/*******************************************************************************************************/
/********************************************* FSCALAR *************************************************/
/*******************************************************************************************************/

FScalar::FScalar() : FTemplate() {
	output = 0;
	
	o_pub = new RosScalarPublisher(1);
}

FScalar::FScalar( double dvalue) : FTemplate() {
	output = dvalue;
	
	o_pub = new RosScalarPublisher(1);
}

