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
	checkShape();
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

FMatrix::FMatrix(unsigned int shape) : FTemplate(), shape(shape)  
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

void FMatrix::checkShape()
{
	if( shape == POINT && !isPoint() )
	{
		 throw std::invalid_argument(class_name()+" : output must be a Point [(1,1) Matrix]");
	}
	if( shape == VECTOR && !isVect() )
	{
		 throw std::invalid_argument(class_name()+" : output must be a Vector [ROW or COL]");
	}
	if( shape == RVECTOR && !isRowVect() ) 
	{
		 throw std::invalid_argument(class_name()+" : output must be a ROW Vector");
	}	
	if( shape == CVECTOR && !isColVect() ) 
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

