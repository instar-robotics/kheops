/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of alexandria <https://github.com/instar-robotics/alexandria>.
 
  alexandria is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  alexandria is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __FPUB__
#define __FPUB__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include <ros/ros.h>

/* Note :
 *      1- Each FMatrixPub or FScalarPub object has 2 default Kheops Input:
 *      - IString topic_name : for the topic Name
 *      - ISInput size_queue : define the size of the queue
 *
 *      2- This 2 inputs MUST BE BIND to the kernel in the method setparameters.
 *      If you extend setparameters to add other Inputs, don't forget to call FMatrixPub::setparameters or FScalarPub::setparameters !
 *
 *      3- And Most important : don't forget to add this Inputs in the XML description !
 *      For now, we don't have mechanisms to load automatically the input in the XML description
 *      Using XML ENTITY could be a good way to do this.
 */

template<class RosMessage>
class FMatrixPub : public FMatrix
{
	protected : 

		IString topic_name;
                ISInput size_queue;

		ros::Publisher pub;

	public : 

		FMatrixPub(unsigned int shape = NONE_SHAPE) : FMatrix(shape) {}

		virtual void setparameters()
		{
			Kernel::iBind(topic_name,"topic_name", getUuid());
			Kernel::iBind(size_queue,"size_queue", getUuid());
		}

		virtual void prerun()
		{
       			ros::NodeHandle n;
			pub = n.advertise<RosMessage>( topic_name , size_queue()());
		}
};

template<class RosMessage>
class FScalarPub : public FScalar
{
	protected : 

		IString topic_name;
                ISInput size_queue;

		ros::Publisher pub;

	public : 

		virtual void setparameters()
		{
			Kernel::iBind(topic_name,"topic_name", getUuid());
			Kernel::iBind(size_queue,"size_queue", getUuid());
		}

		virtual void prerun()
		{
       			ros::NodeHandle n;
			pub = n.advertise<RosMessage>( topic_name , size_queue()());
		}
};

#endif // __FPUB__
