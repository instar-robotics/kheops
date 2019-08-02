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

#ifndef __FSUB__
#define __FSUB__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include <ros/ros.h>


/* Note :
 *      1- Each FMatrixSub or FScalarSub object has 3 default Kheops Input:
 *      - IString topic_name : for the topic Name
 *      - ISInput size_queue : define the size of the queue
 *      - ISInput sleep      : define the behavior of the Function [blocking Function if sleep < 0 or non-blocking Function and time to wait if sleep >= 0]
 *
 *      2- This 3 inputs MUST BE BIND to the kernel in the method setparameters.
 *      If you extend setparameters to add other Inputs, don't forget to call FMatrixSub::setparameters or FScalarSub::setparameters ! 
 *
 *      3- And Most important : don't forget to add this Inputs in the XML description !
 *      For now, we don't have mechanisms to load automatically the input in the XML description
 *      Using XML ENTITY could be a good way to do this.
 */

template<class RosMessage>
class FMatrixSub :  public FMatrix, public RosSubscriber<RosMessage>
{
        protected :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :
                
		FMatrixSub<RosMessage>(unsigned int shape = NONE_SHAPE) : FMatrix(shape), RosSubscriber<RosMessage>() {}

                virtual void onQuit()  {RosSubscriber<RosMessage>::disable();}
                virtual void onPause() {RosSubscriber<RosMessage>::disable();}
                virtual void onRun()   {RosSubscriber<RosMessage>::enable(topic_name, (int)(size_queue()()) );}
                virtual void compute() {RosSubscriber<RosMessage>::callOne(sleep()());}

                virtual void setparameters()
                {       
                        Kernel::iBind(topic_name,"topic_name", getUuid());
                        Kernel::iBind(size_queue,"size_queue", getUuid());
                        Kernel::iBind(sleep,"sleep", getUuid());
                }
};

template<class RosMessage>
class FScalarSub :  public FScalar, public RosSubscriber<RosMessage>
{
        protected :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :
		
		FScalarSub<RosMessage>() : FScalar(), RosSubscriber<RosMessage>() {}
                
                virtual void onQuit()  {RosSubscriber<RosMessage>::disable();}
                virtual void onPause() {RosSubscriber<RosMessage>::disable();}
                virtual void onRun()   {RosSubscriber<RosMessage>::enable(topic_name, (int)(size_queue()()) );}
                virtual void compute() {RosSubscriber<RosMessage>::callOne(sleep()());}

                virtual void setparameters()
                {       
                        Kernel::iBind(topic_name,"topic_name", getUuid());
                        Kernel::iBind(size_queue,"size_queue", getUuid());
                        Kernel::iBind(sleep,"sleep", getUuid());
                }
};

#endif // __FSUB__
