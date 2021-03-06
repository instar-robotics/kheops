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


#include "kheops/links/iscalar.h"
#include "kheops/ros/rospublisher.h"


/********************************************************************************************************/
/******************                     iScalar Section                           *******************/
/********************************************************************************************************/

iScalar::iScalar() : iLink()
{
        o_pub = new RosScalarPublisher(1);
}


iScalar::iScalar(SCALAR const * i) : iLink(i)
{
        o_pub = new RosScalarPublisher(1);
}

iScalar::~iScalar()
{
        if( o_pub != NULL )
        {
                if( o_pub->is_open() ) o_pub->close();
                delete(o_pub);
        }
}

