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


#include "kheops/ros/rosutil.h"
#include <regex>

void RosUtil::clean_topic_name(std::string& str)
{
        std::replace( str.begin(), str.end(), ' ', '_');
        std::replace( str.begin(), str.end(), '-', '_');

	str.erase(std::remove(str.begin(), str.end(), '}'), str.end());
	str.erase(std::remove(str.begin(), str.end(), '{'), str.end());
}
