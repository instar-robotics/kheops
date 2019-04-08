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


#ifndef __WEIGHT_CONVERTER__
#define __WEIGHT_CONVERTER__

#include <map>
#include <string>
#include "kheops/kernel/inputbase.h"
#include "kheops/kernel/ilinkbase.h"

const int SPARSE = 0;
const int  DENSE = 1;

class WeightConverter
{
	private:
			
		std::string file;	

	public : 
		WeightConverter(const std::string& path) : file(path) {}
		~WeightConverter(){}

		void load(std::map<std::string, InputBase*> &inputs, bool ignore_check_size = false);
		void save(std::map<std::string, InputBase*> &inputs);
};

#endif // __RES_CONVERTER__
