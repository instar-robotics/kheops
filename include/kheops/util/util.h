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


#ifndef __UTIL_HPP__
#define __UTIL_HPP__

#include <vector>
#include <string>
#include <uuid/uuid.h>

const uuid_t DUUID = {0,0,0,0};

int getdir (std::string dir, std::vector<std::string> &files);

bool check_file_extension(const std::string& path, const std::string& extension);
void get_file_extension(const std::string& path, std::string& extension);
void get_file_name(const std::string& path, std::string& filename);

double convert_s_to_micros(double value);
double convert_s_to_ms(double value);
double convert_ms_to_s(double value);
double convert_period_frequency(double value);

std::string generate_uuid();

template<class T>
inline bool isequal(T x, T y){ return (fabs(x - y) < std::numeric_limits<T>::epsilon());}

template<class T>
T rectification(T x,T thresold)
{
 if(x <= thresold) return 0.;
 return ( x - thresold);
}


#endif // __UTIL_HPP__
