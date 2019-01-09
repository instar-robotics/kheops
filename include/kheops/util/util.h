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
