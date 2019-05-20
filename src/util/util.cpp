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
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include "kheops/util/util.h"

void getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
	std::stringstream buf ;
	buf << "Error(" << errno << ") opening " << dir ;
	throw std::invalid_argument(buf.str());
    }

    while ((dirp = readdir(dp)) != NULL) {

        std::string file = dirp->d_name;

        if( file != ".."  && file != ".") files.push_back( std::string(dirp->d_name));
    }
    closedir(dp);
}

bool check_file_extension(const std::string& path, const std::string& extension )
{
	boost::filesystem::path p(path);
      
	if( extension == p.extension().string()) return true;
	return false;
}

void get_file_extension(const std::string& path, std::string& extension)
{
	boost::filesystem::path p(path);
        extension = p.extension().string();
}

void get_file_name(const std::string& path, std::string& filename)
{
	boost::filesystem::path p(path);
	filename = p.stem().string();
}

double convert_s_to_micros(double value)
{
        return value * 1000000;
}

double convert_s_to_ms(double value)
{
        return value * 1000;
}

double convert_ms_to_s(double value)
{
        return value / 1000;
}

double convert_period_frequency(double value)
{
        return 1/value;
}


std::string generate_uuid()
{
	uuid_t out;
        uuid_generate(out);

        char cuuid[37];
        uuid_unparse(out,cuuid);
        std::string uuid(cuuid);

	return uuid;
}

