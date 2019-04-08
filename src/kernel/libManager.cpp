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


#include <iostream>
#include <dlfcn.h>
#include <boost/algorithm/string.hpp>
#include "kheops/kernel/libManager.h"
#include "kheops/util/util.h"

LibManager LibManager::singleton;

void LibManager::init(std::string libdir)
{
	singleton.libdir = libdir;
}


void LibManager::load_lib(const std::string& name)
{
	void *handle;
	auto it = libs.find(name);

	if( it != libs.end())
	{
		std::cout << "Load lib : " << it->first << std::endl;
		handle = dlopen (  it->second.c_str() , RTLD_LAZY);
		if (!handle)
		{
		    std::cerr <<  dlerror() << std::endl;
		    exit(1);
		}
	}
	else  throw std::invalid_argument("LibManager : unable to find "+name+" library ");
}


void LibManager::load_libs()
{
  std::vector<std::string> dirs;
  boost::algorithm::split(dirs, libdir, boost::algorithm::is_any_of(":"));	

  for(auto const& dir: dirs)
  {
  	std::vector<std::string> files;
  	getdir (dir, files);

  	std::cout << "Load librairies in " << dir << std::endl; 

  	for(auto const& file: files) 
  	{
		std::string file_extension,file_name,file_path;
	
		get_file_extension( file, file_extension );
		get_file_name(file, file_name);

		boost::replace_first(file_name ,"lib","");
	
		//check .so
		if( file_extension == libExt ) 
		{
			file_path = (dir+file);
			std::cout << "Find Lib : "  << file_name << " in "<< file_path << std::endl;
			libs[file_name] = file_path;
		}
	}
  }
}
