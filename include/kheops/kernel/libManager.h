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


#ifndef __LIB_MANAGER_H__
#define __LIB_MANAGER_H__

#include <map>
#include <string>

const std::string libExt = ".so";

class LibManager 
{
	private :
		// string 1 : lib name. string 2 : lib path
		std::map<std::string, std::string> libs;
		std::string libdir;
		
		static LibManager singleton;

	public :
	
		LibManager(){}
		LibManager(std::string libdir) : libdir(libdir) {}
		~LibManager(){}

		static void init(std::string libdir);
		static inline LibManager& instance() noexcept {return singleton;}
		static inline void load(){ singleton.load_libs(); }
		static inline void load(const std::string& name){ singleton.load_lib(name); }

		void load_lib(const std::string &name);			
		void load_libs();			
};

#endif // __KERNEL_H__
