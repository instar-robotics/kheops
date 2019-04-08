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


#ifndef __FACTORY_HPP__
#define __FACTORY_HPP__

#include <string>
#include <map>

#define REGISTER(classname,buildbase) \
   static const BuilderImpl<classname,buildbase> classname_builder(#classname); 

template <class T>
class Builder
{
        public :
                virtual T* build() = 0;
                Builder(const std::string& classname);
};

template <class A,class T>
class BuilderImpl :  public Builder<T> 
{
        public:
                BuilderImpl(const std::string& classname) : Builder<T>(classname){}
                virtual A* build() {return new A();}
};

template<class T>
class Factory 
{
	private :
                std::map<std::string, Builder<T>*> table;

        public :
		
		Factory(){}
		Factory(const Factory&) = delete;
		Factory& operator=(const Factory&) = delete;

                void registerit( const std::string& classname , Builder<T> * builder)
                {
                         table[classname] = builder;
                }

		bool is_register(const std::string& classname)
		{
			return (table.find(classname) != table.end());
		}
	
                T * create(const std::string& classname)
		{
		 typename std::map<std::string, Builder<T>*>::iterator it  = table.find(classname);

		 if (it != table.end())
			return it->second->build();
		 else
			return (T*)NULL;

		}

		static Factory<T> & Instance() noexcept
		{
  		  static Factory<T> factory;
 		  return factory;
		}
};

template<class T>
Builder<T>::Builder(const std::string& classname)
{       
        Factory<T>::Instance().registerit(classname, this);
}

#endif // __FACTORY_HPP__
