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
