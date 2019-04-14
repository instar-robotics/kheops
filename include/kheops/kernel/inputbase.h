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


#ifndef __INPUT_H__
#define __INPUT_H__

#include <string>
#include <memory>
#include "kheops/kernel/ilinkbase.h" 

typedef std::string IString;

class InputBase
{
	protected :

		std::string uuid;
		bool multiple;
		bool checkSize;

	public : 
		InputBase() : multiple(false), checkSize(true) {}
		virtual ~InputBase() = 0;
		
		inline const std::string& getUuid() { return uuid; }
                inline void setUuid(const std::string& uuid  ) { this->uuid = uuid;}
		
		inline bool isMultiple(){return multiple;} 
		inline void setMultiple(bool m){multiple=m;} 

		inline bool isCheckSize(){return checkSize;} 
		inline void setCheckSize(bool s){checkSize=s;} 

		virtual void clear() = 0;
		virtual void add( std::shared_ptr<iLinkBase> &i) = 0;
		virtual void purge_empty() = 0;
		
		virtual unsigned int size() = 0; 
		
		virtual size_t type() = 0 ;
		virtual std::string type_name() =0 ;

		virtual iLinkBase& operator[](unsigned int i) = 0; 

};

template<class I>
class Input : public InputBase
{
	protected: 

		std::vector<std::weak_ptr<I>> ilinks;

	public : 
		Input() {}
		virtual ~Input() {ilinks.clear();}

		virtual void clear(){ ilinks.clear(); }

		virtual void add(std::shared_ptr<iLinkBase> &i)
		{ 
			if( i == NULL) throw std::invalid_argument("Input : try to add NULL link on input. Input UUID : "+uuid);
			if( typeid(*i).hash_code() != type() ) throw std::invalid_argument("Input : try to add wrong iLink type into Input "+uuid+" : "+typeid(*i).name()+" given, "+type_name()+" expected" ); 

			if( !multiple &&  ilinks.size() >= 1) throw std::invalid_argument("Input : try to add more than one ilink on unique type input. Input UUID : "+uuid);

			ilinks.push_back( std::weak_ptr<I>( std::static_pointer_cast<I>(i) ));
		}
	
		virtual void purge_empty()
		{
			for( auto it = ilinks.begin(); it != ilinks.end (); it++)
			{
				if( (*it).lock() == NULL ) ilinks.erase(it); 
			}
		}

		virtual unsigned int size(){return ilinks.size();}
		virtual size_t type() { return typeid(I).hash_code();}
		virtual std::string type_name() { return typeid(I).name();}
		

		// Kernel operator to acces to Base class
		virtual iLinkBase& operator[](unsigned int i)
		{
			if( i >= ilinks.size()  )  throw std::invalid_argument("Input : out of number ilink");
			return *(ilinks[i].lock());
		}

		I& operator()(unsigned int i){ 
			if( i >= ilinks.size()  )  throw std::invalid_argument("Input : out of number ilink");
			return *(ilinks[i].lock());
		}
		
		I& i(){ 
			if( ilinks.size() == 0 )  throw std::invalid_argument("Input : out of number ilink");
			return *(ilinks[0].lock());
		}


		I& i(unsigned int i){ 
			if( i >= ilinks.size()  )  throw std::invalid_argument("Input : out of number ilink");
			return *(ilinks[i].lock());
		}
		
		
		I& operator()(){ 
			if( ilinks.size() == 0 )  throw std::invalid_argument("Input : out of number ilink");
			return *(ilinks[0]).lock();
		}
};

#endif // __INPUT_H__
