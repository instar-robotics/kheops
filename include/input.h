/*
Copyright Enacted Robotics

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

#ifndef __INPUT_H__
#define __INPUT_H__

#include <memory>
#include "ilink.h" 

template<class I>
class Input
{
	protected: 

		std::vector<std::weak_ptr<I>> inputs;
		
		bool multiple;

	public : 
		Input() : multiple(false) {}
		Input(bool m) : multiple(m) {}
		virtual ~Input() {inputs.clear();}

		inline bool isMultiple(){return multiple;} 
		inline void setMultiple(bool m){multiple=m;} 

		void add( std::shared_ptr<I> &i) 
		{ 
			if( i == NULL)   throw std::invalid_argument("Input : try to add NULL input");

			if( !multiple &&  inputs.size() >= 1) throw std::invalid_argument("Input : try to add more than one ilink on unique type input");
				
			inputs.push_back( std::weak_ptr<I>(i) );
		}

	
		void purge_empty()
		{
			for( auto it = inputs.begin(); it != inputs.end (); it++)
			{
				if( (*it).lock() == NULL ) inputs.erase(it); 
			}
		}
		

		unsigned int size(){return inputs.size();}

		I& operator[](unsigned int i){ 
			if( i > inputs.size()  )  throw std::invalid_argument("Input : out of number ilink");
			return *(inputs[i].lock());
		}

		I& i(){ 
			if( inputs.size() == 0 )  throw std::invalid_argument("Input : out of number ilink");
			return *(inputs[0].lock());
		}
		
		I& operator()(){ 
			if( inputs.size() == 0 )  throw std::invalid_argument("Input : out of number ilink");
			return *(inputs[0]).lock();
		}

		size_t type() { return typeid(I).hash_code();}
};

class IMMInput : public Input<IMMatrix>
{
	public : 
		IMMInput() : Input(true) {}
		virtual ~IMMInput() {}
};

typedef Input<IScalar> ISInput;
typedef Input<IMatrix> IMInput;
typedef Input<IScalarMatrix> ISMInput;

#endif // __INPUT_H__
