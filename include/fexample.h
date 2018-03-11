#ifndef __FEXAMPLE_HPP__
#define __FEXAMPLE_HPP__

#include "function.h"
#include "kernel.h"
#include <iostream>

class FExample : public FScalar
{
	private :

		IScalar is;
		IScalarMatrix ism;
		std::string str;
		
		ISAnchor isanc;
		ISMAnchor ismanc;
		IMMAnchor immanc;
	
        public :

		virtual void compute();
		virtual void setparameters();

};

#endif // __FEXAMPLE_HPP__
