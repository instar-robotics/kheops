#ifndef __FEXAMPLE_HPP__
#define __FEXAMPLE_HPP__

#include "function.h"
#include "kernel.h"
#include <iostream>

class Example : public FScalar
{
	private :

		std::string str;
		
		ISInput u_is;
		ISInput m_is;

		ISMInput u_ism;
		ISMInput m_ism;

		IMInput u_im;
		IMInput m_im;

		IMMInput u_imm;
		IMMInput m_imm;
	
        public :

		virtual void compute();
		virtual void setparameters();

};

#endif // __FEXAMPLE_HPP__
