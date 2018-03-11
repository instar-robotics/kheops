#ifndef __DEMO_FCT_HPP__
#define __DEMO_FCT_HPP__

#include "function.h"
#include "kernel.h"
#include <iostream>

class DemoFct : public FScalar
{
	private : 

		IScalar iscal;

        public :
		
		DemoFct() : FScalar() {}

		virtual void compute()
		{
			std::cout << "demo Box : " <<  iscal() << std::endl;
		}

                virtual void setparameters()
		{
			Kernel::instance().bind(iscal,"iscal", getUuid());
		}


};


#endif // __DEMO_FCT_HPP__
