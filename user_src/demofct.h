#ifndef __DEMO_FCT_HPP__
#define __DEMO_FCT_HPP__

#include "function.h"
#include <iostream>

class DemoFct : public Function
{
        public :
		
		DemoFct() : Function() {}

		virtual void compute()
		{
			std::cout << "demo Box" << std::endl;
		}

};


#endif // __DEMO_FCT_HPP__
