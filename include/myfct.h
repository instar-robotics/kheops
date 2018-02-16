#ifndef __MY_FCT_HPP__
#define __MY_FCT_HPP__

#include "function.h"
#include <iostream>

class MyFct : public Function
{
	
        public :

		virtual void compute()
		{
			std::cout << "My Fct" << std::endl;
		}

};

#endif // __MY_FCT_HPP__
