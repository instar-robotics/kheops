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

#include "frunner.h"

void FRunner::exec()
{
	checkFunction();
			
	Function * f = boost::get(boost::vertex_function , *g)[node] ;

	while( ! __is_asking_stop() && ! __is_asking_local_stop() )
	{
		wait_for_sync();
		if( __is_asking_stop() || __is_asking_local_stop()) continue;
		
		consume(node);
		f->exec();
		produce(node);

		f->nsync_afterCompute();
	}
	produce(node);
}

void FRunner::checkFunction()
{
	if( ( boost::get( boost::vertex_function ,*g  )[node]) == NULL) {
		 throw  std::invalid_argument("FRunner "+uuid+" : Function uninitialized");
	}
}

void FRunner::wait_for_sync()
{
        {
                std::unique_lock<std::mutex> lk(mtx_sync);
                cv_sync.wait(lk,[=]{ return bsync;});
                bsync = false;
        }
}

void FRunner::sync()
{
        {
                std::unique_lock<std::mutex> lk(mtx_sync); 
                bsync = true;
        }
        cv_sync.notify_one();
}

void FRunner::terminate()
{
	local_stop();
	sync();
	join();
}
