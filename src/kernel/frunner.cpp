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

#include <exception>
#include <iostream>

#include "kheops/kernel/frunner.h"
#include "kheops/kernel/kernel.h"

void FRunner::exec()
{
	std::chrono::time_point<std::chrono::system_clock> start,end;

	checkFunction();
			
	Function * f = boost::get(boost::vertex_function , *g)[node] ;
		
	if( Runner::is_asking_pause()) 
	{
		pause();
		Runner::wait_ask_resume();
	}
	
	resume();

	while( ! Runner::is_asking_stop() )
	{
		wait_for_sync();
		if( Runner::is_asking_stop()) continue;
		
		if( Runner::is_asking_pause()) 
		{
			pause();
                	Runner::wait_ask_resume();
                	resume();
			wait_for_sync();
		}
		
		consume(node);
		
		if(is_oscillo_active()) start = std::chrono::system_clock::now();

		try{	
			f->compute();
		}
		catch(std::exception& e)
		{
			std::cerr << "FATAL in Function.compute " << f->getUuid() << ". " <<  e.what() << std::endl;
			Kernel::ask_quit();
			continue;
		}

		if( is_oscillo_active() ) end = std::chrono::system_clock::now();	

		produce(node);

		try{	
			f->kexec_afterCompute();
		}
		catch(std::exception& e)
		{
			std::cerr << "FATAL in Function.exec_afterCompute " << f->getUuid() << ". " <<  e.what() << std::endl;
			Kernel::ask_quit();
			continue;
		}
			
		if( is_oscillo_active() ) 
		{
			std::chrono::duration<double> elapsed_seconds = end-start;
			date_start = start.time_since_epoch().count();
			last_duration = elapsed_seconds.count();
			means+= elapsed_seconds.count();
			nbrun++;
		}
	}
	produce(node);
	stop();
}

void FRunner::checkFunction()
{
	if( ( boost::get( boost::vertex_function ,*g  )[node]) == NULL) {
		 throw  std::invalid_argument("FRunner : Function uninitialized");
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
