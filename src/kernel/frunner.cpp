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


#include <exception>
#include <iostream>

#include "kheops/kernel/frunner.h"
#include "kheops/kernel/kernel.h"

#include "ros/console.h"

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
			f->kcompute();
		}
		catch(std::exception& e)
		{
			ROS_FATAL_STREAM( "FATAL in Function.compute " << f->getUuid() << ". " <<  e.what() );
			Kernel::ask_quit();
			break;
		}

		if( is_oscillo_active() ) end = std::chrono::system_clock::now();	

		produce(node);

		try{	
			f->kexec_afterCompute();
		}
		catch(std::exception& e)
		{
			ROS_FATAL_STREAM("FATAL in Function.exec_afterCompute " << f->getUuid() << ". " <<  e.what());
			Kernel::ask_quit();
			break;
		}
			
		if( is_oscillo_active() ) 
		{
			std::chrono::duration<double> elapsed_seconds = end-start;
			date_start = start.time_since_epoch().count();
			last_duration = elapsed_seconds.count();
			if( last_duration > maxDuration ) maxDuration = last_duration;
			if( last_duration < minDuration ) minDuration = last_duration;
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
