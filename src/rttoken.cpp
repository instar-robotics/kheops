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

#include "rttoken.h"
#include "frunner.h"
#include "publisher.h"
#include <iostream>

RtToken::RtToken() : Runner(),period(0), state(PAUSE), output(false) 
{
	o_pub = new RosOscilloPublisher(1);
	//rt_pub = new 
}

RtToken::RtToken(double period) : Runner(),period(period),state(PAUSE),output(false)
{ 
	o_pub = new RosOscilloPublisher(1);
}

RtToken::RtToken(double value, std::string unit) : Runner(),state(PAUSE), output(false)
{
	setToken(value,unit); 
	o_pub = new RosOscilloPublisher(1);
}


RtToken::~RtToken()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() ) o_pub->close();	
		delete(o_pub);
	}
}

void RtToken::setToken(double value, std::string unit)
{
	if( unit == second )
	{
		setPeriod( value );
	}
	else if ( unit == ms )
	{
		setMsPeriod( value);
	}
	else if( unit == hertz )
	{
		setFrequency(value);
	}
	else
	{
		throw std::invalid_argument("Rt Token Unit must be : ["+second+"] : second  or ["+ms+"] : millisecond  or ["+hertz+"] : hertz" );
	}
}


void RtToken::exec()
{
	while( !__is_asking_stop() )
	{
		wait_ask_resume();
		if( __is_asking_stop() ) continue;

		auto start = std::chrono::system_clock::now();

		sync_all();
		consume(node);

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		double sleep_duration = 0;

		if( elapsed_seconds.count() < period  )
		{
			sleep_duration = period - elapsed_seconds.count();
			usleep( sleep_duration * CONV_S_TO_MS );
		}

		date_start = start.time_since_epoch().count();
		last_duration = elapsed_seconds.count();
		last_sleep = sleep_duration;
		means+= elapsed_seconds.count();
		nbrun++;

		if( is_oscillo_active() )
		{
			if( o_pub->is_open() )
			{
				send_oscillo_message();
			}
		}
	
		if( is_output_active() )
		{
			// String OK/Warning
			// Period 
			// duration 
			// sleep	
		}
	}
	sync_all();
	consume(node);
	stop();
}

void RtToken::wait_ask_resume()
{
        {
		if( __is_asking_pause()) pause();
                std::unique_lock<std::mutex> lk(rt_mtx);
                rt_cv.wait(lk, [=] {  return  !__is_asking_pause();}  );
        }
	resume();
}


void RtToken::wait_for_pause()
{
        {
                std::unique_lock<std::mutex> lk(rt_mtx);
                rt_cv.wait(lk,   [=] {return is_pause();  }  );
        }
}

void RtToken::change_request(int request)
{
        {
                std::unique_lock<std::mutex> lk(rt_mtx);
                Runner::request = request;
        }
        rt_cv.notify_all();
}


void RtToken::change_state(int state)
{
        {
                std::unique_lock<std::mutex> lk(rt_mtx);
                this->state = state;
        }
        rt_cv.notify_all();
}

void RtToken::sync_all()
{
        std::pair<vertex_iter, vertex_iter> it = boost::vertices(*g);
        for( ; it.first != it.second; ++it.first)
        {
            Runner *r = boost::get(boost::vertex_runner, *g, *it.first);
		
            if( r != this) 
	    {                
          	dynamic_cast<FRunner*>(r)->sync();
       	    }
	}
}

void RtToken::terminate()
{
	ask_stop();
	join();
}

void RtToken::start_oscillo_publisher()
{
	if( o_pub != NULL ) 
	{
		o_pub->open();	
	}
	else throw std::invalid_argument("RtToken : failed to open oscillo publisher");
}


void RtToken::stop_oscillo_publisher()
{
	if( o_pub != NULL) 
	{
		o_pub->close();
	}
}

void RtToken::send_oscillo_message()
{
//	 num_vertices(graph);
	o_pub->clear();
	
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(*g);
        for( ; it.first != it.second; ++it.first)
        {
            Runner *r = boost::get(boost::vertex_runner, *g, *it.first);

            if( r == this)
            {
        	o_pub->add( getUuid(), getPeriod(), getMeanDuration(), getLastSleep(), getLastDuration() , getLastStart()  );	
            }
	    else
	    {
		Function * f = boost::get(boost::vertex_function, *g, *it.first);
        	o_pub->add( f->getUuid(), getPeriod(), r->getMeanDuration(), r->getLastSleep(), r->getLastDuration() , r->getLastStart()  );	

	    }
        }
	o_pub->publish();
}
