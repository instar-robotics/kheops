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

#include "kheops/kernel/rttoken.h"
#include "kheops/kernel/frunner.h"
#include "kheops/ros/rospublisher.h"


RtToken::RtToken() : Runner(),period(0),  publish(false) 
{
	o_pub = new RosOscilloPublisher(1,"oscillo");
	rt_pub = new RosRtTokenOutputPublisher(1, "rt_token"); 
}

RtToken::RtToken(double period) : Runner(),period(period),publish(false)
{ 
	o_pub = new RosOscilloPublisher(1,"oscillo");
	rt_pub = new RosRtTokenOutputPublisher(1, "rt_token"); 
}

RtToken::RtToken(double value, std::string unit) : Runner(), publish(false)
{
	setToken(value,unit); 
	o_pub = new RosOscilloPublisher(1,"oscillo");
	rt_pub = new RosRtTokenOutputPublisher(1, "rt_token"); 
}


RtToken::~RtToken()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() ) o_pub->close();	
		delete(o_pub);
	}
	
	if( rt_pub != NULL )
	{
		if( rt_pub->is_open() ) rt_pub->close();
		delete(rt_pub);
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
	while( ! Runner::is_asking_stop() )
	{
		if( Runner::is_asking_pause())
		{	
			sync_all();
			pause();
			Runner::wait_ask_resume();
			resume();
		}

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
			publish_oscillo();
		}
	
		if( is_publish_active() )
		{
			publish_message();
		}
	}
	sync_all();
	stop();
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

void RtToken::active_oscillo(bool state)
{
	if(state) 
	{
		if( o_pub != NULL ) 
		{
			if( !o_pub->is_open()) o_pub->open();	

			Runner::active_oscillo(true);
		}
		else throw std::invalid_argument("RtToken : failed to open oscillo publisher");
	}
	else
	{
		Runner::active_oscillo(false);

		if( o_pub != NULL) 
		{
			if( o_pub->is_open() )  o_pub->close();
		}
	}
}

void RtToken::publish_oscillo()
{
	if( o_pub == NULL ) return;
	if( !o_pub->is_open() ) return;

	o_pub->clear();
	
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(*g);
        for( ; it.first != it.second; ++it.first)
        {
            Runner *r = boost::get(boost::vertex_runner, *g, *it.first);

            if( r == this)
            {
	    	OscilloMessage om( getUuid() );
		om.period = getPeriod();
		om.means = getMeanDuration();
		om.sleep = getLastSleep();
		om.duration = getLastDuration();
		om.start = getLastStart(); 

		if( om.duration >= om.period ) om.warning = true;
		else  om.warning = false; 
            
		o_pub->add( om );	
            }
	    else
	    {
		Function * f = boost::get(boost::vertex_function, *g, *it.first);
		
	    	OscilloMessage om( f->getUuid() );
		om.period = getPeriod();
		om.means = r->getMeanDuration();
		om.sleep = r->getLastSleep();
		om.duration = r->getLastDuration();
		om.start = r->getLastStart(); 
		om.warning = false;
            	
		o_pub->add( om );	
	    }
        }
	o_pub->publish();
}

void RtToken::active_publish(bool state) 
{
	if( state )
	{
		if( rt_pub != NULL && !publish)
		{
			if( !rt_pub->is_open()) rt_pub->open();
		}
		else throw std::invalid_argument("RtToken : failed to open rt_token output publisher");
	}
	else
	{
		if( rt_pub != NULL && publish) 
		{
		 	if( rt_pub->is_open()) rt_pub->close();
		}
	}
	publish = state;
}

void RtToken::publish_message()
{
	OscilloMessage om( getUuid() );
	om.period = getPeriod();
	om.means = getMeanDuration();
	om.sleep = getLastSleep();
	om.duration = getLastDuration();
	om.start = getLastStart(); 

	if( om.duration >= om.period ) om.warning = true;
	else  om.warning = false; 
    
	if( rt_pub != NULL )
	{
		if( rt_pub->is_open() )
		{
			rt_pub->setMessage( om );	
			rt_pub->publish();	
		}
	}
}

