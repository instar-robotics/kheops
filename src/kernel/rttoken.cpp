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


#include "kheops/kernel/rttoken.h"
#include "kheops/kernel/frunner.h"
#include "kheops/ros/rospublisher.h"


RtToken::RtToken() : Runner(),period(0),last_sleep(0) , publish(false)
{
	o_pub = new RosOscilloPublisher(1);
	rt_pub = new RosRtTokenOutputPublisher(1); 
}

RtToken::RtToken(double period) : Runner(),period(period),last_sleep(0) , publish(false)
{ 
	o_pub = new RosOscilloPublisher(1);
	rt_pub = new RosRtTokenOutputPublisher(1); 
}

RtToken::RtToken(double value, std::string unit) : Runner(), period(0),last_sleep(0) , publish(false)
{
	setToken(value,unit); 
	o_pub = new RosOscilloPublisher(1);
	rt_pub = new RosRtTokenOutputPublisher(1); 
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
	if( Runner::is_asking_pause())
        {
                pause();
                Runner::wait_ask_resume();
        }
        resume();


	while( ! Runner::is_asking_stop() )
	{
		if( Runner::is_asking_pause())
		{	
			// HERE : perhaps better to pause before sync_all ? 
			sync_all();
			pause();

			Runner::wait_ask_resume();
			resume();
		}
		else resume();

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

		date_start = std::chrono::duration_cast<std::chrono::milliseconds>( start.time_since_epoch()).count();

		last_duration = elapsed_seconds.count();
		if( last_duration > maxDuration ) maxDuration = last_duration;
		if( last_duration < minDuration ) minDuration = last_duration;
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
		
            if( r != this && r!= NULL) 
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
	    	RtTokenMessage om( getUuid() );
		om.period = getPeriod();
		om.means = getMeanDuration();
		om.sleep = getLastSleep();
		om.duration = getLastDuration();
		om.start = getLastStart(); 
		om.minDuration = getMinDuration();
		om.maxDuration = getMaxDuration();

		if( om.duration >= om.period ) om.warning = true;
		else  om.warning = false; 
            
		o_pub->addRt( om );	

            }
	    else
	    {
		Function * f = boost::get(boost::vertex_function, *g, *it.first);
		
		if( f != NULL )
		{
	    		OscilloMessage om( f->getUuid() );
			om.means = r->getMeanDuration();
			om.duration = r->getLastDuration();
			om.start = r->getLastStart(); 
			om.minDuration = r->getMinDuration();
			om.maxDuration = r->getMaxDuration();
            	
			o_pub->add( om );	
		}
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
	RtTokenMessage om( getUuid() );
	om.period = getPeriod();
	om.means = getMeanDuration();
	om.sleep = getLastSleep();
	om.duration = getLastDuration();
	om.start = getLastStart(); 
	om.minDuration = getMinDuration();
	om.maxDuration = getMaxDuration();

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

void RtToken::set_rt_pub_name(const std::string& name)
{
	std::string pname = name;
	ComInterface::setDefaultName(pname);
	rt_pub->setPubName(pname);
}

void RtToken::set_oscillo_pub_name(const std::string& name)
{
	std::string pname = name;
	ComInterface::setDefaultName(pname);
	o_pub->setPubName(pname);
}

