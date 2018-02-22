/*
Copyright Enacted Robotics

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
#include <iostream>

RtToken RtToken::singleton;

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

		FRunner::sync_all();

//		produce(rt_node);
		wait_for_produce(rt_node);
		consume(rt_node);

		auto end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end-start;

		if( elapsed_seconds.count() > period  )
		{
			std::cout << "Warning : RT_Token timeout. Waited : " <<  getMsPeriod()  << " ms (freq="<< convert_period_frequency(period) <<" Hz). Reel : " << convert_s_to_ms(elapsed_seconds.count()) << " ms " << std::endl;
		}
		else
		{
			double sleep_duration = period - elapsed_seconds.count();
			usleep( sleep_duration * CONV_S_TO_MS );

			std::cout << "RT_Token OK (freq=" << convert_period_frequency(period) << "Hz) :" << convert_s_to_ms(elapsed_seconds.count()) << " ms (real freq=" << convert_period_frequency(elapsed_seconds.count())  << " Hz).  Sleep duration :  " <<   convert_s_to_ms(sleep_duration) << " ms" << std::endl;
		}

		means+= elapsed_seconds.count();
		nbrun++;
	}
	FRunner::sync_all();
	//produce(rt_node);
	// ICI possibilité d'ajouter un wait avec un time out 
	// identifier les liens qui ne se terminent pas : donnent de l'infos sur la branche bloquée
	consume(rt_node);
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

