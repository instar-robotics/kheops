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


#include "kheops/kernel/runner.h"

bool Runner::oscillo = false;
int Runner::request = K_PAUSE;
std::mutex Runner::r_mtx;
std::condition_variable Runner::r_cv;

Runner::~Runner(){}

void Runner::produce(const Graph::vertex_descriptor  v_mtx)
{
	for( auto it =  boost::out_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_klink, *g) [*it.first ]->produce();
	}
}

void Runner::consume(const Graph::vertex_descriptor  v_mtx)
{
	for( auto it =  boost::in_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_klink, *g) [*it.first ]->consume();
	}
}


void Runner::change_request(int request)
{
        {
                std::unique_lock<std::mutex> lk(r_mtx);
		Runner::request = request;
        }
        r_cv.notify_all();
}

void Runner::wait_ask_resume()
{
        {
                std::unique_lock<std::mutex> lk(r_mtx);
                r_cv.wait(lk, [=] {  return  !is_asking_pause();}  );
        }
}


void Runner::wait_for_quit()
{
        {
                std::unique_lock<std::mutex> lk(s_mtx);
                s_cv.wait(lk,  [=] {return is_stop();}  );
        }
}

void Runner::wait_for_run()
{
        {
                std::unique_lock<std::mutex> lk(s_mtx);
                s_cv.wait(lk,  [=] {return is_run();}  );
        }
}

void Runner::wait_for_pause()
{
        {
                std::unique_lock<std::mutex> lk(s_mtx);
                s_cv.wait(lk,  [=] {return is_pause();}  );
        }
}


bool Runner::wait_for_quit_timeout(unsigned int duration)
{
	bool ret = false;
        {
                std::unique_lock<std::mutex> lk(s_mtx);
		ret = s_cv.wait_for(lk,std::chrono::milliseconds(duration),  [=] {return is_stop();});
        }
	return ret;
}

bool Runner::wait_for_run_timeout(unsigned int duration)
{
	bool ret = false;
        {
                std::unique_lock<std::mutex> lk(s_mtx);
		ret = s_cv.wait_for(lk,std::chrono::milliseconds(duration),  [=] {return is_run();});
        }
	return ret;
}

bool Runner::wait_for_pause_timeout(unsigned int duration)
{
	bool ret = false;
        {
                std::unique_lock<std::mutex> lk(s_mtx);
		ret = s_cv.wait_for(lk,std::chrono::milliseconds(duration),  [=] {return is_pause();});
        }
	return ret;
}


void Runner::change_state(int state)
{
        {
                std::unique_lock<std::mutex> lk(s_mtx);
                this->state = state;
        }
        s_cv.notify_all();
}

void Runner::terminate()
{
        thx.detach();
        pthread_cancel(handle); 
        thx.~thread();
}

