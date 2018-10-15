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

