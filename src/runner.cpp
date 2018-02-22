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

#include "runner.h"

int Runner::request = PAUSE;
std::map<int, Runner *> Runner::runners;


void Runner::wait_for_produce(const Graph::vertex_descriptor  v_mtx)
{
	for( auto it =  boost::in_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_weight, *g) [*it.first ]->wait_for_produce();
	}
}

void Runner::wait_for_consume(const Graph::vertex_descriptor v_mtx)
{
	for( auto it =  boost::out_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_weight, *g) [*it.first ]->wait_for_consume();
	}
}


void Runner::produce(const Graph::vertex_descriptor  v_mtx)
{
	for( auto it =  boost::out_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_weight, *g) [*it.first ]->produce();
	}
}

void Runner::consume(const Graph::vertex_descriptor  v_mtx)
{
	for( auto it =  boost::in_edges(v_mtx, *g); it.first != it.second; ++it.first)
	{
		boost::get(boost::edge_weight, *g) [*it.first ]->consume();
	}
}

void Runner::wait_for_sync()
{
        {
                std::unique_lock<std::mutex> lk(mtx_sync); 
                cv_sync.wait(lk,[=]{ return bsync;});
        }
}

void Runner::sync()
{
        {
                std::unique_lock<std::mutex> lk(mtx_sync); 
                bsync = true;
        }
        cv_sync.notify_one();
}

void Runner::desync()
{
        {
              std::unique_lock<std::mutex> lk( Runner::mtx_sync);
              bsync = false;
        }
        cv_sync.notify_one();
}



void Runner::spawn_all()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                it->second->spawn();
        }
}

void Runner::join_all()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                it->second->join();
        }
}

void Runner::sync_all()
{
	for(int i = 0 ; i < Runner::runners.size(); i++)
	{
		Runner::runners[i]->sync();
	}
}

int Runner::add(Runner * runner)
{
	if( runner == NULL) throw std::invalid_argument( "Runner : try to add NULL runner");

        int idr = runners.size();
        runner->setId(idr);
        runners[idr] = runner;
        return idr;
}

Runner * Runner::get(int id)
{
	if( runners.find(id) != runners.end() ) return runners[id];
	else return NULL;
}

void Runner::clear()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                delete(it->second);
        }
        runners.clear();
}
