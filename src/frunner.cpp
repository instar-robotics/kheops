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

#include "frunner.h"

std::map<int, FRunner *> FRunner::runners;

void FRunner::exec()
{
	checkFunction();

	while( ! __is_asking_stop() )
	{
		wait_for_sync();
		if( __is_asking_stop()) continue;

		for(unsigned int i = 0; i < functions.size() ; i++)
		{
			wait_for_produce( functions[i] );
		//TODO : décider si le runner attend la fin de l'execution de la boite successeur 
		//      Si UN seul manager et toutes les functions attachées au RT_TOKEN inutile
		//      Si des functions peuvent ne pas être reliés au RT_TOKEN : 
		//              - Permet de synchroniser ou de ne pas synchroniser leurs executions
		//              - Les deux comportements peuvent être intéressant.
		//      wait_for_consume(**it);

		// Check Here NULL Pointer ? 
			boost::get(boost::vertex_function , *g)[functions[i]]->compute();
			consume(functions[i]);
			produce(functions[i]);
		}

	}
	for(auto it = functions.begin(); it != functions.end(); it++) {produce(*it);}
}

void FRunner::checkFunction()
{
	for(auto it = functions.begin(); it != functions.end(); it++)
	{
		if( ( boost::get( boost::vertex_function ,*g  )[*it]) == NULL) {
			 throw  std::invalid_argument("FRunner "+std::to_string(id)+" : Function uninitialized for node : "+std::to_string(*it));
		}
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

void FRunner::spawn_all()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                it->second->spawn();
        }
}

void FRunner::join_all()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                it->second->join();
        }
}

void FRunner::sync_all()
{
        for(unsigned int i = 0 ; i < FRunner::runners.size(); i++)
        {
                FRunner::runners[i]->sync();
        }
}

int FRunner::add(FRunner * runner)
{
        if( runner == NULL) throw std::invalid_argument( "FRunner : try to add NULL runner");

        int idr = runners.size();
        runner->setId(idr);
        runners[idr] = runner;
        return idr;
}

FRunner * FRunner::get(int id)
{
        if( runners.find(id) != runners.end() ) return runners[id];
        else return NULL;
}

void FRunner::clear()
{
        for( auto it = runners.begin(); it != runners.end(); it++)
        {
                delete(it->second);
        }
        runners.clear();
}

