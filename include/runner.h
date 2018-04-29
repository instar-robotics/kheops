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

#ifndef  __RUNNER_H__
#define  __RUNNER_H__

#include "graph.h"
#include "klink.h"
#include <thread>
#include <uuid/uuid.h>

enum STATE { R_STOP=0, R_PAUSE=1, R_RUN=2 };

class Runner
{
        protected :
		
                std::thread thx;
                Graph const *g;
		Graph::vertex_descriptor node;

		//TODO : pour l'instant pas de gestion de rebouclage du compteur
                unsigned long long nbrun;
                double means;    
                double last_sleep;
                double last_duration;
		double date_start;
	
		static bool oscillo;
		static int request;
		
		inline bool __is_asking_stop() {return Runner::request == R_STOP; }
		inline bool __is_asking_running() {return Runner::request==R_RUN;}
                inline bool __is_asking_pause() {return Runner::request==R_PAUSE;}

	public :

                Runner() : g(NULL),nbrun(0),means(0),last_sleep(0),last_duration(0),date_start(0) { }

                virtual ~Runner(){}
                inline void setGraph(Graph * g){ this->g=g;}

		inline void setNode( Graph::vertex_descriptor node ) { this->node = node;}
                inline Graph::vertex_descriptor  getNode() { return node;}

		inline double getLastStart(){ return date_start;}
		inline double getLastDuration(){ return last_duration;}
                inline double getLastSleep() { return last_sleep;}
                inline double getMeanDuration() { if( nbrun == 0) return 0;  return means/nbrun;}

		// Payload of the runner
                virtual void exec() = 0;
		virtual void terminate() = 0;

                void produce(const Graph::vertex_descriptor  v_mtx);
                void consume(const Graph::vertex_descriptor  v_mtx);

		inline void spawn() {thx = std::thread( [=] { exec(); } );}
		inline void join() {thx.join();}
		inline std::thread & getThread() {return thx;}

                static inline int getRequest(){ return request;}
		static inline bool is_oscillo_active(){return oscillo;}
                static inline void active_oscillo(bool state) {oscillo = state;}
};

#endif //__RUNNER_H__
