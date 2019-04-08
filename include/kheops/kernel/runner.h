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


#ifndef  __RUNNER_H__
#define  __RUNNER_H__

#include <thread>
#include <uuid/uuid.h>
#include "kheops/kernel/kstate.h"
#include "kheops/kernel/graph.h"
#include "kheops/kernel/klink.h"

class Runner
{
        protected :
		
		std::thread::native_handle_type handle;
                std::thread thx;
                Graph const *g;
		Graph::vertex_descriptor node;

		//TODO : pour l'instant pas de gestion de rebouclage du compteur
                unsigned long long nbrun;
                double means;    
                double last_sleep;
                double last_duration;
		double date_start;
	
		std::mutex s_mtx;
                std::condition_variable s_cv;
		int state;

		static bool oscillo;
		static int request;
                static std::mutex r_mtx;
                static std::condition_variable r_cv;

		
	public :

                Runner() : g(NULL),nbrun(0),means(0),last_sleep(0),last_duration(0),date_start(0),state(K_PAUSE) {}

                virtual ~Runner() = 0;
                inline void setGraph(Graph * g){this->g=g;}

		inline void setNode( Graph::vertex_descriptor node ) {this->node = node;}
                inline Graph::vertex_descriptor  getNode() { return node;}

		inline double getLastStart(){ return date_start;}
		inline double getLastDuration(){ return last_duration;}
                inline double getLastSleep() { return last_sleep;}
                inline double getMeanDuration() { if( nbrun == 0) return 0;  return means/nbrun;}

		// Payload of the runner
                virtual void exec() = 0;

                void produce(const Graph::vertex_descriptor v_mtx);
                void consume(const Graph::vertex_descriptor v_mtx);

		inline void spawn() { 
			thx = std::thread( [=] { exec(); } );
			handle = thx.native_handle();
		}
		
		void terminate();
		inline void join() {thx.join();}
		inline bool joinable() { return thx.joinable();}
		inline std::thread & getThread() {return thx;}

		inline int getState(){ return state;}
		inline bool is_run() { return state==K_RUN;}
                inline bool is_stop() { return state==K_STOP;}
                inline bool is_pause() { return state==K_PAUSE;}

		void change_state(int state);
		inline void stop() {change_state(K_STOP);}
                inline void pause() {change_state(K_PAUSE);}
                inline void resume() {change_state(K_RUN);}

		void wait_for_pause();
                void wait_for_quit();
                void wait_for_run();
		
		// duration in milliseconds
		bool wait_for_pause_timeout(unsigned int duration);
                bool wait_for_quit_timeout(unsigned int duration);
                bool wait_for_run_timeout(unsigned int duration);

		// Static parts
		static inline bool is_asking_stop() {return request == K_STOP;}
                static inline bool is_asking_running() {return request== K_RUN;}
                static inline bool is_asking_pause() {return request== K_PAUSE;}

                static void wait_ask_resume();
                static void change_request(int request);
                static inline void ask_stop() {change_request(K_STOP); }
                static inline void ask_pause() {change_request(K_PAUSE);}
                static inline void ask_resume() {change_request(K_RUN); }
                static inline int getRequest(){ return request;}


		static inline bool is_oscillo_active(){return oscillo;}
                static inline void active_oscillo(bool state) {oscillo = state;}
};

#endif //__RUNNER_H__
