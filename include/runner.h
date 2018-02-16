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

#ifndef  __RUNNER_H__
#define  __RUNNER_H__

#include "graph.h"
#include "link.h"
#include <thread>
#include <condition_variable>

enum STATE { STOP=0, PAUSE=1, RUN=2 };

class Runner
{
        protected :

                int id;
                std::thread thx;
                Graph const *g;

                static int state;
                static std::mutex mtx;
                static std::condition_variable cv;

                static inline bool __is_running() { return state==RUN;}
                static inline bool __is_stop() { return state==STOP;}

	public :

                Runner() : id(-1), g(NULL) {}
                Runner(int id) : id(id), g(NULL){}

                virtual ~Runner(){}

                inline void setGraph(Graph * g){ this->g=g;}
		inline void setId(int id){  this->id = id; }
		inline int getId(){ return id; }

		// Payload of the runner
                virtual void exec() = 0;

		void wait_for_produce(const Graph::vertex_descriptor & v_mtx)
                {
                        for( auto it =  boost::in_edges(v_mtx, *g); it.first != it.second; ++it.first)
                        {
                                get(boost::edge_weight, *g) [*it.first ]->wait_for_produce();
                        }
                }

                void wait_for_consume(const Graph::vertex_descriptor & v_mtx)
                {
                        for( auto it =  boost::out_edges(v_mtx, *g); it.first != it.second; ++it.first)
                        {
                                get(boost::edge_weight, *g) [*it.first ]->wait_for_consume();
                        }
                }

                void produce(const Graph::vertex_descriptor & v_mtx)
                {
                        for( auto it =  boost::out_edges(v_mtx, *g); it.first != it.second; ++it.first)
                        {
                                get(boost::edge_weight, *g) [*it.first ]->produce();
                        }
                }

                void consume(const Graph::vertex_descriptor & v_mtx)
                {
                        for( auto it =  boost::in_edges(v_mtx, *g); it.first != it.second; ++it.first)
                        {
                                get(boost::edge_weight, *g) [*it.first ]->consume();
                        }
                }

		void spawn() {
                    thx = std::thread( [=] { exec(); } );
                }
		
		std::thread & getThread() {
                        return thx;
                }

                static void wait_for_running()
                {
                        {
                                std::unique_lock<std::mutex> lk(Runner::mtx);
                                Runner::cv.wait(lk,   [=] {return  (Runner::__is_running() || Runner::__is_stop());}  );
                        }
                }

                static void change_state(int state)
                {
                        {
                                std::unique_lock<std::mutex> lk(Runner::mtx);
                                Runner::state = state;
                        }
                        Runner::cv.notify_all();
                }

                static inline void stop()
                {
			change_state(STOP);
                }

                static inline void pause()
                {
                        change_state(PAUSE);
                }

                static inline void resume()
                {
                        change_state(RUN);
                }

                static inline int getState(){ return state;}

};

#endif //__RUNNER_H__
