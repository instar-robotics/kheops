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

#ifndef __LINK_H__
#define __LINK_H__

#include <condition_variable>


class Link{

	public : 
		
		Link(){}
		virtual ~Link(){}

		virtual void produce() = 0;
		virtual void consume() = 0;
		virtual void wait_for_produce() = 0;	
		virtual void wait_for_consume() = 0;	

};

class Passing_Link : public Link
{
        public:
                Passing_Link(){}
                virtual ~Passing_Link(){}

                virtual inline void produce() {}
                virtual inline void consume() {}
                virtual inline void wait_for_produce() {}
                virtual inline void wait_for_consume() {}
};

class Synchronized_Link : public Link
{
        private :
                std::mutex mtx;
                std::condition_variable cv;
                bool state;

                bool __is_produce() { return state;}
                bool __is_consume() { return state == false;}
        public :
                Synchronized_Link() : Link(),state(false)  {}
                Synchronized_Link(bool state) : Link(), state(state)  {}
                ~Synchronized_Link(){}

                virtual void produce()
                {
                        {
                                std::unique_lock<std::mutex> lk(mtx);
                                state = true;
                        }
                        cv.notify_one();
                }

                virtual void consume()
                {
                        {
                                std::unique_lock<std::mutex> lk(mtx);
                                state = false;
                        }
                        cv.notify_one();
                }

                virtual void wait_for_produce()
                {
                        {
                                std::unique_lock<std::mutex> lk(mtx);
                                cv.wait(lk, std::bind( &Synchronized_Link::__is_produce, this));
                        }
                }

                virtual void wait_for_consume()
                {
                        {
                                std::unique_lock<std::mutex> lk(mtx);
                                cv.wait(lk, std::bind( &Synchronized_Link::__is_consume, this));
                        }
                }
};


#endif  // __LINK_H__
