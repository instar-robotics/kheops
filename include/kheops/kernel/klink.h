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


#ifndef __KLINK_H__
#define __KLINK_H__

#include <condition_variable>


class kLink{

	public : 
		
		kLink(){}
		virtual ~kLink(){}

		virtual void produce() = 0;
		virtual void consume() = 0;

};

class Passing_kLink : public kLink
{
        public:
                Passing_kLink(){}
                virtual ~Passing_kLink(){}

                virtual inline void produce() {}
                virtual inline void consume() {}
};

class Synchronized_kLink : public kLink
{
        private :
                std::mutex mtx;
                std::condition_variable cv;
                bool state;

                bool __is_produce() { return state;}
                bool __is_consume() { return state == false;}
        public :
                Synchronized_kLink() : kLink(),state(false)  {}
                Synchronized_kLink(bool state) : kLink(), state(state)  {}
                ~Synchronized_kLink(){}

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
                                cv.wait(lk, std::bind( &Synchronized_kLink::__is_produce, this));
				state = false;
                        }
                }

};


#endif  // __KLINK_H__
