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


#ifndef __FR_RUNNER_H__
#define __FR_RUNNER_H__

#include "kheops/kernel/runner.h"
#include "kheops/kernel/function.h"

class FRunner : public Runner
{
        protected :

		bool bsync;
                std::mutex mtx_sync;
                std::condition_variable cv_sync;

        public :
                FRunner() : Runner(),bsync(false) {}
                virtual ~FRunner() {}

                virtual void exec();
		virtual void change_state(int state) {this->state = state;}

		void wait_for_sync();
                void sync();

		void checkFunction();
};

#endif //__FR_RUNNER_H__
