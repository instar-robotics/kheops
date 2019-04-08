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


#include "kheops/kernel/cominterface.h"
#include "kheops/kernel/kernel.h"

ComInterface *ComInterface::singleton = NULL;

void ComInterface::exec_request()
{
	bool order;
	while( !qrequest.empty() )
	{
		Request r =  qrequest.front();
		qrequest.pop();

		switch(r.id_cmd)
		{	
			case C_CONTROL:

				switch(r.id_arg)
				{
					case S_RESUME :
						Kernel::resume();
						break;
					case S_QUIT :
						Kernel::ask_quit();
						break;
					case S_PAUSE :
						Kernel::pause();
						break;
				}
				break;

			case C_WEIGHT:
				switch(r.id_arg)
				{
					case S_LOAD :
						Kernel::sweight_load(r.args[0]);
						break;
					case S_SAVE :
						Kernel::sweight_save(r.args[0]);
						break;
				}
				break;

			case C_OSCILLO:

				if( r.id_arg == S_START) order = true;
				else order = false;
				
				Kernel::active_oscillo(order);
				break;

			case C_RTTOKEN:
				if( r.id_arg == S_START) order = true;
				else order = false;	

				Kernel::active_rt_token(order);

				break;
			case C_OUTPUT:
				if( r.id_arg == S_START) order = true;
				else order = false;	
		
				Kernel::active_output(r.args[0], order);

				break;
			case C_ACTIVITY:
				if( r.id_arg == S_START) order = true;
				else order = false;	
		
				Kernel::active_save_activity(r.args[0], order);

				break;
		}	
	}
}
