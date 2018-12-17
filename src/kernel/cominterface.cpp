/*
Copyright Instar Robotics

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
