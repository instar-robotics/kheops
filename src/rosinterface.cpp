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

#include "rosinterface.h"

void RosInterface::init(int argc, char ** argv, std::string prog_name, std::string script_name)
{
	RosWrapper::init(argc, argv, prog_name, script_name);
}

void RosInterface::registerListener()
{
	sHelper = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/help" , &RosInterface::callback_helper, this);
	sControl = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/control" , &RosInterface::callback_control, this);
	sWeight = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/weight" , &RosInterface::callback_weight, this);
	sRtStat = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/rt_stat" , &RosInterface::callback_rt_stat, this);
	sOutput = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/output" , &RosInterface::callback_output, this);
	sOscillo = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/oscillo" , &RosInterface::callback_oscillo, this);
}

bool RosInterface::callback_oscillo( hieroglyph::Oscillo::Request& request, hieroglyph::Oscillo::Response& response)
{
	RtToken& rt = Kernel::instance().getRtToken();

	if( request.cmd == OSCILLO[0] )
	{
		// Start TOPIC 
		rt.start_oscillo_publisher();
		Runner::active_oscillo(true);
		response.state = OSCILLO[0];
	}
	else if( request.cmd == OSCILLO[1])
	{
		// Stop TOPIC 
		Runner::active_oscillo(false);
		rt.stop_oscillo_publisher();
		response.state = OSCILLO[1];
	}

	return true;
}

bool RosInterface::callback_output( hieroglyph::Output::Request& request, hieroglyph::Output::Response& response)
{

	if( request.cmd == OUTPUT[0] )
	{
		if( Kernel::instance().active_output( request.target, true ) )
		{
			response.state = OUTPUT[0]+" : "+request.target;
		}
		else
		{
			response.state = RETURN[1];
		}
	}
	else if( request.cmd == OUTPUT[1])
	{
		if( Kernel::instance().active_output( request.target, false ) )
		{
			response.state = OUTPUT[1]+" : "+request.target;
		}
		else
		{
			response.state = RETURN[1];
		}
	}
	else  response.state = RETURN[0];

	return true;
}

bool RosInterface::callback_rt_stat( hieroglyph::RtStat::Request& request, hieroglyph::RtStat::Response& response)
{
	RtToken& rt = Kernel::instance().getRtToken();		

	response.data.uuid = rt.getUuid();
	response.data.period = rt.getPeriod();
	response.data.means =  rt.getMeanDuration();
	response.data.sleep = rt.getLastSleep(); 
	response.data.start = rt.getLastStart();
	response.data.duration = rt.getLastDuration();
	
	return true;
}

bool RosInterface::callback_weight( hieroglyph::Weight::Request& request, hieroglyph::Weight::Response& response)
{
	int state = Kernel::instance().getState();
	if(state == RUN) Kernel::instance().pause();

	if( request.cmd == WEIGHT[0] )
	{
		if( request.path.size() == 0 )
		{
			Kernel::instance().save_weight();
		}
		else{
			  Kernel::instance().save_weight(request.path);
		}
		response.state = WEIGHT[0];
	}
	else if( request.cmd == WEIGHT[1] )
	{
		if( request.path.size() == 0 )
		{
			Kernel::instance().load_weight();
		}
		else{
			 Kernel::instance().load_weight(request.path);
		}
		response.state = WEIGHT[1];
	}
	else  response.state = RETURN[0];

	if( state == RUN) Kernel::instance().resume();

	return true;
}

bool RosInterface::callback_control( hieroglyph::Control::Request& request, hieroglyph::Control::Response& response)
{

	if( request.cmd == CMD[0])
	{
		response.state = CMD[0];
		Kernel::instance().resume();
	}
	else if( request.cmd == CMD[1])
	{
		response.state = CMD[1];
		quit();
	}
	else if( request.cmd == CMD[2] )
	{
		response.state = CMD[2];
		Kernel::instance().pause();
	}
	else
	{
		response.state = RETURN[0];
	}
	return true;
}

bool RosInterface::callback_helper( hieroglyph::Help::Request& request, hieroglyph::Help::Response& response)
{
	std::cout << "HELP " << std::endl;
	return true;
}

