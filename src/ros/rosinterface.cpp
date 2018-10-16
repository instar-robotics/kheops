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

#include "kheops/ros/rosinterface.h"
#include "kheops/kernel/runner.h"

void RosInterface::init(int argc, char ** argv, std::string prog_name, std::string script_name)
{
	RosWrapper::init(argc, argv, prog_name, script_name);
}

void RosInterface::enter()
{
	ros::Rate r(DEFAULT_RATE);

	while( !Kernel::is_asking_quit() )
	{
	    exec_request();
	    ros::spinOnce();
	    r.sleep();
	}
}

void RosInterface::registerListener()
{
	sHelper = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_HELP] , &RosInterface::callback_helper, this);
	sControl = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_CONTROL] ,&RosInterface::callback_control, this);
	sWeight = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_WEIGHT] , &RosInterface::callback_weight, this);
	sRtStat = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_RTSTAT] , &RosInterface::callback_rt_stat, this);
	sOutput = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_OUTPUT] , &RosInterface::callback_output, this);
	sOscillo = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_OSCILLO] ,&RosInterface::callback_oscillo, this);
	sObjects = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_OBJECTS], &RosInterface::callback_objects, this);
	sRtToken = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_RTTOKEN] , &RosInterface::callback_rt_token, this);
	sActivity = RosWrapper::getNodeHandle()->advertiseService( RosWrapper::getNodeName()+"/"+CMD[C_ACTIVITY] , &RosInterface::callback_activity, this);
}

bool RosInterface::callback_activity(hieroglyph::ArgCmd::Request& request,hieroglyph::ArgCmd::Response& response)
{
	int state = Kernel::instance().getState();

        if( request.cmd ==  CARG[S_START] )
        {
                if( Kernel::instance().save_activity( request.arg, true ) )
                {
                        response.ret =  CARG[S_START]+" : "+request.arg;
                }
                else
                {
                        response.ret = RETURN[1];
                }
        }
        else if( request.cmd == CARG[S_STOP])
        {
                if(state == K_RUN) Kernel::instance().pause();

                if( Kernel::instance().save_activity( request.arg, false ) )
                {
                        response.ret = CARG[S_STOP]+" : "+request.arg;
                }
                else
                {
                        response.ret = RETURN[1];
                }

                if( state == K_RUN) Kernel::instance().resume();
        }
        else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_objects(hieroglyph::Objects::Request& request,hieroglyph::Objects::Response& response)
{
	if( request.object == CARG[S_ALL] ) 
	{
		Kernel::instance().get_objects(response.list);
		response.ret = CARG[S_ALL] ;
	}
	else if( request.object == CARG[S_RTTOKEN])
	{
		Kernel::instance().get_rt_token(response.list);
		response.ret = CARG[S_RTTOKEN] ;
	}
	else if( request.object == CARG[S_FUNCTIONS] )
	{
		Kernel::instance().get_functions(response.list);
		response.ret = CARG[S_FUNCTIONS] ;
	}
	else if( request.object == CARG[S_INPUTS])
	{
		Kernel::instance().get_inputs(response.list);
		response.ret = CARG[S_INPUTS] ;
	}
	else if( request.object == CARG[S_ILINKS])
	{
		Kernel::instance().get_ilinks(response.list);
		response.ret = CARG[S_ILINKS] ;
	}
	else response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_rt_token( hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response)
{
	RtToken& rt = Kernel::instance().getRtToken();
	int state = Kernel::instance().getState();
	
	if( request.cmd == CARG[S_START] )
	{
		rt.active_publish(true);
		response.ret = CARG[S_START];
	}
	else if( request.cmd == CARG[S_STOP] )
	{
		rt.active_publish(false);
		response.ret = CARG[S_STOP];
	}
	else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_oscillo( hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response)
{
	RtToken& rt = Kernel::instance().getRtToken();
	int state = Kernel::instance().getState();

	if( request.cmd == CARG[S_START] )
	{
		// Start TOPIC 
		rt.active_oscillo(true);
		response.ret = CARG[S_START] ;
	}
	else if( request.cmd == CARG[S_STOP])
	{
		if(state == K_RUN) Kernel::instance().pause();

		// Stop TOPIC 
		rt.active_oscillo(false);
		response.ret = CARG[S_STOP];
		
		if( state == K_RUN) Kernel::instance().resume();
	}
	else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_output( hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response)
{
	int state = Kernel::instance().getState();

	if( request.cmd ==  CARG[S_START] )
	{
		if( Kernel::instance().active_publish( request.arg, true ) )
		{
			response.ret =  CARG[S_START]+" : "+request.arg;
		}
		else
		{
			response.ret = RETURN[1];
		}
	}
	else if( request.cmd == CARG[S_STOP])
	{
		if(state == K_RUN) Kernel::instance().pause();

		if( Kernel::instance().active_publish( request.arg, false ) )
		{
			response.ret = CARG[S_STOP]+" : "+request.arg;
		}
		else
		{
			response.ret = RETURN[1];
		}

		if( state == K_RUN) Kernel::instance().resume();
	}
	else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_rt_stat( hieroglyph::RtStat::Request&, hieroglyph::RtStat::Response& response)
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

bool RosInterface::callback_weight( hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response)
{
	Request r;

	if( request.cmd == CARG[S_SAVE] )
	{
		r.id_arg = S_SAVE;
	}
	else if (  request.cmd == CARG[S_LOAD] )
	{
		r.id_arg = S_LOAD;
	}	
	else{
	       	response.ret = RETURN[0];
		return true;  // No, this is not an error, needed by ROS to not breking truth ... :)
	}

	r.args.push_back( request.arg );
	qrequest.push(r);
	response.ret = request.cmd ;

	return true;
}

bool RosInterface::callback_control(hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response)
{
	Request r;

	if( request.cmd == CARG[S_RESUME])
	{
		r.id_arg = S_RESUME;
		response.ret = CARG[S_RESUME];
		qrequest.push(r);
	}
	else if( request.cmd == CARG[S_QUIT] )
	{
		r.id_arg = S_QUIT;
		response.ret = CARG[S_QUIT];
		qrequest.push(r);
	}
	else if( request.cmd == CARG[S_PAUSE] )
	{
		r.id_arg = S_PAUSE;
		response.ret = CARG[S_PAUSE];
		qrequest.push(r);
	}
	else if( request.cmd == CARG[S_STATUS] )
	{
		response.ret = Kernel::instance().getStateName();
	}
	else if( request.cmd == CARG[S_PATH] )
	{
		response.ret = Kernel::instance().getPath();
	}
	else response.ret = RETURN[0];

	return true;
}

//bool RosInterface::callback_helper( hieroglyph::Help::Request& request, hieroglyph::Help::Response& response)
bool RosInterface::callback_helper( hieroglyph::Help::Request&, hieroglyph::Help::Response& response)
{
	response.help = " 1-cmd : list toto \n 2-titi : titi \t a- titu";
	
	std::cout <<  response.help  << std::endl;

	return true;
}

