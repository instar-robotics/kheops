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
#include <regex>

void RosInterface::build()
{
        if (NULL == singleton)
        {
                singleton =  new RosInterface();
        }
}

void RosInterface::destroy()
{
    if (NULL != singleton)
    {
        delete singleton;
        singleton = NULL;
    }
}

void RosInterface::_init(int argc, char ** argv, std::string prog_name, std::string script_name)
{
	name = prog_name+"_"+script_name;
        ros::init(argc, argv, name);

	ros::Time::init();
        while (!ros::master::check())
        {
                std::cout << "waiting..." << std::endl;
                ros::Duration(0.5).sleep();
        }
        std::cout << "master started!" << std::endl;
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
	ros::NodeHandle n;

	sHelper = n.advertiseService( name+"/"+CMD[C_HELP] , &RosInterface::callback_helper, this);
	sControl = n.advertiseService( name+"/"+CMD[C_CONTROL] ,&RosInterface::callback_control, this);
	sWeight = n.advertiseService( name+"/"+CMD[C_WEIGHT] , &RosInterface::callback_weight, this);
	sRtStat = n.advertiseService( name+"/"+CMD[C_RTSTAT] , &RosInterface::callback_rt_stat, this);
	sOutput = n.advertiseService( name+"/"+CMD[C_OUTPUT] , &RosInterface::callback_output, this);
	sOscillo = n.advertiseService( name+"/"+CMD[C_OSCILLO] ,&RosInterface::callback_oscillo, this);
	sObjects = n.advertiseService( name+"/"+CMD[C_OBJECTS], &RosInterface::callback_objects, this);
	sRtToken = n.advertiseService( name+"/"+CMD[C_RTTOKEN] , &RosInterface::callback_rt_token, this);
	sActivity = n.advertiseService( name+"/"+CMD[C_ACTIVITY] ,&RosInterface::callback_activity,this);
}

bool RosInterface::callback_activity(hieroglyph::ArgCmd::Request& request,hieroglyph::ArgCmd::Response& response)
{
        Request r;
        r.id_cmd = C_ACTIVITY;

        if( request.cmd ==  CARG[S_START] )
        {
                if(  Kernel::instance().find_object( request.arg ))
                {
                        r.id_arg = S_START;
                        r.args.push_back( request.arg );
                        response.ret =  CARG[S_START]+" : "+request.arg;
                        qrequest.push(r);
                }
                else response.ret = RETURN[1];
        }
        else if( request.cmd == CARG[S_STOP])
        {
                if(  Kernel::instance().find_object( request.arg ))
                {
                        r.id_arg = S_STOP;
                        r.args.push_back( request.arg );
                        response.ret =  CARG[S_STOP]+" : "+request.arg;
                        qrequest.push(r);
                }
                else response.ret = RETURN[1];
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
	Request r;
	r.id_cmd = C_RTTOKEN;

	if( request.cmd == CARG[S_START] )
	{
		r.id_arg = S_START;
		qrequest.push(r);
		response.ret = CARG[S_START];
	}
	else if( request.cmd == CARG[S_STOP] )
	{
		r.id_arg = S_STOP;
		qrequest.push(r);
		response.ret = CARG[S_STOP];
	}
	else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_oscillo( hieroglyph::SimpleCmd::Request& request, hieroglyph::SimpleCmd::Response& response)
{
	Request r;
	r.id_cmd = C_OSCILLO;

	if( request.cmd == CARG[S_START] )
	{
		response.ret = CARG[S_START] ;
		r.id_arg = S_START;
		qrequest.push(r);
	}
	else if( request.cmd == CARG[S_STOP])
	{
		response.ret = CARG[S_STOP];
		r.id_arg = S_STOP;
		qrequest.push(r);
	}
	else  response.ret = RETURN[0];

	return true;
}

bool RosInterface::callback_output( hieroglyph::ArgCmd::Request& request, hieroglyph::ArgCmd::Response& response)
{
	Request r;
	r.id_cmd = C_OUTPUT;


	if( request.cmd ==  CARG[S_START] )
	{
		if(  Kernel::instance().find_object( request.arg )) 
		{
			r.id_arg = S_START;	
			r.args.push_back( request.arg );
			response.ret =  CARG[S_START]+" : "+request.arg;
			qrequest.push(r);
		}
		else response.ret = RETURN[1];
	}
	else if( request.cmd == CARG[S_STOP])
	{
		if(  Kernel::instance().find_object( request.arg )) 
		{
			r.id_arg = S_STOP;	
			r.args.push_back( request.arg );
			response.ret =  CARG[S_STOP]+" : "+request.arg;
			qrequest.push(r);
		}
		else response.ret = RETURN[1];
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
	r.id_cmd = C_WEIGHT;

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

	r.id_cmd = C_CONTROL;
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

bool RosInterface::callback_helper( hieroglyph::Help::Request&, hieroglyph::Help::Response& response)
{
	response.help.push_back("1- help : list all the request supported by kheops\n");
	response.help.push_back("2- control : general command\n");
	response.help.push_back("   a- resume\n");
	response.help.push_back("   b- pause\n");
	response.help.push_back("   c- quit\n");
	response.help.push_back("   d- status\n");
	response.help.push_back("   e- path\n");
	response.help.push_back("3- weight : command relative to the weight file\n");
	response.help.push_back("   a- save 'path'  (path is not mandatory, take the default weigth path)\n");
	response.help.push_back("   b- load 'path'  (path is not mandatory, take the default weigth path)\n");
	response.help.push_back("4- rt_stat : get rt_token status\n");
	response.help.push_back("5- oscillo :\n");
	response.help.push_back("   a- start  (active oscillo)\n");
	response.help.push_back("   b- stop   (stop oscillo)\n");
	response.help.push_back("6- output :\n");
	response.help.push_back("   a- start 'uuid'  (active output for uuid object )\n");
	response.help.push_back("   b- stop  'uuid'  (stop output for uuid object )\n");
	response.help.push_back("7- objects :\n");
	response.help.push_back("   a- all : get the list of uuid/type of all the objects\n");
	response.help.push_back("   b- ilink : get list of uuid of ilink\n");
	response.help.push_back("   c- input : get list of uuid of input\n");
	response.help.push_back("   d- function  get list of uuid of function\n");
	response.help.push_back("   c- rt_token :  get list of uuid of rt_token\n");
	response.help.push_back("8- rt_token :\n");
	response.help.push_back("   a- start (active rt_token topic)\n");
	response.help.push_back("   b- stop  (stop rt_token topic)\n");
	response.help.push_back("9- save_activity :\n");
	response.help.push_back("   a- start 'uuid' (start saving function's activity into SHM)\n");
	response.help.push_back("   b- stop 'uuid' (stop saving function's activity into SHM)\n");

	return true;
}

void RosInterface::_setDefaultName(std::string& str)
{
	str = name + "/" + str;
	RosUtil::clean_topic_name(str);
}
