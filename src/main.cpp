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


#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <syslog.h>
#include <signal.h>
#include <exception>
#include <cstdlib>


#include "kheops/kernel/rttoken.h"
#include "kheops/kernel/frunner.h"
#include "kheops/kernel/kernel.h"
#include "kheops/kernel/inputbase.h"
#include "kheops/kernel/libManager.h"
#include "kheops/kernel/publisher.h"
#include "kheops/ros/rosinterface.h"
#include "kheops/util/util.h"

const std::string KHEOPS_LIB_PATH = "KHEOPS_LIB_PATH";

void signals_handler(int number)
{
  switch (number)
  {
    case SIGINT:
      	syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGINT received : interruption " );
      	break;
    case SIGTERM :
        syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGTERM received : terminaison" );
	break;
    default : 
	std::string mess = "KHEOPS received unknown SIGNAL" + number;
  	syslog( LOG_LOCAL0|LOG_LOCAL0, "%s", mess.c_str());
	break;
  }

  Kernel::ask_quit();
  Kernel::quit();
  RosInterface::destroy();

  syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS signal handler end " );
}

void print_splash(void)
{
	std::cout << "######################################################" << std::endl;
	std::cout << "#                                                    #" << std::endl;
	std::cout << "#       Kheops : the neural networks simulator       #" << std::endl;
	std::cout << "#                                                    #" << std::endl;
	std::cout << "#  Powered by Instar Robotics                        #" << std::endl;
	std::cout << "######################################################" << std::endl;
}

void print_help(void)
{
	std::cout << "Use : kheops [OPTION] ...[FILE] ...." << std::endl;
	std::cout << "The Neural Network and Dynamical Function simulator" << std::endl;

	std::cout << "options : " << std::endl;
	std::cout << "  -h : display this menu" << std::endl;
	std::cout << "  -s [FILE] : path to the script file " << std::endl;	
	std::cout << "  -w [FILE] : path to the binary file containing neural weights" << std::endl;	
	std::cout << "  -l [DIR] : path to the directory containing the external libraries" << std::endl;	
	std::cout << "  -p : start running in pause mode " << std::endl;	
	std::cout << "  -v : active verbose mode " << std::endl;	
	std::cout << "  -i : ignore matrix check size integrity when load weigt file" << std::endl;	
}


int main(int argc, char **argv)
{
	struct sigaction action;
	bool verbose = false;
	bool resume = true;
	bool run = false;
	bool ignore_matrix_check = false;
	unsigned char opt;
	int ret = 0;
	
	std::string script;
	std::string libdir;
	std::string weight;

	print_splash();
	
	std::string progname;
	get_file_name( argv[0], progname);

	while (((opt = getopt (argc, argv, "vphs:w:l:i")) != -1) && (opt != 255))
	{
		switch( opt )	
		{
			case 'p' :  resume=false;break;
			case 'v' :  verbose=true;break;
			case 'h' :  print_help(); return 0;
			case 's' :
				run = true;  
				script = optarg ; 
				break;
			case 'w' :  weight = optarg ; break;
			case 'l' :  libdir = optarg ; break;
			case 'i' :  ignore_matrix_check = true ; break;
			default : 
				    std::cout << "Unkown option ! "  << std::endl;
				    print_help();
				    return 0;
		}	
	}
	
	
	if( !run )
	{
		std::cout << "Fatal : need to load a XML Script file \n" << std::endl;
		print_help();

		return 0;
	} 	

        // *************Signaux handler operation ****************
	
	sigfillset(&action.sa_mask);
	action.sa_handler = signals_handler;
  	action.sa_flags = 0;

  	if (sigaction(SIGINT, &action, NULL ) != 0)
	{
		std::cerr << "Unable to redefine SIGINT signal" << std::endl;
		syslog( LOG_LOCAL0|LOG_ERR , "Unable to redefine SIGINT signal" );
		return 0;
	}
  	if (sigaction(SIGTERM, &action, NULL ) != 0)
	{
		std::cerr << "Unable to redefine SIGTERM signal" << std::endl;
		syslog( LOG_LOCAL0|LOG_ERR , "Unable to redefine SIGTERM signal" );
		return 0;
	}

        // ************* End Signaux handler operation *************

	char* lib = secure_getenv(KHEOPS_LIB_PATH.c_str());

	if( lib != NULL )
	{
		if( libdir.size() > 0 ) libdir+=":";
		libdir.append(lib);
	}

	// *********************************************************
	// Use Ros builder by default
	RosInterface::build();

	LibManager::init(libdir);
	LibManager::load();

	Kernel::init(script, weight, ignore_matrix_check);	
	
	try{
		ComInterface::init( argc, argv, progname , Kernel::instance().getName() );	

		Kernel::load();
		Kernel::prerun();
		Kernel::start(resume);

		RosInterface::getInstance()->registerListener();
		ret = RosInterface::getInstance()->enter();
	}
	catch (std::exception& e)
	{
		std::cerr << "FATAL. " <<  e.what() << std::endl;		
	}

	Kernel::quit();
	RosInterface::destroy();

	return ret;
}
