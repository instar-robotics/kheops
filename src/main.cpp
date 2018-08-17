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

#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <syslog.h>
#include <signal.h>
#include <exception>

#include "kheops/kernel/rttoken.h"
#include "kheops/kernel/frunner.h"
#include "kheops/kernel/kernel.h"
#include "kheops/kernel/inputbase.h"
#include "kheops/kernel/libManager.h"
#include "kheops/kernel/publisher.h"
#include "kheops/ros/rosinterface.h"
#include "kheops/util/util.h"

ComInterface * cinter = NULL;

void signals_handler(int number)
{
  switch (number)
  {
    case SIGINT:
      	syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGINT received : interruption " );
      	if( cinter != NULL) delete(cinter); 
	Kernel::terminate();
      	break;
    case SIGTERM :
        syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGTERM received : terminaison" );
	if(cinter != NULL) delete(cinter);
        Kernel::terminate();
	break;
    default : 
	std::string mess = "KHEOPS received unknown SIGNAL" + number;
  	syslog( LOG_LOCAL0|LOG_LOCAL0, "%s", mess.c_str());
	break;
  }

  syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS signal handler end " );
  exit(0);
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
	char opt;
	
	std::string script;
	std::string libdir;
	std::string weight;

	print_splash();
	
	std::string progname;
	get_file_name( argv[0], progname);

	while ((opt = getopt (argc, argv, "vphs:w:l:i")) != -1)
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
			default : return 0;
		}	
	}
	
	if( !run )
	{
		std::cout << "Fatal : need to load a XML Script file \n" << std::endl;
		print_help();

		return 0;
	} 	

        /*************Signaux handler operation ****************/
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

        /************* End Signaux handler operation ****************/

	cinter = new RosInterface();

	LibManager::init(libdir);
	LibManager::load();

	Kernel::init(script,weight, ignore_matrix_check);	
	
	try{

		cinter->init( argc, argv, progname , Kernel::instance().getName() );	

		Kernel::load();
		Kernel::prerun();
		Kernel::start(resume);

		cinter->registerListener();
		cinter->enter();
	}
	catch (std::exception& e)
	{
		std::cerr << "FATAL. " <<  e.what() << std::endl;		
		Kernel::quit();
	}

	Kernel::wait();
	delete(cinter);

	return 0;
}
