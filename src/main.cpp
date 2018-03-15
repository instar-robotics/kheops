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


#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <syslog.h>
#include <signal.h>

#include "rttoken.h"
#include "frunner.h"
#include "kernel.h"
#include "xmlconverter.h"
#include "input.h"
#include "libManager.h"

#include <chrono>

void signals_handler(int numero)
{
  switch (numero)
    {
    case SIGINT:
      syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGINT received : terminaison " );
      break;
    case SIGTERM :
      syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS SIGTERM received : terminaison" );
	break;

    default : 
		
  	syslog( LOG_LOCAL0|LOG_LOCAL0 , "KHEOPS OTHER SIGNAL " );
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
	std::cout << "#  Powered by Enacted Robotics                       #" << std::endl;
	std::cout << "######################################################" << std::endl;

}

void print_help(void)
{
	std::cout << "Use : kheops [OPTION] ...[FILE] ...." << std::endl;
	std::cout << "Utility to multiplex access to microcontroller resources" << std::endl;

	std::cout << "options : " << std::endl;
	std::cout << "  -h : display this menu" << std::endl;
	std::cout << "  -v : verbose mode " << std::endl;	
	std::cout << "  -s : xml script file path " << std::endl;	
	std::cout << "  -r : res (neural weight) file path" << std::endl;	
	std::cout << "  -l : path to external lib" << std::endl;	
}


int main(int argc, char **argv)
{
	struct sigaction action;
	int verbose = 0;
	char opt;
	
	std::string fscript;
	std::string libdir;
	std::string fres;

	print_splash();

	while ((opt = getopt (argc, argv, "vhs:r:l:")) != -1)
	{
		switch( opt )	
		{
			case 'v' :  verbose=1;break;
			case 'h' :  print_help(); return 0;
			case 's' :  fscript = optarg ; break;
			case 'r' :  fres = optarg ; break;
			case 'l' :  libdir = optarg ; break;
			default : return 0;
		}	
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

	XmlConverter::Initialize();

	LibManager::init(libdir);
	LibManager::instance().load_libs();

	Kernel::init(fscript,fres);	
	Kernel::instance().load_functions();
	Kernel::instance().load_links();
	Kernel::instance().bind_functions();
	Kernel::instance().add_rttoken();
	Kernel::instance().runner_allocation();
	
        RtToken::instance().spawn();
	FRunner::spawn_all();
	
	bool run  = true;
        while(run)
        {
                std::string buffer;
                getline(std::cin,buffer);

                if(buffer == "pause") {
                        RtToken::instance().ask_pause();
                        RtToken::instance().wait_for_pause();
                }
                else if (buffer == "run" ) RtToken::instance().ask_resume();
                else if (buffer == "q")
                {
                        run = false;
                }
		else if (buffer == "pred")
		{
                	getline(std::cin,buffer);
                        RtToken::instance().ask_pause();
                        RtToken::instance().wait_for_pause();
			Kernel::instance().add_function_pred("MyFct", buffer  ,0,0);
                        RtToken::instance().ask_resume();
		}
		else if (buffer == "suc")
		{
                	getline(std::cin,buffer);
                        RtToken::instance().ask_pause();
                        RtToken::instance().wait_for_pause();
			Kernel::instance().add_function_suc("MyFct", buffer  ,0,0);
                        RtToken::instance().ask_resume();
		}
		else if (buffer == "ins")
		{
			std::string suc;
                	getline(std::cin,buffer);
                	getline(std::cin,suc);
                        RtToken::instance().ask_pause();
                        RtToken::instance().wait_for_pause();
			Kernel::instance().insert_function("MyFct", buffer ,suc ,0,0);
			Kernel::instance().del_klink(buffer ,suc);
                        RtToken::instance().ask_resume();
		}
		else if(buffer == "rt")
		{
			std::string unit;
                	getline(std::cin,buffer);
                	getline(std::cin,unit);
                        RtToken::instance().ask_pause();
                        RtToken::instance().wait_for_pause();
			RtToken::instance().setToken( std::stoi(buffer),  unit );
                        RtToken::instance().ask_resume();
		}
		else if(buffer == "w")
		{
			write_graph(Kernel::instance().getGraph(), Kernel::instance().getName());
		}
		else if(buffer == "quiet")
		{
			RtToken::instance().setQuiet(  !RtToken::instance().is_quiet());
		}

                std::cout << "M state : "  << RtToken::instance().getRequest() << std::endl;
        }
        RtToken::instance().ask_stop();
	RtToken::instance().join();
	FRunner::join_all();
	FRunner::clear();

	return 0;
}
