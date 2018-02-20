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

#include "kernel.h"
#include "xmlconverter.h"


//TEST
/*
#include "myfct.h"
#include "frunner.h"
#include "rttoken.h"
*/

void gestionnaire_signaux(int numero)
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
	action.sa_handler = gestionnaire_signaux;
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
        /*************End Signaux handler operation ****************/

	XmlConverter::Initialize();

	Kernel::init(fscript,fres,libdir);	
	std::cout << "HERE" << std::endl;
	Kernel::get().load_lib();
	std::cout << "HERE" << std::endl;
	Kernel::get().load_functions();
	std::cout << "HERE" << std::endl;
	Kernel::get().load_inputs();
	std::cout << "HERE" << std::endl;
	Kernel::get().add_rttoken();
	std::cout << "HERE" << std::endl;
	Kernel::get().runner_allocation();
	std::cout << "HERE" << std::endl;
	
	Kernel::get().spawn();
	
	bool run  = true;
        while(run)
        {

                std::string buffer;
                getline(std::cin,buffer);

                if(buffer == "pause") {
                        Runner::pause();
                }
                else if (buffer == "run" ) Runner::resume();
                else if (buffer == "q")
                {
                        run = false;
                }

                std::cout << "M state : "  << Runner::getState() << std::endl;
        }

	Runner::stop();	
	Kernel::get().join();

	return 0;


	/*****************/
	// TEST
	/*	
  	Graph g;

	MyFct mf;

	Graph::vertex_descriptor v1 = boost::add_vertex(&mf, g);
 	Graph::vertex_descriptor v2 = boost::add_vertex(&mf, g);
	Graph::vertex_descriptor rt_node = boost::add_vertex(g);

	Synchronized_Link l12;
  	Synchronized_Link ld1;	
  	Synchronized_Link l2e;	

	EdgeWeightProperty e12 =dynamic_cast<Link*>(&l12);
  	EdgeWeightProperty ed1 =dynamic_cast<Link*>(&ld1);
  	EdgeWeightProperty e2e =dynamic_cast<Link*>(&l2e);

	add_edge(v1, v2, e12, g);
  	add_edge(rt_node , v1, ed1 , g);
  	add_edge(v2, rt_node, e2e , g);

	FRunner r1(1);
	FRunner r2(2);

	r1.setGraph(&g);
	r2.setGraph(&g);

	r1.add_node(&v1);
	r2.add_node(&v2);

	RtToken RT(20000,"Hz");
	RT.setGraph(&g);
	RT.setRtNode(&rt_node);

	r1.spawn();
  	r2.spawn();
  	RT.spawn();

 	Runner::resume();

	bool run  = true;
        while(run)
        {

                std::string buffer;
                getline(std::cin,buffer);

                if(buffer == "pause") {
                        Runner::pause();
                }
                else if (buffer == "run" ) Runner::resume();
                else if (buffer == "q")
                {
                        run = false;
                }

                std::cout << "M state : "  << Runner::getState() << std::endl;
        }

	Runner::stop();	

	r1.getThread().join();
        r2.getThread().join();
        RT.getThread().join();

	return 0;
	*/
}
