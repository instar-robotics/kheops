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

#include <dlfcn.h>
#include <sstream>

#include "kernel.h"
#include "factory.h"
#include "util.h"
#include "frunner.h"
#include "rttoken.h"

Kernel Kernel::singleton;

void Kernel::load_graph()
{
	Function *f = xmlc->getFirstFunction();
	do
	{
		if( f == NULL) throw  std::invalid_argument("Kernel : Unable to find Function");
		else add_function(f);

	}while( (f = xmlc->getNextFunction()) != NULL );
}

void Kernel::load_input()
{
	std::vector<std::string> inputs; 
	for( auto it = boost::vertices(graph); it.first != it.second; ++it.first)
	{
		std::cout << (graph[*it.first])->getUuid() << std::endl;
		xmlc->getInputsUuid(  (graph[*it.first])->getUuid() , inputs );

		// Pour chaque inputs :
		// getNode 
		//

		inputs.clear();
	}	
}

void Kernel::load_lib()
{
  void *handle;	
  std::vector<std::string> files;
  getdir (libdir, files);

  for( auto it  = files.begin(); it != files.end(); it++)
  {
        std::cout << "Load lib : "  << (*it) << std::endl;
        handle = dlopen (  (libdir+(*it)).c_str() , RTLD_LAZY);
        if (!handle) {
            fputs (dlerror(), stderr);
            exit(1);
        }
  }
}

void Kernel::add_function( Function *funct  )
{
	if( funct != NULL) Graph::vertex_descriptor v1 = boost::add_vertex(funct, graph);
	else throw  std::invalid_argument("Kernel : try to add Null Function");
}


