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

#ifndef __KERNEL_H__
#define __KERNEL_H__

#include <map>
#include "graph.h"
#include "xmlconverter.h"
#include "runner.h"

const std::string rtnode_name = "rtnode";

class Kernel 
{
	private :

		std::string script_name;

		std::string scriptfile;
		std::string resfile;
		std::string libdir;

		XmlConverter * xmlc;

		Graph graph;
		std::map<std::string, Graph::vertex_descriptor> node_map;
		std::map<int, Runner *> runners;
		
		static Kernel singleton;

	public :
	
		Kernel() : xmlc(NULL) {}
		~Kernel()
		{
			delete xmlc;
			graph.clear();
			runners.clear();
			node_map.clear();
		}
		Kernel(const Kernel&) = delete;
                Kernel& operator=(const Kernel&) = delete;

		static void init(std::string scriptfile, std::string resfile, std::string libdir)
		{
			singleton.scriptfile=scriptfile;
			singleton.resfile=resfile;
			singleton.libdir = libdir;
			
			singleton.xmlc = new  XmlConverter(scriptfile);
		
			singleton.script_name = singleton.xmlc->getScriptName(); 

			std::cout << "Run : " << singleton.script_name << " script"<< std::endl;
		}		

		static Kernel& get() noexcept { 
     			 return singleton;
   		}

		void load_lib();			
		void load_input();			
		void load_graph();

		void add_rttoken();

		void add_function(Function *box);
		void del_function(Function * funct);
		void del_function(const std::string & uuid);

		void simple_runner_allocation();
		void runner_allocation();

		void spawn();
		void join();

};

#endif // __KERNEL_H__
