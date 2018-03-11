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
#include "input.h"

class Kernel 
{
	private :

		std::string scriptfile;
		std::string resfile;
		std::string libdir;

		XmlConverter * xmlc;

		Graph graph;
		std::map<std::string, Graph::vertex_descriptor> node_map;
		
		static Kernel singleton;

		XScript xs;

	public :
	
		Kernel() : xmlc(NULL) {}
		~Kernel();
		Kernel(const Kernel&) = delete;
                Kernel& operator=(const Kernel&) = delete;

		static void init(std::string scriptfile, std::string resfile, std::string libdir);
		static inline Kernel& instance() noexcept {return singleton;}

		void load_lib();			
		void load_links();			
		void load_functions();

		void add_rttoken();

		Function* buildFunction(const XFunction&);
		void add_function(Function *funct);
		void del_function(Function *funct);
		void del_function(const std::string & uuid);
		
		// Beta function : do not use this for now
		// Warning 1 : Doesn't remap Input for now !!
		// Warning 2 : XML Model is not updated
		void add_function_suc(std::string Fct, std::string pred_uuid, int x=-1, int y=-1);
		// Beta function : do not use this for now
		// Warning 1 : Doesn't remap Input for now !!
		// Warning 2 : XML Model is not updated
		void add_function_pred(std::string Fct, std::string suc_uuid, int x=-1, int y=-1);

		// Beta function : do not use this for now
		//Warning 1 : Doesn't delete the old ling between pred_uuid and suc_uuid
		// Need to call del_link if a link exist before pred and suc
		//Warning 2 : Doesn't remap Input for now !! 
		//Warning 3 : XML Model is not updated
		void insert_function(std::string Fct, std::string pred_uuid, std::string suc_uuid ,int x=-1, int y=-1);
		// Beta function : do not use this for now
		// Warning 1 : Doesn't remap Input for now !!
		// Warning 2 : XML Model is not updated
		void del_link(std::string pred_uuid, std::string suc_uuid);

		void simple_runner_allocation();
		void runner_allocation();

		void write_graph();

		void bind( IScalar& value, std::string var_name, std::string uuid );
		void bind( IScalarMatrix& value, std::string var_name, std::string uuid );
		void bind( ISAnchor& value, std::string var_name, std::string uuid );
		void bind( ISMAnchor& value, std::string var_name, std::string uuid );
		void bind( IMMAnchor& value, std::string var_name, std::string uuid );
		void bind( std::string& value, std::string var_name, std::string uuid );

};

#endif // __KERNEL_H__
