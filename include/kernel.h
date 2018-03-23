/*
Copyright INSTAR Robotics

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
#include <memory>
#include "graph.h"
#include "xmlconverter.h"
#include "runner.h"
#include "input.h"

class Kernel 
{
//	private :
	public:

		std::string scriptfile;
		std::string resfile;

		Graph graph;
		// string : uuid function
		std::map<std::string, Graph::vertex_descriptor> node_map;
		// string : uuid link
		std::map<std::string, Graph::edge_descriptor> edge_map;

		// string : uuid link
		// This maps should be useful to debug weight with ROS Topic
		std::map<std::string, std::shared_ptr<IScalar>> is_ilink;
		std::map<std::string, std::shared_ptr<IMatrix>> im_ilink;

		// string 1 : input uuid
		std::map<std::string, ISInput*> is_input;
		std::map<std::string, IMInput*> im_input;
		std::map<std::string, ISMInput*> ism_input;
		std::map<std::string, IMMInput*> imm_input;

		// string 1 : uuid ilink, string 2 : uuid input
		std::map<std::string,std::string> ilink_to_input;
		// string 1 : uuid input, string 2 : uuid function
		std::map<std::string,std::string> input_to_funct;
		
		XScript xs;

		static Kernel singleton;

	public :
	
		Kernel(){}
		~Kernel();
		Kernel(const Kernel&) = delete;
                Kernel& operator=(const Kernel&) = delete;

		static void init(std::string scriptfile, std::string resfile);
		static inline Kernel& instance() noexcept {return singleton;}

		inline const Graph& getGraph(){return graph;}
		inline const std::string& getName(){return xs.name;}

		void load_links();			
		void load_functions();
		
		void add_ilink(const std::string& in_uuid,const XLink&);
		void add_iscalar(const std::string& in_uuid,const XLink&);
		void add_imatrix(const std::string& in_uuid,const XLink&);
		void add_ismatrix(const std::string& in_uuid,const XLink&);
		void add_immatrix(const std::string& in_uuid,const XLink&);
		void del_ilink(const std::string& link_uuid);
		// string 1 : function uuid
		void purge_output_ilinks(const std::string& uuid);

		// string 1 : function uuid
		void add_klink(const std::string& in_uuid,const XLink&);
		// string 1 : link uuid
		void del_klink(const std::string& link_uuid);
		// string 1 : function uuid
		void purge_klinks(const std::string& uuid);

		void add_function(const XFunction&);
		void del_function(const std::string & uuid);
		
		void add_rttoken();
		void update_rttoken_value( const XRtToken& xrt );
		void create_rt_klink();
		void clear_rt_klink();

		void runner_allocation();
		void add_runner(const std::string& uuid);
		void remove_runner(const std::string& uuid);

		void separate_runner_allocation();
		void add_separate_runner(const std::string& uuid);
		void remove_separate_runner(const std::string& uuid);

		void bind( ISInput& value,const std::string& var_name,const std::string& uuid );
		void bind( IMInput& value,const std::string& var_name,const std::string& uuid );
		void bind( ISMInput& value,const std::string& var_name,const std::string& uuid );
		void bind( IMMInput& value,const std::string& var_name,const std::string& uuid );
		void bind( std::string& value,const std::string& var_name,const std::string& uuid );

		// String 1 : input uuid
		void unbind_isinput(const std::string& uuid);
		void unbind_iminput(const std::string& uuid);
		void unbind_isminput(const std::string& uuid);
		void unbind_imminput(const std::string& uuid);

		// String 1 : input uuid
		void purge_empty_links(const std::string& in_uuid);
};

#endif // __KERNEL_H__
