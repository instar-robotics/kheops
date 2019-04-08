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


#ifndef __KERNEL_H__
#define __KERNEL_H__

#include <map>
#include <memory>
#include "kheops/kernel/publisher.h"
#include "kheops/kernel/kstate.h"
#include "kheops/kernel/graph.h"
#include "kheops/kernel/runner.h"
#include "kheops/kernel/inputbase.h"
#include "kheops/kernel/rttoken.h"
#include "kheops/iostream/xmlconverter.h"
#include "kheops/links/iscalar.h"
#include "kheops/links/imatrix.h"

// in millisecondes
const int wait_delay = 100;

class Kernel 
{
	private :
		static Kernel singleton;

		std::string script_file;
		std::string weight_file;

		// Store graph of Function/Runner and klink
		Graph graph;

		RtToken rttoken;

		// string : uuid function
		std::map<std::string, Graph::vertex_descriptor> node_map;
		// string : uuid link
		std::map<std::string, Graph::edge_descriptor> edge_map;

		// string : uuid link
		// This maps should be useful to debug weight with ROS Topic
		std::map<std::string, std::shared_ptr<iLinkBase>> ilinks;

		// string 1 : input uuid
		std::map<std::string, InputBase*> inputs;

		// string 1 : uuid ilink, string 2 : uuid input
		std::map<std::string,std::string> ilink_to_input;
		// string 1 : uuid input, string 2 : uuid function
		std::map<std::string,std::string> input_to_funct;
		
		XScript xs;

		bool ignore_matrix_check;
		bool squit;

		StatusPublisher *k_pub;
	
		Kernel();
	public :

		~Kernel();
		Kernel(const Kernel&) = delete;
                Kernel& operator=(const Kernel&) = delete;

		inline const Graph& getGraph(){return graph;}
		inline const std::string& getName(){return xs.name;}

		void load_links();			
		void load_functions();
		void load_rttoken();
		void load_weight();
		void save_weight();
		void load_weight(const std::string& filename);
		void save_weight(const std::string& filename);
		
		// in_uuid = Input Uuid
		void add_ilink(const std::string& in_uuid,const XLink&);
		void add_iscalar(const std::string& in_uuid,const XLink&);
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
		void prerun_functions();
		void onQuit_functions();
		void onPause_functions();
		void onRun_functions();
		
		void init_rt_token();
		void update_rt_token_value( const XRtToken& xrt );
		void create_rt_klink();
		void clear_rt_klink();

		void add_runner(const std::string& uuid);
		void remove_runner(const std::string& uuid);
		void spawn_runners();
		void quit_runners();
		void wait_for_resume_runners();
		void wait_for_pause_runners();

		void bind( InputBase& value,const std::string& var_name,const std::string& uuid );
		void bind( IString& value,const std::string& var_name,const std::string& uuid );

		// String 1 : input uuid
		void unbind_input(const std::string& uuid);

		// String 1 : input uuid
		void purge_empty_links(const std::string& in_uuid);

		// CMD Section 
		inline int getState(){ return rttoken.getState();}
		const std::string& getStateName(){ return K_STATE[rttoken.getState()];}
		RtToken& getRtToken() {return rttoken;}
		const std::string& getPath(){return script_file;}

		
		// Save activity  		
		bool save_activity(const std::string& uuid, bool state);

		//Objects
		void get_objects(std::vector<std::string> & objects);
		void get_inputs(std::vector<std::string> & objects);
		void get_ilinks(std::vector<std::string> & objects);
		void get_functions(std::vector<std::string> & objects);
		void get_rt_token(std::vector<std::string> & objects);
		bool find_object(const std::string& uuid);

		//Publish
		// Active publishing of the object defined by UUID
		bool active_publish(const std::string& uuid, bool state);
		void publish_status(StatusMessage &message);
		void active_status(bool state);

		// Static member : 
		static inline Kernel& instance() noexcept {return singleton;}
		static void init(std::string scriptfile, std::string resfile, bool ignore_matrix_check = false);
                
		static bool is_run() { return singleton.getRtToken().is_run();}
                static bool is_stop() { return singleton.getRtToken().is_stop();}
                static bool is_pause() { return singleton.getRtToken().is_pause();}

		static void build();
		static void destroy();
		static void load(); 	
		static void prerun(); 	
		static void start(bool run);
		static void pause(); 
		static void resume(); 
		static void quit();
		static void ask_quit() {singleton.squit = true;}
		static bool is_asking_quit() {return singleton.squit;}
		static void sweight_save(std::string& path);
		static void sweight_load(std::string& path);
		static void active_oscillo(bool order);
		static void active_output(const std::string& uuid, bool order);
		static void active_save_activity(const std::string& uuid, bool order);
		static void active_rt_token(bool order);
		
		static void iBind(InputBase& value,const std::string& var_name,const std::string& uuid );
		static void iBind(IString& value,const std::string& var_name,const std::string& uuid );
};

#endif // __KERNEL_H__
