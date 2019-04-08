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
#include <boost/filesystem.hpp>
#include <exception>
#include <stdexcept>

#include "kheops/kernel/kernel.h"
#include "kheops/kernel/factory.h"
#include "kheops/kernel/libManager.h"
#include "kheops/kernel/frunner.h"
#include "kheops/iostream/weightconverter.h"
#include "kheops/util/util.h"
#include "kheops/kernel/cominterface.h"
#include "kheops/ros/rospublisher.h"


Kernel Kernel::singleton;

Kernel::Kernel() : squit(false), k_pub(NULL) 
{	 
	k_pub = new RosStatusPublisher(5);
}

Kernel::~Kernel()
{	
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL ) delete(f);
		
		
		if(it->first != rttoken.getUuid())
		{
			Runner * runner = boost::get(boost::vertex_runner, graph)[it->second];
			delete(runner);
		}
	}
	
	for( auto it = edge_map.begin(); it != edge_map.end(); it++)
	{
		kLink *l = boost::get(boost::edge_klink , graph)[ it->second  ];
		if( l != NULL ) delete(l);
	}

	for( auto it = ilinks.begin(); it != ilinks.end(); it++) 
	{
		ilinks.erase(it);
	}
	
	graph.clear();
	node_map.clear();
	edge_map.clear();

	ilink_to_input.clear();
	input_to_funct.clear();
	
	inputs.clear();

	if( k_pub != NULL )
        {
                if( k_pub->is_open() ) k_pub->close();
                delete(k_pub);
        }

}

/*******************************************************************************************************/
/****************** 	 		Load Section	 			      ******************/
/*******************************************************************************************************/

void Kernel::load_functions()
{
	//build function using Factory
	for(auto it = xs.functions.begin(); it != xs.functions.end(); it++)
	{
		if(! Factory<Function>::Instance().is_register(it->second.name) )
		{
			std::cout << "Function "+it->second.name+" is not known. Try to load it" << std::endl;
			LibManager::load(it->second.libname);	
		}
		
		add_function( it->second );	
	}

	//setparameters : call bind function for each inputs
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL )
		{
		 	f->ksetparameters();
		}
	}
}


void Kernel::load_links()
{
	for(auto funct = xs.functions.begin(); funct != xs.functions.end(); funct++)
	{
		for( auto input = funct->second.inputs.begin(); input != funct->second.inputs.end(); input++)
		{
			//TODO : check here Input type
			for( auto link = input->second.links.begin(); link !=  input->second.links.end(); link++)
			{
				if( !xs.isLinkCst(*link) )
				{
					add_klink( funct->second.uuid , *link );
				}
				//TODO : check that input is not a string !
				// BUT this is creapy ! 
				// Add check type INPUT instead!
				if( link->value.size() == 0 ) add_ilink( input->second.uuid , *link );
			}
		}
	}	 	
}

void Kernel::load_rttoken()
{
	rttoken.set_rt_pub_name("rt_token");
	rttoken.set_oscillo_pub_name("oscillo");
	update_rt_token_value(xs.rt);
	create_rt_klink();
	clear_rt_klink();
}

void Kernel::load_weight()
{
	WeightConverter wc(weight_file);
	wc.load(inputs,ignore_matrix_check);
}

void Kernel::load_weight(const std::string& filename)
{
	std::string tmp_file = filename;
	if( !check_file_extension(filename, ".weight")) tmp_file+=".weight";

	WeightConverter wc(tmp_file);
	wc.load(inputs,ignore_matrix_check);
}

void Kernel::save_weight()
{
	WeightConverter wc(weight_file);
	wc.save(inputs);
}

void Kernel::save_weight(const std::string& filename)
{
	std::string tmp_file = filename;
	if( !check_file_extension(filename, ".weight")) tmp_file+=".weight";

	WeightConverter wc(tmp_file);
	wc.save(inputs);
}

/*******************************************************************************************************/
/****************** 			Function Section 			      ******************/
/*******************************************************************************************************/

void Kernel::add_function( const XFunction& xf)
{
      Function *f = Factory<Function>::Instance().create(xf.name);
      if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+xf.name);
      else
      { 
		f->setUuid(xf.uuid);
		f->set_pub_name(xf.topic_name);
		f->set_publish(xf.publish);
		f->set_save(xf.save);

		if( f->type() == typeid(MatrixXd).hash_code()) 		
		{
		  if( xf.rows <= 0 || xf.cols <= 0)  throw  std::invalid_argument("Kernel : Can't build Matrix Function without valid rows/cols value");
		  dynamic_cast<FMatrix*>(f)->setSize(xf.rows,xf.cols);
		}
		
		if( node_map.find( f->getUuid() ) == node_map.end() ) 
		{
			Graph::vertex_descriptor vd = boost::add_vertex(graph);
			boost::put(boost::vertex_function, graph, vd, f);
			node_map[f->getUuid()] = vd;

			// Add a runner to the function : 
			add_runner( f->getUuid() );
		}
		else throw  std::invalid_argument("Kernel : try to add a function with an existed uuid");
     }
}

void Kernel::del_function(const std::string& uuid)
{
	if( node_map.find( uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to del an unknown function "+ uuid );   

	Function *f =  boost::get(boost::vertex_function , graph)[ node_map[uuid]];

	f->onQuit();

	//purge all ilinks in all inputs
	for( auto input = f->get_input().begin() ; input !=  f->get_input().end(); input++)
	{
		unbind_input( (*input)->getUuid());
	}

	// purge output link
	purge_output_ilinks(uuid);
	
	// purge all klinks for the function
	purge_klinks(uuid);
	
	// remove the runner
	remove_runner(uuid);
	
	clear_vertex(node_map[uuid] , graph);
	remove_vertex( node_map[uuid] , graph);
	node_map.erase( node_map.find(uuid));

	delete(f);
}


void Kernel::prerun_functions()
{
        for(auto it = node_map.begin(); it != node_map.end(); ++it)
        {
		Function *f =  boost::get(boost::vertex_function , graph)[ node_map[it->first]];
		if( f != NULL ) f->kprerun();
        }
}

void Kernel::onQuit_functions()
{
        for(auto it = node_map.begin(); it != node_map.end(); ++it)
        {
		Function *f =  boost::get(boost::vertex_function , graph)[ node_map[it->first]];
		if( f != NULL ) f->onQuit();
	}
}

void Kernel::onPause_functions()
{
        for(auto it = node_map.begin(); it != node_map.end(); ++it)
        {
		Function *f =  boost::get(boost::vertex_function , graph)[ node_map[it->first]];
		if( f != NULL ) f->onPause();
	}
}

void Kernel::onRun_functions()
{
        for(auto it = node_map.begin(); it != node_map.end(); ++it)
        {
		Function *f =  boost::get(boost::vertex_function , graph)[ node_map[it->first]];
		if( f != NULL ) f->onRun();
	}
}

/*******************************************************************************************************/
/***************** 		        	RTTOKEN Section	              	      ******************/
/*******************************************************************************************************/

void Kernel::init_rt_token()
{
	//TODO : choose if RtToken UUID is generated or read from the XML ?
	std::string uuid = xs.rt.uuid;
	Graph::vertex_descriptor rt_node = boost::add_vertex(graph);
	boost::put(boost::vertex_runner, graph, rt_node, &rttoken);
	node_map[uuid] = rt_node;
	
	rttoken.setUuid(uuid);
	rttoken.setNode(rt_node);
	rttoken.setGraph(&graph);
}

void Kernel::update_rt_token_value(const XRtToken& xrt)
{
	rttoken.setToken(xrt.value , xrt.unit );
}

void Kernel::create_rt_klink()
{
	Graph::vertex_descriptor rt_node = rttoken.getNode();
	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
        {
		if( *it.first != rt_node)
		{
			auto it_out = boost::out_edges(*it.first, graph);
			if( it_out.first == it_out.second)
			{
				XLink xl; 
				xl.uuid = generate_uuid();
				xl.isSecondary = false;

				Function *f =  boost::get(boost::vertex_function, graph,  *it.first   ) ;
				if( f == NULL ) throw std::invalid_argument( "Kernel : try to add RT klink to a NULL Function " );
				xl.uuid_pred = f->getUuid();

				add_klink( rttoken.getUuid() , xl );
			}
		}
	}
}

void Kernel::clear_rt_klink()
{
	Graph::vertex_descriptor rt_node = rttoken.getNode();

	for( auto it = boost::in_edges(rt_node, graph); it.first != it.second; ++it.first)
	{
		// Get the vertex source 
		Graph::vertex_descriptor v = boost::source(*it.first, graph);
		
		// check the number of out edges. if more than one link : delete the klink
		auto it_out = boost::out_edges(v, graph);
		if( ++it_out.first != it_out.second ) 
		{
			del_klink( boost::get( boost::edge_uuid, graph, *it.first) );
		}
	}
}

/*******************************************************************************************************/
/******************* 		           kLink Section		   	   *********************/
/*******************************************************************************************************/

void Kernel::add_klink(const std::string &in_uuid, const XLink& xl)
{
	if(  node_map.find( xl.uuid_pred ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add link with unkown source "+xl.uuid_pred );
	if( node_map.find( in_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add link with unkown target "+in_uuid );

	if( edge_map.find( xl.uuid ) == edge_map.end() ) 
	{
		std::pair<Graph::edge_descriptor, bool> e = add_edge(node_map[ xl.uuid_pred],node_map[in_uuid],graph);

		if( xl.uuid_pred == in_uuid ) boost::put(boost::edge_klink, graph, e.first , new Passing_kLink());
		else boost::put(boost::edge_klink, graph, e.first , new Synchronized_kLink(xl.isSecondary));

		boost::put( boost::edge_uuid, graph, e.first , xl.uuid );
		edge_map[xl.uuid] = e.first;
	}
	else throw std::invalid_argument("Kernel : uuid link have to be unique. "+xl.uuid+" already register");
}

void Kernel::del_klink(const std::string& link_uuid)
{
	auto it = edge_map.find(link_uuid);

	if( it != edge_map.end()) 
	{
		Graph::edge_descriptor e = edge_map[link_uuid];
		edge_map.erase(it);
		kLink * l = boost::get(boost::edge_klink, graph, e);
		boost::remove_edge(e,graph);
		if(l != NULL) 
		{
			l->produce();
			delete(l);
		}
	}
	else throw std::invalid_argument("Kernel : delete klink "+link_uuid+" failed, unable to find the klink");
}

void Kernel::purge_klinks(const std::string& uuid)
{

	std::pair<out_edge_iterator, out_edge_iterator> it_out = boost::out_edges( node_map[uuid] , graph);

	for( ; it_out.first != it_out.second; ++it_out.first)
	{
		del_klink(  boost::get( boost::edge_uuid, graph, *it_out.first) );
	}

	std::pair<in_edge_iterator, in_edge_iterator> it_in = boost::in_edges(node_map[uuid] , graph);

	for( ; it_in.first != it_in.second; ++it_in.first)
	{
		del_klink(  boost::get( boost::edge_uuid, graph, *it_in.first) );
	}
}

/*******************************************************************************************************/
/****************** 			iLink Section	 			      ******************/
/*******************************************************************************************************/

void Kernel::add_ilink(const std::string& in_uuid, const XLink& xl)
{
	// Control klink exist : non constant input and klink doesn't exist -> error 
	if( !xs.isLinkCst(xl) )
	{
		if(edge_map.find( xl.uuid) == edge_map.end() ) throw std::invalid_argument( "Kernel : unable to find link "+xl.uuid );
		if( node_map.find( xl.uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to add ilink from unkown function "+xl.uuid_pred );
	}		

	if( input_to_funct.find(in_uuid) == input_to_funct.end() ) throw std::invalid_argument( "Kernel : try to add ilink to an unbind input "+in_uuid);
	
	if( node_map.find( input_to_funct[in_uuid] ) == node_map.end()) throw std::invalid_argument( "Kernel : try to add ilink to unkown function "+input_to_funct[in_uuid] );

	//TODO : replace by Builder and Factory 
	auto it = inputs.find(in_uuid);
	if( it != inputs.end() )
	{
		if(  it->second->type() == typeid(iScalar).hash_code() ) 
		{
			add_iscalar(in_uuid, xl);
		}
		else if(  it->second->type() == typeid(iScalarMatrix).hash_code() )
		{
			add_ismatrix(in_uuid, xl);
		}
		else if( it->second->type() == typeid(iMMatrix).hash_code() )
		{
			add_immatrix(in_uuid, xl);
		} 
		else throw std::invalid_argument( "Kernel : try to add ilink with unkown type : "+xl.uuid );
	} 
	else throw std::invalid_argument( "Kernel : try to add ilink to unkown input "+in_uuid );
}
	
void Kernel::add_iscalar(const std::string& in_uuid,const XLink& xl)	
{
	// Build ilink
	std::shared_ptr<iLinkBase> is (new iScalar);

	//set is
	is->setUuid(xl.uuid);
	dynamic_cast<iScalar*>(is.get())->w( xl.weight );

	if( xs.isLinkCst(xl) )
	{
		dynamic_cast<iScalar*>(is.get())->setCValue(1);
	}
	else
	{
		Function *f =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( f->type() == is->i_type())
		{
			dynamic_cast<iScalar*>(is.get())->i( &(dynamic_cast<FScalar*>(f)->getOutput()) );
	 
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type. Input "+is->i_type_name()+" on  Function "+f->type_name());
       
		if( input_to_funct[in_uuid] == xl.uuid_pred) is->activateBuffer();
	}

	// Add ilink
	inputs[in_uuid]->add(is);
	ilinks[xl.uuid] = is;
	ilink_to_input[xl.uuid] = in_uuid;
}

void Kernel::add_ismatrix(const std::string& in_uuid,const XLink& xl)
{
	// Build ilink
	std::shared_ptr<iLinkBase> ism(new iScalarMatrix);
	
	//set is
	ism->setUuid(xl.uuid);
	dynamic_cast<iScalarMatrix*>(ism.get())->w( xl.weight );

	if( xs.isLinkCst(xl) )
	{
		if( xs.constants.find(xl.uuid_pred) == xs.constants.end() ) throw std::invalid_argument("Kernel : unable to find constant : "+xl.uuid_pred);

		if( xs.constants[xl.uuid_pred].type != "MATRIX" ) throw std::invalid_argument("Kernel : constant "+xl.uuid_pred+" must be type MATRIX to be linked to Function "+in_uuid) ;

		dynamic_cast<iScalarMatrix*>(ism.get())->setCValue( MatrixXd::Constant( xs.constants[xl.uuid_pred].rows , xs.constants[xl.uuid_pred].cols , 1 )); 

	}
	else
	{
		Function *pf =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( pf->type() == ism->i_type())
		{
			dynamic_cast<iScalarMatrix*>(ism.get())->i( &(dynamic_cast<FMatrix*>(pf)->getOutput()) );
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type. Input "+ism->i_type_name()+" on  Function "+pf->type_name());
	
		if( input_to_funct[in_uuid] == xl.uuid_pred) ism->activateBuffer();
	}
	// Add ilink
	inputs[in_uuid]->add(ism);
	ilinks[xl.uuid] = ism;
	ilink_to_input[xl.uuid] = in_uuid;
}

void Kernel::add_immatrix(const std::string& in_uuid,const XLink& xl)
{
	// Build ilink
	std::shared_ptr<iLinkBase> imm;

	Function *sf =  boost::get(boost::vertex_function, graph,  node_map[input_to_funct[in_uuid]]) ;
	
	if( xl.con.type.size() == 0 ) throw std::invalid_argument("Kernel : you have to specify the connectivity for MATRIX_MATRIX Link ! ilink UUID : "+in_uuid);

	imm =  std::shared_ptr<iMMatrix>(new iMMatrix( sf->getRows(), sf->getCols()  ));

	//set is
	imm->setUuid(xl.uuid);

	if( xs.isLinkCst(xl) )
	{
		if( xs.constants.find(xl.uuid_pred) == xs.constants.end() ) throw std::invalid_argument("Kernel : unable to find constant : "+xl.uuid_pred);

		if( xs.constants[xl.uuid_pred].type != "MATRIX" ) throw std::invalid_argument("Kernal : constant "+xl.uuid_pred+" must be type MATRIX to be linked to Function "+in_uuid) ;

		dynamic_cast<iMMatrix*>(imm.get())->setCValue( MatrixXd::Constant( xs.constants[xl.uuid_pred].rows , xs.constants[xl.uuid_pred].cols , 1 )); 
	}
	else
	{
		Function *pf =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( pf->type() == imm->i_type())
		{
			dynamic_cast<iMMatrix*>(imm.get())->i( &(dynamic_cast<FMatrix*>(pf)->getOutput()) );
 
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
		
		if( input_to_funct[in_uuid] == xl.uuid_pred) imm->activateBuffer();
	}

	dynamic_cast<iMMatrix*>(imm.get())->resizeWeight();
	dynamic_cast<iMMatrix*>(imm.get())->initWeight(xl.weight);
	dynamic_cast<iMMatrix*>(imm.get())->buildFilter(xl.con);

	// Add ilink
	inputs[in_uuid]->add(imm);
	ilinks[xl.uuid] = imm;
	ilink_to_input[xl.uuid] = in_uuid;
}


void Kernel::del_ilink(const std::string& link_uuid)
{
	if( ilink_to_input.find(link_uuid ) == ilink_to_input.end() ) throw  std::invalid_argument("Kernel : unable to delete the unreferenced ilink "+link_uuid);

	std::string input = ilink_to_input[link_uuid];
	ilink_to_input.erase( ilink_to_input.find( link_uuid ));

	auto it = ilinks.find( link_uuid );

	if( it != ilinks.end())
	{
		ilinks.erase(it);
		// shared_ptr : should delete automatically the ilink
	}
	else throw std::invalid_argument( "Kernel : unable to delete ilink "+link_uuid);
}

void Kernel::purge_output_ilinks(const std::string& uuid)
{
	std::pair<out_edge_iterator, out_edge_iterator> it_out = boost::out_edges( node_map[uuid] , graph);
	
	for( ; it_out.first != it_out.second; ++it_out.first)
	{
		std::string l_uuid = boost::get( boost::edge_uuid, graph, *it_out.first); 
	
		if(  rttoken.getNode() != boost::target( *it_out.first,graph))
		{
			if( ilink_to_input.find(l_uuid) != ilink_to_input.end() ) 
			{
				std::string in_uuid = ilink_to_input[ l_uuid ];
				del_ilink(  l_uuid );
				purge_empty_links( in_uuid );	
			}
			throw std::invalid_argument("Kernel : unable to find input for ilink "+l_uuid);
		}
	}
}

/*******************************************************************************************************/
/****************** 			Runner Section	 			      ******************/
/*******************************************************************************************************/

void Kernel::add_runner(const std::string& uuid)
{
        if( node_map.find( uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add runner to an unkown function "+ uuid ); 

	FRunner * fr = new FRunner();
	fr->setGraph(&graph);
	fr->setNode( node_map[uuid] );
	boost::put(boost::vertex_runner, graph, node_map[uuid], fr);
}

void Kernel::remove_runner(const std::string& uuid)
{
        if( node_map.find( uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to remove runner to an unkown function "+ uuid ); 

	Runner * runner = boost::get(boost::vertex_runner, graph, node_map[uuid]);

	if( runner != NULL) 
	{	
		runner->terminate();
		delete(runner);
	}
}

void Kernel::spawn_runners()
{
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
	for( ; it.first != it.second; ++it.first)
	{	
		Runner * runner = boost::get(boost::vertex_runner, graph, *it.first);
	
		if( runner != NULL ) runner->spawn();
	}
}

void Kernel::quit_runners()
{
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
        for( ; it.first != it.second; ++it.first)
        {
                Runner* runner = boost::get(boost::vertex_runner, graph, *it.first);

                if( runner != NULL ) 
		{
			if( runner->wait_for_quit_timeout(wait_delay) &&  runner->joinable() )
			{	
				runner->join();
			}
			else 
			{
				runner->terminate();
        		}
		}
	}
}

void Kernel::wait_for_resume_runners()
{
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
        for( ; it.first != it.second; ++it.first)
        {
                Runner* runner = boost::get(boost::vertex_runner, graph, *it.first);

                if( runner != NULL )
                {
                        if( ! runner->wait_for_run_timeout(wait_delay))
                        {
                        	throw std::invalid_argument( "Kernel : unable to resume the kernel, RUNNER is \"locked\" in pause mode");
                        }
                }
        }

}

void Kernel::wait_for_pause_runners()
{
        std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
        for( ; it.first != it.second; ++it.first)
        {
                Runner* runner = boost::get(boost::vertex_runner, graph, *it.first);

                if( runner != NULL )
                {
                        if( ! runner->wait_for_pause_timeout(wait_delay))
                        {
                                throw std::invalid_argument( "Kernel : unable to pause the kernel, RUNNER is locked in \"resume\" mode");
                        }
                }
        }

}

/*******************************************************************************************************/
/****************** 			Bind Section 				      ******************/
/*******************************************************************************************************/

void Kernel::bind(InputBase& value,const std::string& var_name,const std::string& uuid)
{

        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind \""+var_name+"\", a "+value.type_name()+" input from unkown function "+uuid );

        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input \""+var_name+"\", a "+value.type_name()+" input from function "+uuid  );

        inputs[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = &value;        
        input_to_funct[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = uuid;
        value.setUuid( xs.functions[uuid].inputs.find(var_name)->second.uuid );

        Function *f = boost::get(boost::vertex_function, graph, node_map[uuid]) ;
        f->add_input(&value);   
}

void Kernel::bind(IString& value,const std::string& var_name,const std::string& uuid)
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a string input to unkown function "+uuid );

        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

        if( xs.isLinkCst( xs.functions[uuid].inputs[var_name].links[0]) )
        {
                value = xs.functions[uuid].inputs[var_name].links[0].value; 
        }
        else throw std::invalid_argument( "Kernel : string input have to be constant");
}

void Kernel::unbind_input(const std::string& uuid)
{
        if( inputs.find(uuid) == inputs.end()) throw std::invalid_argument("Kernel : try to clear input "+uuid+", but enable to find");

        InputBase *in = inputs[uuid];

        for( unsigned int i = 0 ; i < in->size(); i++ )
        {
               del_ilink((*in)[i].getUuid());
        }

        inputs.erase( inputs.find(uuid));
        input_to_funct.erase( input_to_funct.find(uuid));
}

void Kernel::purge_empty_links(const std::string& uuid)
{
	if( inputs.find(uuid) != inputs.end() )
        {
                inputs[uuid]->purge_empty();
        }
        else throw std::invalid_argument("Kernel : unable to purge ilink from input "+uuid );
}

/*******************************************************************************************************/
/****************** 		    	 Save Activity  			      ******************/
/*******************************************************************************************************/

bool Kernel::save_activity(const std::string& uuid, bool state)
{
	bool ret = true;
        if( node_map.find( uuid ) != node_map.end() )
        {
                Function *f =  boost::get(boost::vertex_function , graph)[ node_map[uuid]];

                if( f != NULL ) f->active_save(state);
                else ret = false;
        }

	return ret;
}

/*******************************************************************************************************/
/****************** 			Objects Section 			      ******************/
/*******************************************************************************************************/

void Kernel::get_objects(std::vector<std::string> & objects)
{
	get_rt_token(objects);
	get_functions(objects);
	get_inputs(objects);
	get_ilinks(objects);
}

void Kernel::get_inputs(std::vector<std::string> & objects)
{
	for(auto it = inputs.begin(); it != inputs.end(); ++it)
	{
		objects.push_back("input:"+it->first);
	}
}

void Kernel::get_ilinks(std::vector<std::string> & objects)
{
	for(auto it = ilinks.begin(); it != ilinks.end(); ++it)
	{
		objects.push_back("link:"+it->first);
	}
}

void Kernel::get_functions(std::vector<std::string> & objects)
{
	for(auto it = node_map.begin(); it != node_map.end(); ++it)
	{
		if( it->first != rttoken.getUuid() ) objects.push_back("function:"+it->first);
	}
}

void Kernel::get_rt_token(std::vector<std::string> & objects)
{
	objects.push_back("rt_token:"+rttoken.getUuid());
}

bool Kernel::find_object(const std::string& uuid)
{
	bool ret = false;

	// Active RtToken Output
        if( uuid == rttoken.getUuid() )
        {
		ret = true;
        }
        // Active Function Output
        else if( node_map.find( uuid ) != node_map.end() )
        {
		ret = true;
        }
        // Active ilink Output
        else if( ilinks.find( uuid ) != ilinks.end() )
        {
		ret = true;
        }

	return ret;
}

/*******************************************************************************************************/
/****************** 			Publish Section 			      ******************/
/*******************************************************************************************************/

bool Kernel::active_publish(const std::string& uuid, bool state)
{
	bool ret = true;

	// Active RtToken Output
	if( uuid == rttoken.getUuid() )
	{
		rttoken.active_publish(state);
	}
	// Active Function Output 
	else if( node_map.find( uuid ) != node_map.end() )
	{
		Function *f =  boost::get(boost::vertex_function , graph)[ node_map[uuid]];

		if( f != NULL ) f->active_publish(state);
		else ret = false;
	}
	// Active ilink Output
	else if( ilinks.find( uuid ) != ilinks.end() ) 
	{
		ilinks[uuid]->active_publish(state);		
	}
	else 
	{
		ret = false;
	}

	return ret;
}

void Kernel::publish_status(StatusMessage &message)
{	
	if( k_pub == NULL ) return;
	if( !k_pub->is_open()) return; 

	k_pub->setMessage(message);
	k_pub->publish();
}

void Kernel::active_status(bool state)
{
        if(state)
        {
                if( k_pub != NULL )
                {
                        if( !k_pub->is_open()){
				std::string name = "status";
				ComInterface::setDefaultName(name);
				k_pub->setPubName(name);
			       	k_pub->open();
			}
                }
                else throw std::invalid_argument("Kernel : failed to open status publisher");
        }
        else
        {
                if( k_pub != NULL)
                {
                        if( k_pub->is_open() )  k_pub->close();
                }
        }
}

/*******************************************************************************************************/
/**************** 			Static Section 				     *******************/
/*******************************************************************************************************/

void Kernel::init(std::string script_file, std::string weight_file, bool ignore_matrix_check)
{
	
	namespace fs = boost::filesystem;
	fs::path full_path = fs::system_complete(script_file);
	singleton.script_file=full_path.c_str();

	XmlConverter * xmlc = new  XmlConverter(script_file);
	xmlc->loadScript(singleton.xs);
	delete xmlc;

	std::cout << "Run : " << singleton.xs.name << " script"<< std::endl;

	singleton.init_rt_token();

	if( weight_file.size() == 0) 
	{
		singleton.weight_file=singleton.xs.name+".weight";
	}
	else
	{
		singleton.weight_file=weight_file;
	}

	singleton.ignore_matrix_check = ignore_matrix_check;
}


void Kernel::quit()
{
	StatusMessage m;

	try{
		Runner::ask_stop();

		// Call Function::onQuit 
		singleton.onQuit_functions();
		singleton.quit_runners();

		m.key = CMD[C_CONTROL] ;
		m.value = CARG[S_QUIT];
		singleton.publish_status(m);
	}
	catch(...)
	{
	  m.key = CMD[C_CONTROL] ;
	  m.value = CARG[S_QUIT]+"_failed";
	  singleton.publish_status(m);

	  std::exception_ptr eptr = std::current_exception(); 
	  if (eptr) 
	  { 
		  std::rethrow_exception(eptr);
	  }
	}
}

void Kernel::resume()
{
	StatusMessage m;

	try{
		// Call onResume
		singleton.onRun_functions();
	
		Runner::ask_resume();	
        	singleton.wait_for_resume_runners();

		m.key = CMD[C_CONTROL] ;
		m.value = CARG[S_RESUME];
		singleton.publish_status(m);
	}
	catch(...)
	{
	  m.key = CMD[C_CONTROL] ;
	  m.value = CARG[S_RESUME]+"_failed";
	  singleton.publish_status(m);

	  std::exception_ptr eptr = std::current_exception(); 
	  if (eptr) 
	  { 
		  std::rethrow_exception(eptr);
	  }
	}
}

void Kernel::pause()
{
	StatusMessage m;

	try{
		Runner::ask_pause();
	
		// Call onPause
		singleton.onPause_functions();
		singleton.wait_for_pause_runners();
	
		m.key = CMD[C_CONTROL] ;
		m.value = CARG[S_PAUSE];
		singleton.publish_status(m);
	}
	catch(...)
	{
	  m.key = CMD[C_CONTROL] ;
	  m.value = CARG[S_PAUSE]+"_failed";
	  singleton.publish_status(m);

	  std::exception_ptr eptr = std::current_exception(); 
	  if (eptr) 
	  {
		  std::rethrow_exception(eptr);
	  }
	}
}

void Kernel::load()
{
	singleton.load_functions();
        singleton.load_links();
        singleton.load_rttoken();
        singleton.load_weight();
}

void Kernel::prerun()
{
	singleton.prerun_functions();
}

void Kernel::start(bool run)
{ 	
	singleton.active_status(true);
	singleton.spawn_runners();
	if( run ){
	       	resume();
	}
	else pause();
}

void Kernel::sweight_save(std::string& path)
{
	StatusMessage m;
	std::exception_ptr eptr;

	pause();
	
	try
	{
		if( path.size() == 0) singleton.save_weight();
		else singleton.save_weight(path);

		m.key = CMD[C_WEIGHT] ;
		m.value = CARG[S_SAVE];
		singleton.publish_status(m);
	}
	catch(...)
	{
		m.key = CMD[C_WEIGHT] ;
		m.value = CARG[S_SAVE]+"_failed";
		singleton.publish_status(m);

		eptr = std::current_exception(); 
		if (eptr) 
		{
			std::rethrow_exception(eptr);
		}
	}
	resume();
}

void Kernel::sweight_load(std::string& path)
{
	StatusMessage m;
	std::exception_ptr eptr;

	pause();

	try{
		if( path.size() == 0) singleton.load_weight();
		else singleton.load_weight(path);

	  	m.key = CMD[C_WEIGHT] ;
		m.value = CARG[S_LOAD];
		singleton.publish_status(m);
	}
	catch(...)
	{
		m.key = CMD[C_WEIGHT] ;
		m.value = CARG[S_LOAD]+"_failed";
		singleton.publish_status(m);

		eptr = std::current_exception(); 
		if (eptr) 
		{
			std::rethrow_exception(eptr);
		}
	}
	resume();
}

void Kernel::active_oscillo(bool order)
{
	//StatusMessage m;
        //std::exception_ptr eptr;

	pause();
	singleton.rttoken.active_oscillo(order);
	resume();	

	/*
	try{
        	singleton.rttoken.active_oscillo(order);

		m.key = CMD[C_OSCILLO];
		if( order ) m.value = CARG[S_START];
		else m.value = CARG[S_STOP];
		singleton.publish_status(m);
	}
        catch(...)
	{
		m.key = CMD[C_OSCILLO];
                if( order ) m.value = CARG[S_START]+"_failed";
                else m.value = CARG[S_STOP]+"_failed";
		singleton.publish_status(m);
		
		eptr = std::current_exception(); 
		if (eptr) 
		{
			std::rethrow_exception(eptr);
		}
	}	
	resume();	
	*/

}

void Kernel::active_output(const std::string& uuid, bool order)
{
	bool state = is_pause();
	pause();

	if( ! singleton.active_publish( uuid, order) )
	{
		//TODO publish Message : failed
	}

	if( !state ) resume();
}

void Kernel::active_rt_token(bool order)
{
	bool state = is_pause();
	pause();
	singleton.rttoken.active_publish(order);
	if( !state ) resume();
}

void Kernel::active_save_activity(const std::string& uuid, bool order)
{
	bool state = is_pause();
	pause();
	singleton.save_activity(uuid,order);
	if( !state ) resume();
}

void Kernel::iBind( InputBase& value,const std::string& var_name,const std::string& uuid )
{
	singleton.bind(value,var_name,uuid);
}

void Kernel::iBind( IString& value,const std::string& var_name,const std::string& uuid )
{
	singleton.bind(value,var_name,uuid);
}
