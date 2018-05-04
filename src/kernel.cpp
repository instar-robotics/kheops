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

#include <iostream>
#include "kernel.h"
#include "factory.h"
#include "util.h"
#include "frunner.h"
#include "weightconverter.h"

Kernel Kernel::singleton;

Kernel::~Kernel()
{	
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL ) delete(f);
	}
	
	for( auto it = edge_map.begin(); it != edge_map.end(); it++)
	{
		kLink *l = boost::get(boost::edge_klink , graph)[ it->second  ];
		if( l != NULL ) delete(l);
	}

	for( auto it = is_ilink.begin(); it != is_ilink.end(); it++) 
	{
		is_ilink.erase(it);
	}
	
	for( auto it = ism_ilink.begin(); it != ism_ilink.end(); it++) 
	{
		ism_ilink.erase(it);
	}

	for( auto it = imm_ilink.begin(); it != imm_ilink.end(); it++) 
	{
		imm_ilink.erase(it);
	}

	graph.clear();
	node_map.clear();
	edge_map.clear();

	ilink_to_input.clear();
	input_to_funct.clear();
	
	is_input.clear();
	ism_input.clear();
	imm_input.clear();
}

/********************************************************************************************************/
/****************** 			Load Section	 			      *******************/
/********************************************************************************************************/

void Kernel::load_functions()
{
	//build function using Factory
	for(auto it = xs.functions.begin(); it != xs.functions.end(); it++)
	{
		if( Factory<Function>::Instance().is_register(it->second.name) )
		{
			add_function( it->second );	
		}	
		else
		{
//			std::cout << "Function "+it->second.name+" is not a native function. Try to load it" << std::endl;
			throw  std::invalid_argument("Kernel : Unable to load Function "+it->second.name);
		}
	}
	//setparameters : call bind function for each inputs
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL )
		{
		 	f->setparameters();
		}
	}
}


void Kernel::load_links()
{
	for(auto funct = xs.functions.begin(); funct != xs.functions.end(); funct++)
	{
		for( auto input = funct->second.inputs.begin(); input != funct->second.inputs.end(); input++)
		{
			for( auto link = input->second.links.begin(); link !=  input->second.links.end(); link++)
			{
				if( !link->isCst )
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
	update_rt_token_value(xs.rt);
	create_rt_klink();
	clear_rt_klink();
}

void Kernel::load_weight()
{
	WeightConverter wc(weight_file);
	wc.load(imm_input);
}

void Kernel::load_weight(const std::string& filename)
{
	std::string tmp_file = filename;
	if( !check_file_extension(filename, ".weight")) tmp_file+=".weight";

	WeightConverter wc(tmp_file);
	wc.load(imm_input);
}

void Kernel::save_weight()
{
	WeightConverter wc(weight_file);
	wc.save(imm_input);
}

void Kernel::save_weight(const std::string& filename)
{
	std::string tmp_file = filename;
	if( !check_file_extension(filename, ".weight")) tmp_file+=".weight";

	WeightConverter wc(tmp_file);
	wc.save(imm_input);
}

/********************************************************************************************************/
/****************** 			Function Section 			      *******************/
/********************************************************************************************************/

void Kernel::add_function( const XFunction& xf)
{
      Function *f = Factory<Function>::Instance().create(xf.name);
      if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+xf.name);
      else
      { 
		f->setUuid(xf.uuid);
		f->set_topic_name(xf.topic_name);
		f->active_publish(xf.publish);
		f->active_save(xf.save);

		if( f->type() == typeid(MatrixXd).hash_code()) 		
		{
		  if( xf.rows <= 0 || xf.cols <= 0)  throw  std::invalid_argument("Kernel : Can't build Matrix without valid rows/cols value");
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

	//purge all ilinks in all inputs
	for( auto input = f->get_isinput().begin() ; input !=  f->get_isinput().end(); input++)
	{
		unbind_isinput( (*input)->getUuid());
	}

	for( auto input = f->get_isminput().begin(); input !=  f->get_isminput().end(); input++)
	{
		unbind_isminput( (*input)->getUuid());
	}
	
	for( auto input = f->get_imminput().begin(); input !=  f->get_imminput().end(); input++)
	{
		unbind_imminput( (*input)->getUuid());
	}

	// purge output link
	purge_output_ilinks(uuid);
	
	// purge all klinks for the function
	purge_klinks(uuid);
	
	// remove from runner
	remove_runner(uuid);
	
	clear_vertex(node_map[uuid] , graph);
	remove_vertex( node_map[uuid] , graph);
	node_map.erase( node_map.find(uuid));

	delete(f);
}

/********************************************************************************************************/
/****************** 		        	RTTOKEN Section	              	      *******************/
/********************************************************************************************************/

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

/********************************************************************************************************/
/****************** 			        kLink Section			      *******************/
/********************************************************************************************************/

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

/********************************************************************************************************/
/****************** 			iLink Section	 			      *******************/
/********************************************************************************************************/

void Kernel::add_ilink(const std::string& in_uuid, const XLink& xl)
{
	// Control klink exist : non constant input and klink doesn't exist -> error 

	if( !xl.isCst)
	{
		if(edge_map.find( xl.uuid) == edge_map.end() ) throw std::invalid_argument( "Kernel : unable to find link "+xl.uuid );
		if( node_map.find( xl.uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to add ilink from unkown function "+xl.uuid_pred );
	}		

	if( input_to_funct.find(in_uuid) == input_to_funct.end() ) throw std::invalid_argument( "Kernel : try to add ilink to an unbind input "+in_uuid);
	
	if( node_map.find( input_to_funct[in_uuid] ) == node_map.end()) throw std::invalid_argument( "Kernel : try to add ilink to unkown function "+input_to_funct[in_uuid] );

	if( is_input.find(in_uuid) != is_input.end() )
	{
		add_iscalar(in_uuid, xl);
	}
	else if( ism_input.find(in_uuid) != ism_input.end())
	{
		add_ismatrix(in_uuid, xl);
	}
	else if( imm_input.find(in_uuid) != imm_input.end())
	{
		add_immatrix(in_uuid, xl);
	}
	else throw std::invalid_argument( "Kernel : try to add ilink to unkown input "+in_uuid );
}
	
void Kernel::add_iscalar(const std::string& in_uuid,const XLink& xl)	
{
	// Build ilink
	std::shared_ptr<IScalar> is (new IScalar);

	//set is
	is->setUuid(xl.uuid);
	is->w( xl.weight );

	if( xl.isCst == true )
	{
		is->setCValue(1);
	}
	else
	{
		Function *f =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( f->type() == is->type())
		{
			is->i( &(dynamic_cast<FScalar*>(f)->getOutput()) );
	 
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type. Input "+is->type_name()+" on  Function "+f->type_name());
       
		if( input_to_funct[in_uuid] == xl.uuid_pred) is->activateBuffer();
	}

	// Add ilink
	is_input[in_uuid]->add(is);
	is_ilink[xl.uuid] = is;
	ilink_to_input[xl.uuid] = in_uuid;
}

void Kernel::add_ismatrix(const std::string& in_uuid,const XLink& xl)
{
	// Build ilink
	std::shared_ptr<IScalarMatrix> ism(new IScalarMatrix);
	
	Function *sf =  boost::get(boost::vertex_function, graph,  node_map[input_to_funct[in_uuid]]) ;

	//set is
	ism->setUuid(xl.uuid);
	ism->w( xl.weight );

	if( xl.isCst == true )
	{
		ism->setCValue( MatrixXd::Constant( sf->getRows(),sf->getCols(), 1 )); 
	}
	else
	{
		Function *pf =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( pf->type() == ism->type())
		{
			//check size (rows/cols). All inputs must have the same size 
			if( pf->getRows() != sf->getRows() || pf->getCols() != sf->getCols()) throw  std::invalid_argument( "Kernel : try to bind scalar matrix with different sizes ");

			ism->i( &(dynamic_cast<FMatrix*>(pf)->getOutput()) );
	 
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type. Input "+ism->type_name()+" on  Function "+pf->type_name());
	
		if( input_to_funct[in_uuid] == xl.uuid_pred) ism->activateBuffer();
	}
	// Add ilink
	ism_input[in_uuid]->add(ism);
	ism_ilink[xl.uuid] = ism;
	ilink_to_input[xl.uuid] = in_uuid;
}

void Kernel::add_immatrix(const std::string& in_uuid,const XLink& xl)
{
	// Build ilink
	std::shared_ptr<IMMatrix> imm;

	Function *sf =  boost::get(boost::vertex_function, graph,  node_map[input_to_funct[in_uuid]]) ;
	
	if( xl.isSparse == true) 
	{
		imm =  std::shared_ptr<IMMatrix>(new ISparseMatrix( sf->getRows(), sf->getCols(), 0  ));
		//TODO : add filter management 
	}
	else
	{
		imm =  std::shared_ptr<IMMatrix>(new IDenseMatrix( sf->getRows(), sf->getCols(), 0  ));
	}
	
	//set is
	imm->setUuid(xl.uuid);

	if( xl.isCst == true )
	{
		//By default constant are 1x1 Matrix 
		imm->setCValue( MatrixXd::Constant( 1, 1 , 1 )); 

		//TODO : should be possible to and rows/cols size into XLink to custom the constant size
		//imm->setCValue( MatrixXd::Constant( sf->getRows(),sf->getCols(), std::stod( it->value ))); 
	}
	else
	{
		Function *pf =  boost::get(boost::vertex_function, graph, node_map[xl.uuid_pred]) ;
		if( pf->type() == imm->type())
		{
			imm->i( &(dynamic_cast<FMatrix*>(pf)->getOutput()) );
 
		}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
		
		if( input_to_funct[in_uuid] == xl.uuid_pred) imm->activateBuffer();
	}
	imm->resizeWeight();
	imm->initWeight(xl.weight);

	// Add ilink
	imm_input[in_uuid]->add(imm);
	imm_ilink[xl.uuid] = imm;
	ilink_to_input[xl.uuid] = in_uuid;
}


void Kernel::del_ilink(const std::string& link_uuid)
{
	if( ilink_to_input.find(link_uuid ) == ilink_to_input.end() ) throw  std::invalid_argument("Kernel : unable to delete the unreferenced ilink "+link_uuid);

	std::string input = ilink_to_input[link_uuid];
	ilink_to_input.erase( ilink_to_input.find( link_uuid ));

	auto it = is_ilink.find( link_uuid );

	if( it != is_ilink.end())
	{
		is_ilink.erase(it);
		// shared_ptr : should delete automatically the ilink
	}
	else 
	{
		auto it = ism_ilink.find( link_uuid );
		if( it != ism_ilink.end())
		{
			ism_ilink.erase(it);
			// shared_ptr : should delete automatically the ilink
		}
		else
		{
			auto it = imm_ilink.find( link_uuid );
			if( it != imm_ilink.end())
			{
				imm_ilink.erase(it);
			}
			else throw std::invalid_argument( "Kernel : unable to delete ilink "+link_uuid);
		}

	}
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

/********************************************************************************************************/
/****************** 			Runner Section	 			      *******************/
/********************************************************************************************************/

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
	
	runner->terminate();
	delete(runner);
}

void Kernel::spawn_runners()
{
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
	for( ; it.first != it.second; ++it.first)
	{	
		Runner * runner = boost::get(boost::vertex_runner, graph, *it.first);
	
		runner->spawn();
	}
}

void Kernel::join_runners()
{
	std::pair<vertex_iter, vertex_iter> it = boost::vertices(graph);
	for( ; it.first != it.second; ++it.first)
	{	
		Runner* runner = boost::get(boost::vertex_runner, graph, *it.first);
	
		runner->join();
	}
}


/********************************************************************************************************/
/****************** 			Bind Section 				      *******************/
/********************************************************************************************************/

void Kernel::bind(ISInput& value,const  std::string& var_name,const std::string& uuid)
{

        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind \""+var_name+"\", a SCALAR_SCALAR input from unkown function "+uuid );

        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input \""+var_name+"\", a SCALAR_SCALAR input from function "+uuid  );

	if( value.isMultiple() != xs.functions[uuid].inputs.find(var_name)->second.multiple)
	{
		if(  value.isMultiple()  ) throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as multiple and mark as not multiple in XML file" );
		else throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as not multiple and mark as multiple in XML file" ); 
	}

	is_input[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = &value;	
	input_to_funct[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = uuid;
	value.setUuid(  xs.functions[uuid].inputs.find(var_name)->second.uuid  );

	Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid]) ;
	f->add_input(&value);	
}

void Kernel::bind(ISMInput& value,const std::string &var_name,const std::string& uuid)
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind \""+var_name+"\", a SCALAR_MATRIX input to unkown function "+uuid );
                
        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input \""+var_name+"\", a SCALAR_MATRIX input from function "+uuid  );
	
	if( value.isMultiple() != xs.functions[uuid].inputs.find(var_name)->second.multiple)
	{
		if(  value.isMultiple()  ) throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as multiple and mark as not multiple in XML file" );
		else throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as not multiple and mark as multiple in XML file" ); 
	}

	ism_input[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] =  &value;
	input_to_funct[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = uuid;
	value.setUuid(  xs.functions[uuid].inputs.find(var_name)->second.uuid  );
	
	Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid]) ;
	f->add_input(&value);	
}

void Kernel::bind(IMMInput& value,const std::string& var_name,const std::string& uuid )
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind \""+var_name+"\", a MATRIX_MATRIX input to unkown function "+uuid );
                
        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input \""+var_name+"\", a MATRIX_MATRIX input from function "+uuid  );
	
	if( value.isMultiple() != xs.functions[uuid].inputs.find(var_name)->second.multiple)
	{
		if(  value.isMultiple()  ) throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as multiple and mark as not multiple in XML file" );
		else throw std::invalid_argument( "Kernel : input \""+var_name+"\" is define as not multiple and mark as multiple in XML file" ); 
	}

	imm_input[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] =  &value;
	input_to_funct[ xs.functions[uuid].inputs.find(var_name)->second.uuid ] = uuid;
	value.setUuid(  xs.functions[uuid].inputs.find(var_name)->second.uuid  );

	Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid]) ;
	f->add_input(&value);	
}


void Kernel::bind(std::string& value,const std::string& var_name,const std::string& uuid)
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a string input to unkown function "+uuid );

        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

        if(  xs.functions[uuid].inputs[var_name].links.size() != 1 ) throw std::invalid_argument( "Kernel : string input "+var_name+" should have only one link. "+ std::to_string(xs.functions[uuid].inputs[var_name].links.size())+" given" );
        if( xs.functions[uuid].inputs[var_name].links[0].isSparse == true) throw std::invalid_argument( "Kernel : string input can't be sparse type");

        if( xs.functions[uuid].inputs[var_name].links[0].isCst == true )
        {
                value = xs.functions[uuid].inputs[var_name].links[0].value; 
        }
        else throw std::invalid_argument( "Kernel : string input have to be constant");
}


void Kernel::unbind_isinput(const std::string& uuid)
{
	if( is_input.find(uuid) == is_input.end()) throw std::invalid_argument("Kernel : try to clear scalar input "+uuid+", but enable to find");

	ISInput *input = is_input[uuid];
	
	for( unsigned int i = 0 ; i < input->size(); i++ )
	{
	       del_ilink((*input)[i].getUuid());
	}
	
	is_input.erase( is_input.find(uuid)); 
	input_to_funct.erase( input_to_funct.find(uuid));
}

void Kernel::unbind_isminput(const std::string& uuid)
{
	if( ism_input.find(uuid) == ism_input.end()) throw std::invalid_argument("Kernel : try to clear scalar matrix input "+uuid+", but enable to find");

	ISMInput *input = ism_input[uuid];

	for( unsigned int i = 0 ; i < input->size(); i++ )
	{
	       del_ilink((*input)[i].getUuid());
	}
	
	ism_input.erase( ism_input.find(uuid)); 
	input_to_funct.erase( input_to_funct.find(uuid));
}

void Kernel::unbind_imminput(const std::string& uuid)
{
	if( imm_input.find(uuid) == imm_input.end()) throw std::invalid_argument("Kernel : try to clear matrix matrix input "+uuid+", but enable to find");

	IMMInput *input = imm_input[uuid];

	for( unsigned int i = 0 ; i < input->size(); i++ )
	{
	       del_ilink((*input)[i].getUuid());
	}
	
	imm_input.erase( imm_input.find(uuid)); 
	input_to_funct.erase( input_to_funct.find(uuid));
}


void Kernel::purge_empty_links(const std::string& uuid)
{
	if( is_input.find(uuid) != is_input.end() )
        {
                is_input[uuid]->purge_empty();
        }
        else if( ism_input.find(uuid) != ism_input.end() )
        {
                ism_input[uuid]->purge_empty();
        }
        else if( imm_input.find(uuid) != imm_input.end() )
        {
                imm_input[uuid]->purge_empty();
        }
        else throw std::invalid_argument("Kernel : unable to purge ilink from input "+uuid );
}


/********************************************************************************************************/
/****************** 			CMD Section 				      *******************/
/********************************************************************************************************/

void Kernel::resume()
{
	rttoken.ask_resume();
}

void Kernel::pause()
{
 	rttoken.ask_pause();
        rttoken.wait_for_pause();
}

/********************************************************************************************************/
/****************** 			Publish Section 			      *******************/
/********************************************************************************************************/

bool Kernel::active_publish(const std::string& uuid, bool state)
{

	bool ret = true;
	//TODO : Should Be useful to add an Object class which factorises some members and methods

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
	// Active Scalar ilink Output
	else if( is_ilink.find( uuid ) != is_ilink.end() ) 
	{
		is_ilink[uuid]->active_publish(state);		
	}
	// Active ScalarMatrix ilink Output
	else if( ism_ilink.find( uuid ) != ism_ilink.end() )
	{
		ism_ilink[uuid]->active_publish(state);		
	}
	// Active MatrixMatrix ilink Output
	else if( imm_ilink.find( uuid ) != imm_ilink.end() )
	{
		imm_ilink[uuid]->active_publish(state);		
	}
	else 
	{
		ret = false;
	}

	return ret;
}

/********************************************************************************************************/
/****************** 			Objects Section 			      *******************/
/********************************************************************************************************/

void Kernel::get_objects(std::vector<std::string> & objects)
{
	get_rt_token(objects);
	get_functions(objects);
	get_inputs(objects);
	get_ilinks(objects);
}

void Kernel::get_inputs(std::vector<std::string> & objects)
{
	for(auto it = is_input.begin(); it != is_input.end(); ++it)
	{
		objects.push_back("input:"+it->first);
	}
	
	for(auto it = ism_input.begin(); it != ism_input.end(); ++it)
	{
		objects.push_back("input:"+it->first);
	}
	
	for(auto it = imm_input.begin(); it != imm_input.end(); ++it)
	{
		objects.push_back("input:"+it->first);
	}
}

void Kernel::get_ilinks(std::vector<std::string> & objects)
{
	for(auto it = is_ilink.begin(); it != is_ilink.end(); ++it)
	{
		objects.push_back("link:"+it->first);
	}
	
	for(auto it = ism_ilink.begin(); it != ism_ilink.end(); ++it)
	{
		objects.push_back("link:"+it->first);
	}
	
	for(auto it = imm_ilink.begin(); it != imm_ilink.end(); ++it)
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


/********************************************************************************************************/
/****************** 			Static Section 				      *******************/
/********************************************************************************************************/

void Kernel::init(std::string script_file, std::string weight_file)
{
	singleton.script_file=script_file;

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
}

void Kernel::terminate()
{
	singleton.rttoken.ask_stop();
	singleton.join_runners();
}

void Kernel::load()
{
	singleton.load_functions();
        singleton.load_links();
        singleton.load_rttoken();
        singleton.load_weight();
}

void Kernel::start(bool run)
{
        singleton.spawn_runners();
	if( run ) singleton.resume();
}
