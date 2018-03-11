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
#include <iostream>

#include "kernel.h"
#include "factory.h"
#include "util.h"
#include "frunner.h"
#include "rttoken.h"

Kernel Kernel::singleton;

Kernel::~Kernel()
{	
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL ) delete(f);
	}

	graph.clear();
	node_map.clear();
}

void Kernel::init(std::string scriptfile, std::string resfile, std::string libdir)
{
	singleton.scriptfile=scriptfile;
	singleton.resfile=resfile;
	singleton.libdir = libdir;

	XmlConverter * xmlc = new  XmlConverter(scriptfile);
	xmlc->loadScript(singleton.xs);
	delete xmlc;

	std::cout << "Run : " << singleton.xs.name << " script"<< std::endl;
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
			add_function( buildFunction( it->second ) );	
		}	
		else
		{
			std::cout << "Function "+it->second.name+" is not a native function. Try to load it" << std::endl;
		}
	}
	
	//setparameters : call bind function for each inputs
	for( auto it = node_map.begin(); it != node_map.end(); it++)
	{
		Function *f = boost::get(boost::vertex_function , graph)[ it->second  ];
		if( f != NULL ) f->setparameters();
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
					if( node_map.find( link->uuid_pred ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add link with unkown source "+ link->uuid_pred );
					if( node_map.find( funct->second.uuid  ) == node_map.end()) throw   std::invalid_argument( "Kernel : try to add link with unkown target "+funct->second.uuid );

					std::pair<Graph::edge_descriptor, bool> e = add_edge( node_map[ link->uuid_pred], node_map[funct->second.uuid]  , graph );
					boost::put( boost::edge_type, graph, e.first , link->isSecondary );
				}
			}
		}
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


/********************************************************************************************************/
/****************** 			Function Section 			      *******************/
/********************************************************************************************************/

Function* Kernel::buildFunction(const XFunction& xf)
{
      Function *f = Factory<Function>::Instance().create(xf.name);
      if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+xf.name);
      else
      { 
		f->setUuid(xf.uuid);
		if( f->type() == typeid(MatrixXd).hash_code()) 		
		{
		  //TODO : Check size here ?
		  dynamic_cast<FMatrix*>(f)->setSize(xf.rows,xf.cols);
		}
      }
      return f;
}

void Kernel::add_function( Function *funct  )
{
	if( funct != NULL) 
	{
		if( node_map.find( funct->getUuid() ) == node_map.end() ) 
		{
			Graph::vertex_descriptor vd = boost::add_vertex(graph);
			boost::put(boost::vertex_function, graph, vd, funct );
			boost::put(boost::vertex_name, graph, vd, -1);
			node_map[funct->getUuid()] = vd;
		}
		else throw  std::invalid_argument("Kernel : try to add a function with an existed uuid");
	}
	else throw  std::invalid_argument("Kernel : try to add Null Function");
}

void Kernel::add_function_suc(std::string Fct, std::string pred_uuid, int rows, int cols)
{
        std::string uuid = generate_uuid();
	
	if( node_map.find( pred_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ pred_uuid );

	if( Factory<Function>::Instance().is_register(Fct) )
	{
		Function *f = Factory<Function>::Instance().create(Fct);
      		if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+Fct);
     		else{
                	f->setUuid(uuid);

			if( rows == -1 ){ rows=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getRows();}
			if( cols == -1 ){ cols=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getCols();}

			if( f->type() == typeid(MatrixXd).hash_code()) 		
			{
			  dynamic_cast<FMatrix*>(f)->setSize(rows,cols);
			}
	
			// Add function to graph
			add_function(f);

			// add link between predecessor and new node 
			std::pair<Graph::edge_descriptor, bool> e = add_edge( node_map[pred_uuid] , node_map[uuid], graph  );
			boost::put( boost::edge_weight, graph, e.first , new Synchronized_Link()); 
			
			// add link between new node and RT Token
			std::pair<Graph::edge_descriptor, bool>  rt_e = add_edge( node_map[uuid], RtToken::instance().getRtNode() , graph  );
			boost::put( boost::edge_weight, graph, rt_e.first, new Synchronized_Link()); 

			// add a runner to the new node
			add_runner(uuid);
      		}
	}
}


void Kernel::add_function_pred(std::string Fct, std::string suc_uuid, int rows, int cols)
{
        std::string uuid = generate_uuid();
    
        if( node_map.find( suc_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ suc_uuid );
        
        if( Factory<Function>::Instance().is_register(Fct) )
        {
                Function *f = Factory<Function>::Instance().create(Fct);
                if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+Fct);
                else{
                        f->setUuid(uuid);

                        if(rows == -1){ rows=boost::get(boost::vertex_function, graph)[node_map[suc_uuid]]->getRows();}
                        if(cols == -1){ cols=boost::get(boost::vertex_function, graph)[node_map[suc_uuid]]->getCols();}

			if( f->type() == typeid(MatrixXd).hash_code()) 		
			{
			  dynamic_cast<FMatrix*>(f)->setSize(rows,cols);
			}

			// Add Node to the graph
                        add_function(f);

			// Add link between new node and successor 
			std::pair<Graph::edge_descriptor, bool>  e = add_edge( node_map[uuid],  node_map[suc_uuid] , graph  );
			boost::put( boost::edge_weight, graph , e.first, new Synchronized_Link()); 
			            
			// add a runner to the new node
			add_runner(uuid);
                }
        }
}


void Kernel::insert_function(std::string Fct, std::string pred_uuid, std::string suc_uuid ,int rows, int cols)
{
        std::string uuid = generate_uuid();

        if( node_map.find( pred_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ pred_uuid ); 
        if( node_map.find( suc_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ suc_uuid ); 

        if( Factory<Function>::Instance().is_register(Fct) )
        {
                Function *f = Factory<Function>::Instance().create(Fct);
                if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+Fct);
                else{
                        f->setUuid(uuid);
        
                        if(rows==-1){ rows=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getRows();}
                        if(cols==-1){ cols=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getCols();}

			if( f->type() == typeid(MatrixXd).hash_code()) 		
			{
			  dynamic_cast<FMatrix*>(f)->setSize(rows,cols);
			}
  
			// Add Node to the graph
                        add_function(f);

			// Add link between predecessor and new node
			std::pair<Graph::edge_descriptor, bool> ep = add_edge( node_map[pred_uuid] , node_map[uuid] ,graph  );
			boost::put( boost::edge_weight, graph, ep.first, new Synchronized_Link()); 
			
			// Add link between new node and successor
			std::pair<Graph::edge_descriptor, bool> es = add_edge( node_map[uuid], node_map[suc_uuid]  ,graph  );
			boost::put( boost::edge_weight, graph, es.first, new Synchronized_Link()); 

			// Add a new Runner 
			add_runner(uuid);
                }
        }
}

void Kernel::del_function( Function * funct)
{
	if( funct == NULL) throw  std::invalid_argument( "Kernel : try to delete an NULL Function ");
	if( node_map.find( funct->getUuid()  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to delete an unknown function "+ funct->getUuid()  );   
	clear_vertex( node_map[funct->getUuid()] , graph);
	remove_vertex( node_map[funct->getUuid()] , graph);

	delete(funct);
}

void Kernel::del_function(const std::string& uuid)
{
	if( node_map.find( uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to del an unknown function "+ uuid );   

	Function *f =  boost::get(boost::vertex_function , graph)[ node_map[uuid]];

	clear_vertex( node_map[uuid] , graph);
	remove_vertex( node_map[uuid] , graph);

	delete(f);
}

void Kernel::add_rttoken()
{
	Graph::vertex_descriptor rt_node = boost::add_vertex(graph);

	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
        {
		if( *it.first != rt_node)
		{
			auto it_out = boost::out_edges(*it.first, graph);
			if( it_out.first == it_out.second)
			{
				add_edge(*it.first, rt_node  ,graph  );
			}
		}
        }

	RtToken::instance().setToken(xs.rt.value , xs.rt.unit );
	RtToken::instance().setGraph(&graph);
	RtToken::instance().setRtNode(rt_node);
	boost::put(boost::vertex_name, graph, rt_node, 0);
}

void Kernel::del_link(std::string pred_uuid, std::string suc_uuid)
{
	if( node_map.find( pred_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ pred_uuid );   
        if( node_map.find( suc_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ suc_uuid );

	Graph::edge_descriptor e;
	bool succes;
	tie(e, succes) = boost::edge( node_map[pred_uuid], node_map[suc_uuid], graph );

	if (succes)
	{	
		Link * l = boost::get(boost::edge_weight, graph, e);
		boost::remove_edge(e, graph);
		l->produce();
		delete(l);
	}	
}

/********************************************************************************************************/
/****************** 			Runner Section	 			      *******************/
/********************************************************************************************************/

void Kernel::separate_runner_allocation()
{
	// Build all runners
	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
	{
		Graph::vertex_descriptor v =  *it.first ;

		if( boost::get( boost::vertex_name , graph, v) == -1) 
		{
			FRunner * fr = new FRunner();
			fr->setGraph(&graph);
			FRunner::add(fr);
			fr->add_node( v );

			boost::put(boost::vertex_name, graph, v, fr->getId());
		}
	}

	// Create all links
	for( auto it = boost::edges(graph); it.first != it.second; ++it.first)
	{
		bool type = boost::get( boost::edge_type, graph)[*it.first];
		boost::put(boost::edge_weight, graph, *it.first , new Synchronized_Link(type));
	}
}

void Kernel::runner_allocation()
{
	this->separate_runner_allocation();
}

void Kernel::add_runner(const std::string& uuid)
{
	// NB : naive runner strategy : could add the function to pred'runner or remap all the runner using the graph
	this->add_separate_runner(uuid);
}

void Kernel::add_separate_runner(const std::string& uuid)
{
        if( node_map.find( uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add runner to an unkown function "+ uuid ); 

	FRunner * fr = new FRunner();
	fr->setGraph(&graph);
	FRunner::add(fr);
	fr->add_node( node_map[uuid] );
	boost::put(boost::vertex_name, graph, node_map[uuid], fr->getId());
			
	// Spawn the runner
	fr->spawn();
}

//TODO : under construction. Try to implements better allocation strategies (decrease number of Runner by script
/*
	// Alternative to naive runner strategy : add functuin to pred's runner
	int idRunner = boost::get( boost::vertex_name, graph, node_map[pred_uuid]  ) ;
	dynamic_cast<FRunner*>(runners[idRunner])->add_node( node_map[uuid] );
	boost::put(boost::vertex_name, graph, node_map[uuid], idRunner);
*/			

/*
void Kernel::runner_construction()
{
	for( auto it = boost::edges(graph); it.first != it.second; ++it.first)
	{
		Link *l;
		int id_source = boost::get( boost::vertex_name, graph)[ boost::source(*it.first, graph) ];
		int id_target = boost::get( boost::vertex_name, graph)[ boost::target(*it.first, graph) ];

		std::cout << "Runner : " << id_source << " " << id_target << std::endl;

		if( id_source == id_target ) 
		{
			l = new Passing_Link();
		}
		else
		{
			l = new Synchronized_Link();
		}
		
		boost::put(boost::edge_weight, graph, *it.first , l);
	}

	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
        {
        }
}

	for( auto it = boost::edges(graph); it.first != it.second; ++it.first)
	{
		std::cout <<  boost::source(*it.first, graph) << " " <<  boost::target(*it.first, graph) << std::endl;
	}
*/

/********************************************************************************************************/
/****************** 			Bind Section 				      *******************/
/********************************************************************************************************/

void Kernel::bind(IScalar &value, std::string var_name,std::string uuid)
{
	if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a scalar input to unkown function "+uuid );
		
	if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

	if(  xs.functions[uuid].inputs[var_name].isAnchor == true)  throw std::invalid_argument( "Kernel : scalar input "+var_name+" must not be an anchor" ); 

	if(  xs.functions[uuid].inputs[var_name].links.size() != 1 ) throw std::invalid_argument( "Kernel : scalar input "+var_name+" should have only one link. "+ std::to_string(xs.functions[uuid].inputs[var_name].links.size())+" given" );	
	if( xs.functions[uuid].inputs[var_name].links[0].isSparse == true) throw std::invalid_argument( "Kernel : scalar input can't be sparse type"); 
	
	if( xs.functions[uuid].inputs[var_name].links[0].isCst == true )
	{
		value.setCValue( std::stod(xs.functions[uuid].inputs[var_name].links[0].value) );
	}
	else
	{
		std::string uuid_pred = xs.functions[uuid].inputs[var_name].links[0].uuid_pred;

		if( node_map.find( uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to get output from unkown function "+uuid_pred );

		Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid_pred]) ;
		if( f->type() == value.type())
		{
			value.i( &(dynamic_cast<FScalar*>(f)->getOutput()) );

		}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
	}
	value.w( xs.functions[uuid].inputs[var_name].links[0].weight );
	value.setOp( xs.functions[uuid].inputs[var_name].links[0].op );
}


void Kernel::bind(IScalarMatrix& value, std::string var_name,std::string uuid)
{
	if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a scalarMatrix input to unkown function "+uuid );
		
	if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

	if(  xs.functions[uuid].inputs[var_name].isAnchor == true)  throw std::invalid_argument( "Kernel : scalarMatrix input "+var_name+" must not be an anchor" ); 

	if(  xs.functions[uuid].inputs[var_name].links.size() != 1 ) throw std::invalid_argument( "Kernel : scalarMatrix input "+var_name+" should have only one link. "+ std::to_string(xs.functions[uuid].inputs[var_name].links.size())+" given" );	
	if( xs.functions[uuid].inputs[var_name].links[0].isSparse == true) throw std::invalid_argument( "Kernel : scalarMatrix input can't be sparse type"); 
	
	Function *sf =  boost::get(boost::vertex_function, graph, node_map[uuid]) ;

	if( xs.functions[uuid].inputs[var_name].links[0].isCst == true )
	{
		// If value is a constant input, the matrix rows/cols is egal to the rows/cols of the function
		value.setCValue( MatrixXd::Constant( sf->getRows(),sf->getCols(),  std::stod( xs.functions[uuid].inputs[var_name].links[0].value))); 
	}
	else
	{
		std::string uuid_pred = xs.functions[uuid].inputs[var_name].links[0].uuid_pred;

		if( node_map.find( uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to get output from unkown function "+uuid_pred );

		Function *pf =  boost::get(boost::vertex_function, graph, node_map[uuid_pred]) ;
		if( pf->type() == value.type())
		{
			value.i( &(dynamic_cast<FMatrix*>(pf)->getOutput()) );
			// TODO : check size input / output ?

		}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
	}
	value.w( xs.functions[uuid].inputs[var_name].links[0].weight );
	value.setOp( xs.functions[uuid].inputs[var_name].links[0].op );
}


void Kernel::bind(ISAnchor& value, std::string var_name,std::string uuid)
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a scalarAnchor input to unkown function "+uuid );
                
        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

        if(  xs.functions[uuid].inputs[var_name].isAnchor == false)  throw std::invalid_argument( "Kernel : scalarAnchor input "+var_name+" must be an anchor" );

        if(  xs.functions[uuid].inputs[var_name].links.size() == 0 ) throw std::invalid_argument( "Kernel : scalarAnchor input "+var_name+" should have at least one link. "+ std::to_string(xs.functions[uuid].inputs[var_name].links.size())+" given" );        
        if( xs.functions[uuid].inputs[var_name].links[0].isSparse == true) throw std::invalid_argument( "Kernel : scalarAnchor input can't be sparse type");

	for( auto it =  xs.functions[uuid].inputs[var_name].links.begin(); it != xs.functions[uuid].inputs[var_name].links.end(); it++)
	{
		IScalar *is = new IScalar();
		if( it->isCst == true )
		{
			is->setCValue( std::stod( it->value) );
		}
		else
		{
			std::string uuid_pred = it->uuid_pred;
			
			if( node_map.find( uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to get output from unkown function "+uuid_pred );

			Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid_pred]) ;
			if( f->type() == value.type())
			{
				is->i( &(dynamic_cast<FScalar*>(f)->getOutput()) );
		 
			}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
		}
		is->w( it->weight );
		is->setOp( it->op );
		value.add_input(is);
	}
}


void Kernel::bind(ISMAnchor& value, std::string var_name,std::string uuid)
{
        if( node_map.find(uuid) == node_map.end() || xs.functions.find(uuid) == xs.functions.end() ) throw std::invalid_argument( "Kernel : try to bind a smatrixAnchor input to unkown function "+uuid );
                
        if( xs.functions[uuid].inputs.find(var_name) ==  xs.functions[uuid].inputs.end() )  throw std::invalid_argument( "Kernel : unable to find input "+var_name+" for function "+uuid  );

        if(  xs.functions[uuid].inputs[var_name].isAnchor == false)  throw std::invalid_argument( "Kernel : smatrixAnchor input "+var_name+" must be an anchor" );

        if(  xs.functions[uuid].inputs[var_name].links.size() == 0 ) throw std::invalid_argument( "Kernel : smatrixAnchor input "+var_name+" should have at least one link. "+ std::to_string(xs.functions[uuid].inputs[var_name].links.size())+" given" );        
        if( xs.functions[uuid].inputs[var_name].links[0].isSparse == true) throw std::invalid_argument( "Kernel : smatrixAnchor input can't be sparse type");
	
	//TODO : check size (rows/cols). All inputs must have the same size 
	for( auto it =  xs.functions[uuid].inputs[var_name].links.begin(); it != xs.functions[uuid].inputs[var_name].links.end(); it++)
	{
		IScalarMatrix * ism = new IScalarMatrix();
		if( it->isCst == true )
		{
			//is->setCValue( std::stod( it->value) );
			//value.setCValue( MatrixXd::Constant( sf->getRows(),sf->getCols(),  std::stod( xs.functions[uuid].inputs[var_name].links[0].value))); 
		}
		else
		{
			std::string uuid_pred = it->uuid_pred;
			
			if( node_map.find( uuid_pred ) == node_map.end()) throw std::invalid_argument( "Kernel : try to get output from unkown function "+uuid_pred );

			Function *f =  boost::get(boost::vertex_function, graph, node_map[uuid_pred]) ;
			if( f->type() == value.type())
			{
				ism->i( &(dynamic_cast<FMatrix*>(f)->getOutput()) );
		 
			}else throw std::invalid_argument( "Kernel : try to bind no compatible type ");
		}
		ism->w( it->weight );
		ism->setOp( it->op );
		value.add_input(ism);
	}
}


void Kernel::bind(IMMAnchor& value, std::string var_name,std::string uuid)
{

	// Default value : should be possible to add a 1x1 matrix 
}


void Kernel::bind(std::string& value, std::string var_name,std::string uuid)
{
}

