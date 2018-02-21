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
#include <uuid/uuid.h>
#include <iostream>

#include "kernel.h"
#include "factory.h"
#include "util.h"
#include "frunner.h"
#include "rttoken.h"

Kernel Kernel::singleton;

Kernel::~Kernel()
{
	delete xmlc;
	graph.clear();
	node_map.clear();
	
	for( auto it = runners.begin(); it != runners.end(); it++)
	{
		delete(it->second);
	}

	runners.clear();
}

void Kernel::init(std::string scriptfile, std::string resfile, std::string libdir)
{
	singleton.scriptfile=scriptfile;
	singleton.resfile=resfile;
	singleton.libdir = libdir;

	singleton.xmlc = new  XmlConverter(scriptfile);

	singleton.xmlc->getScriptName( singleton.script_name  );

	std::cout << "Run : " << singleton.script_name << " script"<< std::endl;
}

void Kernel::load_functions()
{
	std::vector<XFunction> functions;
	xmlc->getFunctions(functions);

	for(auto it = functions.begin(); it != functions.end(); it++)
	{
		if( Factory<Function>::Instance().is_register(it->name) )
		{
			add_function( buildFunction(*it ) );	
		}	
		else
		{
			std::cout << "Function "+it->name+" is not a native function. Try to load it" << std::endl;
		}
	}
}

void Kernel::load_inputs()
{
	std::vector<XInput> inputs;
	xmlc->getInputs(inputs);
	
	for( auto it = inputs.begin(); it != inputs.end(); it++)
	{
		if( node_map.find( (*it).uuid_pred ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add link with unkown source "+(*it).uuid_pred  );
		if( node_map.find( (*it).uuid_suc ) == node_map.end()) throw   std::invalid_argument( "Kernel : try to add link with unkown target "+(*it).uuid_suc );

		add_edge( node_map[(*it).uuid_pred], node_map[(*it).uuid_suc]  , graph );
	} 	
	inputs.clear();
	
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

Function* Kernel::buildFunction(const XFunction& xf)
{
      Function *f = Factory<Function>::Instance().create(xf.name);
      if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+xf.name);
      else{ 
		f->setUuid(xf.uuid);
		f->setSize(xf.x,xf.y,0);
      }
      return f;
}

void Kernel::add_function( Function *funct  )
{
	if( funct != NULL) {
		if( node_map.find( funct->getUuid() ) == node_map.end() ) 
		{
			Graph::vertex_descriptor vd = boost::add_vertex(graph);
			boost::put(boost::vertex_function, graph, vd, funct );
			boost::put(boost::vertex_name, graph, vd, -1);
			node_map[funct->getUuid()] = vd;
		}
		else throw  std::invalid_argument("Kernel : uuid function in xml file has to be unique");
	}
	else throw  std::invalid_argument("Kernel : try to add Null Function");
}

void Kernel::add_function_on_fly(std::string Fct, std::string pred_uuid, int x, int y)
{
	uuid_t out;
	uuid_generate(out);

	char cuuid[37];
	uuid_unparse(out,cuuid);
	std::string uuid(cuuid);
	
	if( node_map.find( pred_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ pred_uuid );

	if( Factory<Function>::Instance().is_register(Fct) )
	{
		Function *f = Factory<Function>::Instance().create(Fct);
      		if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+Fct);
     		else{
                	f->setUuid(uuid);

			if( x == -1 ){ x=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getX();}
			if( y == -1 ){ y=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getY();}

                	f->setSize(x,y,0);

			add_function(f);
			EdgeWeightProperty e =dynamic_cast<Link*>(new Synchronized_Link());
			add_edge( node_map[pred_uuid] , node_map[uuid] ,e, graph  );
			
			EdgeWeightProperty rt_e =dynamic_cast<Link*>(new Synchronized_Link());
			add_edge( node_map[uuid], RtToken::instance().getRtNode() ,rt_e, graph  );

			int idRunner = add_frunner();
			dynamic_cast<FRunner*>(runners[idRunner])->add_node( node_map[uuid] );
                        boost::put(boost::vertex_name, graph, node_map[uuid], idRunner);

			runners[idRunner]->spawn();
      		}
	}
}


void Kernel::insert_function_on_fly(std::string Fct, std::string pred_uuid, std::string suc_uuid ,int x, int y)
{
        uuid_t out;
        uuid_generate(out);

        char cuuid[37];
        uuid_unparse(out,cuuid);
        std::string uuid(cuuid);

        if( node_map.find( pred_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ pred_uuid ); 
        if( node_map.find( suc_uuid  ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add function with unkown source "+ suc_uuid ); 

        if( Factory<Function>::Instance().is_register(Fct) )
        {
                Function *f = Factory<Function>::Instance().create(Fct);
                if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to build Function "+Fct);
                else{
                        f->setUuid(uuid);
        
                        if( x == -1 ){ x=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getX();}
                        if( y == -1 ){ y=boost::get(boost::vertex_function, graph)[node_map[pred_uuid]]->getY();}

                        f->setSize(x,y,0);
  
                        add_function(f);

                        EdgeWeightProperty ep =dynamic_cast<Link*>(new Synchronized_Link());
                        add_edge( node_map[pred_uuid] , node_map[uuid] ,ep, graph  );
                        
                        EdgeWeightProperty es =dynamic_cast<Link*>(new Synchronized_Link());
                        add_edge( node_map[uuid],  node_map[suc_uuid]  ,es, graph  );
/*
                        int idRunner = boost::get( boost::vertex_name, graph, node_map[pred_uuid]  ) ;
                        dynamic_cast<FRunner*>(runners[idRunner])->add_node( node_map[uuid] );
                        boost::put(boost::vertex_name, graph, node_map[uuid], idRunner);
*/			
			int idRunner = add_frunner();
			dynamic_cast<FRunner*>(runners[idRunner])->add_node( node_map[uuid] );
                        boost::put(boost::vertex_name, graph, node_map[uuid], idRunner);

			runners[idRunner]->spawn();
                }
        }
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


void Kernel::del_function( Function * funct)
{
	clear_vertex( node_map[funct->getUuid()] , graph);
	remove_vertex( node_map[funct->getUuid()] , graph);
}

void Kernel::del_function(const std::string& uuid)
{
	clear_vertex( node_map[uuid] , graph);
	remove_vertex( node_map[uuid] , graph);
}

void Kernel::spawn()
{
	RtToken::instance().spawn();
	for( auto it = runners.begin(); it != runners.end(); it++)
	{
		it->second->spawn();
	}
}

void Kernel::join()
{
	RtToken::instance().join();
	for( auto it = runners.begin(); it != runners.end(); it++)
	{
		it->second->join();
	}
}

int Kernel::add_frunner()
{
	int idr = runners.size();
	FRunner *fr = new FRunner(idr);
	fr->setGraph(&graph);
	runners[idr] = dynamic_cast<Runner*>(fr);

	return idr;
}

void Kernel::add_rttoken()
{
	Graph::vertex_descriptor rt_node = boost::add_vertex(graph);

	Link *l;
	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
        {
		if( *it.first != rt_node)
		{
			auto it_in = boost::in_edges( *it.first  , graph);
			if( it_in.first == it_in.second )
			{
				add_edge( rt_node, *it.first, graph  );
			}

			auto it_out = boost::out_edges(*it.first, graph);
			if( it_out.first == it_out.second)
			{
				add_edge(*it.first, rt_node  ,graph  );
			}
		}
        }

	XRtToken xrt;
	xmlc->getRtToken(xrt); 

	RtToken::instance().setToken(xrt.value , xrt.unit );
	RtToken::instance().setGraph(&graph);
	RtToken::instance().setRtNode(rt_node);
	boost::put(boost::vertex_name, graph, rt_node, 0);
}


void Kernel::simple_runner_allocation()
{
	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
	{
		Graph::vertex_descriptor v =  *it.first ;

		if( boost::get( boost::vertex_name , graph, v) == -1) 
		{
			int idRunner = add_frunner();
			dynamic_cast<FRunner*>(runners[idRunner])->add_node(v);

			boost::put(boost::vertex_name, graph, v, idRunner);
			idRunner++;
		}
	}

	for( auto it = boost::edges(graph); it.first != it.second; ++it.first)
	{
		boost::put(boost::edge_weight, graph, *it.first , new Synchronized_Link());
	}
}

void Kernel::runner_allocation()
{
	this->simple_runner_allocation();
}

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
