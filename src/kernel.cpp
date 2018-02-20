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

Kernel::~Kernel()
{
	delete xmlc;
	graph.clear();
	runners.clear();
	node_map.clear();
}

void Kernel::init(std::string scriptfile, std::string resfile, std::string libdir)
{

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
	// FAIRE DANS L'AUTRE SENS ? 
	// Comme pour load_graph : lecture du XML et recherche dans le graphe
	// Mais il faut un moyen de chercher dans le graphe (mettre Uuid dans les nodes) avec une MAP Propeties

	// Dans tout les cas : c'est une bonne idée de mettre l'uuid dans les nodes en plus

/*
	std::vector<XInput> inputs; 
	for( auto it_dest = boost::vertices(graph); it_dest.first != it_dest.second; ++it_dest.first)
	{
		//xmlc->getInputsUuid(  (graph[*it_dest.first])->getUuid() , inputs );
		xmlc->getInputsUuid( boost::get(boost::vertex_function, graph)[*it_dest.first]->getUuid(),inputs);
		for( auto it_source = inputs.begin(); it_source != inputs.end(); it_source++)
		{
			add_edge( node_map[(*it_source).uuid_pred], *it_dest.first, graph );	
		}
		inputs.clear();
	}
*/
	std::vector<XInput> inputs;
	xmlc->getInputs(inputs);
	
	for( auto it = inputs.begin(); it != inputs.end(); it++)
	{
		if( node_map.find( (*it).uuid_pred ) == node_map.end()) throw  std::invalid_argument( "Kernel : try to add link with unkown source "+(*it).uuid_pred  );
		if( node_map.find( (*it).uuid_suc ) == node_map.end()) throw   std::invalid_argument( "Kernel : try to add link with unkown target "+(*it).uuid_suc );

		add_edge( node_map[(*it).uuid_pred],  node_map[(*it).uuid_suc]  , graph );
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
	for( auto it = runners.begin(); it != runners.end(); it++)
	{
		it->second->spawn();
	}
}

void Kernel::join()
{
	for( auto it = runners.begin(); it != runners.end(); it++)
	{
		it->second->join();
	}
}

void Kernel::add_rttoken()
{
	Graph::vertex_descriptor rt_node = boost::add_vertex(graph);
	node_map[rtnode_name] = rt_node  ;

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

	RtToken * RT = new RtToken( xrt.value , xrt.unit );
	RT->setGraph(&graph);
	RT->setRtNode(rt_node);
	boost::put(boost::vertex_name, graph, rt_node, 0);
	runners[0] = RT;
}


void Kernel::simple_runner_allocation()
{
	FRunner *fr;
	int idRunner=1;

	for( auto it =  boost::vertices(graph) ; it.first != it.second; ++it.first)
	{
		Graph::vertex_descriptor v =  *it.first ;

		if( boost::get( boost::vertex_name , graph, v) == -1) 
		{
			fr = new FRunner(idRunner);
			fr->setGraph(&graph);
			fr->add_node(v);

			runners[idRunner] = dynamic_cast<Runner*>(fr);

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
