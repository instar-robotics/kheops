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

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "function.h"
#include "klink.h"
#include <string>

namespace boost {
    enum vertex_function_t { vertex_function};
    BOOST_INSTALL_PROPERTY(vertex, function);
}

namespace boost {
    enum vertex_runner_t { vertex_runner};
    BOOST_INSTALL_PROPERTY(vertex, runner);
}
typedef boost::property <boost::vertex_runner_t, int,  boost::property<boost::vertex_function_t, Function*  >> VertexProperties;


namespace boost {
    enum edge_type_t { edge_type};
    BOOST_INSTALL_PROPERTY(edge, type);
}

namespace boost {
    enum edge_klink_t { edge_klink};
    BOOST_INSTALL_PROPERTY(edge, klink);
}

namespace boost {
    enum edge_uuid_t { edge_uuid};
    BOOST_INSTALL_PROPERTY(edge, uuid);
}

typedef boost::property<boost::edge_klink_t, kLink*, boost::property< boost::edge_type_t, bool, boost::property< boost::edge_uuid_t, std::string>>> EdgeWeightProperty;

typedef boost::adjacency_list<boost::vecS, boost::listS, boost::bidirectionalS , VertexProperties , EdgeWeightProperty>  Graph;


typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<Graph>::in_edge_iterator in_edge_iterator;

void write_graph(const Graph& graph, std::string name);

#endif //__GRAPH_H__
