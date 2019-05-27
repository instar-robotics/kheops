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


#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <vector>
#include <string>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include "kheops/kernel/function.h"
#include "kheops/kernel/klink.h"

class Runner;

/* definition of basic boost::graph properties */
enum vertex_properties_t { vertex_properties };
enum edge_properties_t { edge_properties };
namespace boost {
    BOOST_INSTALL_PROPERTY(vertex, properties);
    BOOST_INSTALL_PROPERTY(edge, properties);
}

namespace boost {
    enum vertex_function_t { vertex_function};
    BOOST_INSTALL_PROPERTY(vertex, function);
}

namespace boost {
    enum vertex_runner_t { vertex_runner};
    BOOST_INSTALL_PROPERTY(vertex, runner);
}

typedef boost::property <boost::vertex_runner_t, Runner* ,  boost::property<boost::vertex_function_t, Function*>> VertexProperties;

namespace boost {
    enum edge_klink_t { edge_klink};
    BOOST_INSTALL_PROPERTY(edge, klink);
}

namespace boost {
    enum edge_uuid_t { edge_uuid};
    BOOST_INSTALL_PROPERTY(edge, uuid);
}

typedef boost::property<boost::edge_klink_t, kLink*, boost::property< boost::edge_uuid_t, std::string>> EdgeWeightProperty;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS , VertexProperties , EdgeWeightProperty>  Graph;

typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS > Graph_debug;


typedef boost::graph_traits<Graph>::edge_iterator edge_iter;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<Graph>::in_edge_iterator in_edge_iterator;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Graph>::vertices_size_type vertices_size;

typedef boost::iterator_property_map<std::vector<vertices_size>::iterator, boost::property_map<Graph, boost::vertex_index_t>::const_type>  property_map_type;

struct do_nothing
{
    template <typename VertexOrEdge1, typename VertexOrEdge2>
    void operator()(const VertexOrEdge1& , VertexOrEdge2& ) const 
    {
    }
};

#endif //__GRAPH_H__
