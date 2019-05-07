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


#ifndef __XML_CONVERTER__
#define __XML_CONVERTER__

#include "kheops/kernel/type.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <vector>
#include <map>

using namespace boost::property_tree;

struct XConnectivity
{
	std::string type;
	std::vector<std::string> nei_expr;
};

struct XLink
{
	std::string uuid;
	std::string uuid_pred;

	bool isSecondary;
	
	SCALAR weight;
	std::string value;

	XConnectivity con;
};

struct XInput
{
	std::string uuid;
	std::string name;

	std::vector<XLink> links;
};

struct XFunction
{
	std::string uuid;
	std::string name; 
	std::string libname; 

	std::string type;
	unsigned int rows;
	unsigned int cols;

	bool commented;
	bool publish;
	bool save;
	std::string topic_name;

	// string : input name
	std::map<std::string,XInput> inputs;
};

struct XConstant
{
	std::string uuid;
	std::string name; 
	
	std::string type;
	unsigned int rows;
	unsigned int cols;
};

struct XRtToken
{
	std::string uuid;
	std::string unit;
	double value;
};

struct XScript
{
	std::string name;
	XRtToken rt;	

	// string : function uuid
	std::map<std::string, XFunction> functions;
	std::map<std::string, XConstant> constants;

	bool isLinkCst(const XLink& l) { return (constants.find( l.uuid_pred) != constants.end() );}
};

class XmlConverter
{
	private :

	ptree tree;

	void __convertXmlToFunction(const ptree &tree, XFunction &f);
	void __convertXmlToConstant(const ptree &tree, XConstant &c);
	void __convertXmlToInput( const ptree &tree, XInput &xi );
	void __convertXmlToLink( const ptree &tree, XLink &xi );
	
	void __loadInputs( const ptree &tree, std::map<std::string, XInput> &inputs  );
	void __loadLinks( const ptree &tree, std::vector<XLink> &inputs  );

	void __loadFunctions(std::map<std::string,XFunction> &functions);
	void __loadConstants(std::map<std::string,XConstant> &constants);
	void __loadRtToken(XRtToken & rt);

	public : 
	
	XmlConverter(std::string filepath);
	XmlConverter(){}
	~XmlConverter(){}

	void loadScript(XScript& xs);
};

#endif // __XML_CONVERTER__
