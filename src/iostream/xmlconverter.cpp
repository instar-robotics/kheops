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


#include <algorithm>
#include <stdexcept>
#include "kheops/iostream/xmlconverter.h"

XmlConverter::XmlConverter(std::string filepath)
{
	read_xml(filepath, tree);
}

void XmlConverter::__convertXmlToConstant(const ptree &tree, XConstant &c)
{
	c.uuid = tree.get<std::string>("<xmlattr>.uuid");
	c.name = tree.get<std::string>("name");
        
	c.rows = 0;
        c.cols = 0;

        c.type = tree.get<std::string>("output.<xmlattr>.type");
        if( c.type == "MATRIX")
        {
                c.rows = tree.get<unsigned int>("output.rows");
                c.cols = tree.get<unsigned int>("output.cols");
        }else if( c.type != "SCALAR" && c.type != "STRING" ) throw std::invalid_argument("XML : \"constant type\" must be \"SCALAR\" , \"MATRIX\" or \"STRING\" ");
}

void XmlConverter::__convertXmlToFunction(const ptree &tree, XFunction &f)
{
	f.uuid = tree.get<std::string>("<xmlattr>.uuid");
	f.name = tree.get<std::string>("name");
	f.libname = tree.get<std::string>("libname");

	f.save = false;
        try{
                std::string save  = tree.get<std::string>("save");
                if( save == "true" ) f.save = true;
                else if( save != "false" ) throw std::invalid_argument("XML : \"save\" attribute is boolean, value must be \"true\" or \"false\" ");
        }
        catch(std::invalid_argument e){throw;}
        catch(...){}

        f.publish = false;
        try{
                std::string publish  = tree.get<std::string>("publish");
                if( publish == "true" ) f.publish = true;
                else if( publish != "false" ) throw std::invalid_argument("XML : \"publish\" attribute is boolean, value must be \"true\" or \"false\" ");
        }
        catch(std::invalid_argument e){throw;}
        catch(...){}

        if( f.publish )
        {
                f.topic_name = tree.get<std::string>("publish.<xmlattr>.topic");
        }

	f.commented = false;
	try{
		std::string com = tree.get<std::string>("<xmlattr>.commented");
                if( com == "true" ) f.commented = true;
                else if( com != "false" ) throw std::invalid_argument("XML : \"commented\" attribute is boolean, value must be \"true\" or \"false\" ");
	}
        catch(std::invalid_argument e){throw;}
        catch(...){}

        f.rows = 0;
        f.cols = 0;

        f.type = tree.get<std::string>("output.<xmlattr>.type");
        if( f.type == "MATRIX")
        {
                f.rows = tree.get<unsigned int>("output.rows");
                f.cols = tree.get<unsigned int>("output.cols");
        }else if( f.type != "SCALAR" ) throw std::invalid_argument("XML : \"output type\" must be \"SCALAR\" or \"MATRIX\" ");
}


void XmlConverter::__convertXmlToInput( const ptree &tree, XInput &xi )
{
	xi.uuid = tree.get<std::string>("<xmlattr>.uuid");
        xi.name = tree.get<std::string>("name");
}

void XmlConverter::__convertXmlToLink( const ptree &tree, XLink &xl )
{
	xl.uuid = tree.get<std::string>("<xmlattr>.uuid");
	xl.uuid_pred = tree.get<std::string>("from");
	
	std::string secondary  = tree.get<std::string>("<xmlattr>.secondary");
	if( secondary == "true" ) xl.isSecondary = true;
	else if( secondary == "false" ) xl.isSecondary = false; 
	else throw std::invalid_argument("XML : \"secondary\" attribute is boolean, value must be \"true\" or \"false\" ");
	
	xl.weight = 0;
	try{
		xl.weight = tree.get<double>("weight");
	}
	catch(...){}
	
	xl.value = "";
	try{
		xl.value = tree.get<std::string>("value");
	}
	catch(...){}

	try{
		xl.con.type = tree.get<std::string>("connectivity.<xmlattr>.type");

		for( const ptree::value_type &xExpr : tree.get_child("connectivity."))
                {
			if( xExpr.first == "expression" )
			{
				xl.con.nei_expr.push_back(xExpr.second.data());
			}
		}
	}
	catch(...){}
}

void XmlConverter::loadScript(XScript &xs)
{
 	xs.name = tree.get<std::string>("script.name");
	__loadRtToken( xs.rt );
	__loadConstants( xs.constants );
	__loadFunctions( xs.functions );
}

void XmlConverter::__loadRtToken( XRtToken & rt )
{
	rt.value = tree.get<double>("script.rt_token"); 
	rt.unit =  tree.get<std::string>("script.rt_token.<xmlattr>.unit");
	rt.uuid = tree.get<std::string>("script.rt_token.<xmlattr>.uuid");
}

void XmlConverter::__loadFunctions(std::map<std::string,XFunction> &functions)
{
	for( const ptree::value_type &xFunc : tree.get_child("script.functions"))
	{
		if( xFunc.first == "function" )
		{
			XFunction f;

			__convertXmlToFunction(  xFunc.second, f);
			__loadInputs( xFunc.second, f.inputs );

			if( functions.find( f.uuid ) == functions.end() ) functions[f.uuid] = f;
                        else throw std::invalid_argument("XML : uuid function in xml file has to be unique");
		}
	}
}

void XmlConverter::__loadConstants(std::map<std::string,XConstant> &constants)
{
	for( const ptree::value_type &xConst : tree.get_child("script.functions"))
	{
		if( xConst.first == "constant" )
		{
			XConstant c;

			__convertXmlToConstant(  xConst.second, c);

			if( constants.find( c.uuid ) == constants.end() ) constants[c.uuid] = c;
                        else throw std::invalid_argument("XML : uuid constant in xml file has to be unique");
		}
	}
}

void XmlConverter::__loadInputs(const ptree &tree, std::map<std::string, XInput> &inputs )
{
	for( const ptree::value_type &xInput : tree.get_child("inputs"))
	{
		if( xInput.first == "input" )
		{
			XInput i;

			__convertXmlToInput( xInput.second, i);
			__loadLinks( xInput.second, i.links );
			
			inputs[i.name] = i;
		}
	}
}

void XmlConverter::__loadLinks(const ptree &tree, std::vector<XLink> &links  )
{
	for( const ptree::value_type &xLink : tree.get_child("links"))
	{
		if( xLink.first == "link" )
		{
			XLink l;
			__convertXmlToLink(xLink.second ,l );
			links.push_back(l);	
		}	
	}
}

