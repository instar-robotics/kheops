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

#include <algorithm>
#include <iostream>
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
                else if( save == "false" ) f.save = false;
                else throw std::invalid_argument("XML : \"save\" attribute is boolean, value must be \"true\" or \"false\" ");
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
		//TODO : READ Complexe connectivity
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

