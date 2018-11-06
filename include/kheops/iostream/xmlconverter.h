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

#ifndef __XML_CONVERTER__
#define __XML_CONVERTER__

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <vector>
#include <map>

using namespace boost::property_tree;

struct XConnectivity
{
	std::string type;
};

struct XLink
{
	std::string uuid;
	std::string uuid_pred;

	bool isSecondary;
	
	double weight;
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
