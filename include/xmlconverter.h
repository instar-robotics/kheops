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

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <string>
#include <vector>
#include <map>

using namespace xercesc;

struct XLink
{
	std::string uuid;

	std::string uuid_pred;
	std::string op;
	double weight;

	bool isSparse;
	bool isSecondary;
	bool isCst;
	
	std::string value;
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

	unsigned int rows;
	unsigned int cols;

	// string : input name
	std::map<std::string,XInput> inputs;
};

struct XRtToken
{
	std::string unit;
	double value;
};

struct XScript
{
	std::string name;
	XRtToken rt;	

	// string : function uuid
	std::map<std::string, XFunction> functions;
};

class XmlConverter
{
	private :

	DOMDocument* m_doc;
	XercesDOMParser *parser ;
	ErrorHandler* errHandler;
	
	void __convertXmlToFunction(const DOMElement &el, XFunction &f);
	void __convertXmlToInput( const DOMElement &el, XInput &xi );
	void __convertXmlToLink( const DOMElement &el, XLink &xi );
	
	void __loadInputs( const DOMElement &el, std::map<std::string, XInput> &inputs  );
	void __loadLinks( const DOMElement &el, std::vector<XLink> &inputs  );
	void __loadFunctions(std::map<std::string,XFunction> &functions);
	
	void __loadScriptName(std::string &name);
	void __loadRtToken(XRtToken & rt);

	public : 
	
	~XmlConverter()
	{
		delete parser;
		delete errHandler;
	}

	XmlConverter(std::string filepath);
	XmlConverter() : m_doc(NULL),parser(NULL),errHandler(NULL) {}

	static inline void Initialize()	{ XMLPlatformUtils::Initialize();}

	void loadScript(XScript& xs);
};

#endif // __XML_CONVERTER__
