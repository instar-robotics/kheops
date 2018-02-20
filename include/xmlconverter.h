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

#ifndef __XML_CONVERTER__
#define __XML_CONVERTER__

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <string>
#include <vector>

using namespace xercesc;

struct XFunction
{
	std::string uuid;
	std::string name; 
	int x;
	int y;
};

struct XInput
{
	std::string name;
	std::string uuid_pred;
	std::string uuid_suc;
};

struct XRtToken
{
	std::string unit;
	double value;
};

class XmlConverter
{
	private :

	DOMDocument* m_doc;
	XercesDOMParser *parser ;
	ErrorHandler* errHandler;
	
	void __convertXmlToFunction(const DOMElement &el, XFunction &f);
	void __convertXmlToInput( const DOMElement &el, XInput &xi );

	public : 
	
	~XmlConverter()
	{
		delete parser;
		delete errHandler;
	}

	XmlConverter(std::string filepath);
	XmlConverter() : m_doc(NULL),parser(NULL),errHandler(NULL) {}

	static inline void Initialize()	{ XMLPlatformUtils::Initialize();}

	void getFunctions(std::vector<XFunction> &functions);
	void getInputsUuid( std::string FunctUuid, std::vector<XInput> &xinputs );
	void getInputs( std::vector<XInput> &xinputs);

	void getScriptName(std::string &name);
	void getRtToken( XRtToken & rt );
};

#endif // __XML_CONVERTER__
