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
#include <iostream>
#include "function.h"
#include "factory.h"

using namespace xercesc;

class XmlConverter
{
	private :

	DOMDocument* m_doc;
	XercesDOMParser *parser ;
	ErrorHandler* errHandler;

	DOMElement * xFunc;
	
	Function *  __convertXmlToFunction(const DOMElement &el)
	{
                std::string name =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("name"))->item(0))->getTextContent()));
		std::string uuid = XMLString::transcode( el.getAttribute( XMLString::transcode(  "uuid" )));

		Function *f = Factory<Function>::Instance().create(name);
		if( f ==  NULL ) throw  std::invalid_argument("Kernel : Unable to find Function");
		else f->setUuid(uuid);

		return f;
	}

	public : 
	
	~XmlConverter()
	{
		delete parser;
		delete errHandler;
	}

	XmlConverter(std::string filepath) : m_doc(NULL), parser(NULL),errHandler(NULL), xFunc(NULL)
	{
		parser = new XercesDOMParser();
		errHandler = (ErrorHandler*) new HandlerBase();
		parser->setErrorHandler(errHandler);
		parser->parse(filepath.c_str());
		m_doc = parser->getDocument();
	}

	XmlConverter() : m_doc(NULL),parser(NULL),errHandler(NULL), xFunc(NULL) { }

	static void Initialize()
	{
		XMLPlatformUtils::Initialize();
	}

	std::string getScriptName()
	{
		return  XMLString::transcode( (dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("script"))->item(0))->getFirstElementChild())->getTextContent())  ;
	}

	Function * getFirstFunction()
	{
		xFunc = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild()  ;
		if( xFunc == NULL) return NULL;

		return __convertXmlToFunction( (*xFunc) );
	}

	Function * getNextFunction()
	{
		if( xFunc == NULL) return NULL;

		xFunc = xFunc->getNextElementSibling();
	
		if( xFunc == NULL) return NULL;
		else return __convertXmlToFunction( (*xFunc) );
	}

	void getInputsUuid( std::string FunctUuid, std::vector<std::string> &links_name  )
	{
		DOMElement * function = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild();
		if (function == NULL) return;

		do{
			if( XMLString::transcode(function->getAttribute( XMLString::transcode("uuid"))) == FunctUuid )
			{
				// SPLIT Finds du Tag Inputs
				DOMElement * inputs = dynamic_cast<DOMElement *>( function->getElementsByTagName(XMLString::transcode("inputs"))->item(0))->getFirstElementChild() ; 
				if( inputs == NULL) return ;
				do{
					links_name.push_back( XMLString::transcode(inputs->getElementsByTagName(XMLString::transcode("link"))->item(0)->getTextContent()));

				}while( (inputs = inputs->getNextElementSibling()) != NULL  );

			}
		}while( (function = function->getNextElementSibling()) != NULL );	
	}


	double get_RtToken()
	{
		return std::stod(   XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getTextContent()  ) );
	}

	std::string get_RtToken_Unit()
	{

		return XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getAttribute( XMLString::transcode(  "unit" )));


	}
};

#endif // __XML_CONVERTER__
