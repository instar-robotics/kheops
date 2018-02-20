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

#include <stdexcept>
#include "xmlconverter.h"

XmlConverter::XmlConverter(std::string filepath) : m_doc(NULL), parser(NULL),errHandler(NULL)
{
	parser = new XercesDOMParser();
	errHandler = (ErrorHandler*) new HandlerBase();
	parser->setErrorHandler(errHandler);
	parser->parse(filepath.c_str());
	m_doc = parser->getDocument();
}



void XmlConverter::__convertXmlToFunction(const DOMElement &el, XFunction &f)
{
	f.uuid = XMLString::transcode( el.getAttribute( XMLString::transcode(  "uuid" )));

	if( f.uuid.size() == 0 ) throw std::invalid_argument("XML : Function uuid is empty");

	if( el.getElementsByTagName(XMLString::transcode("name"))->getLength() == 0) throw std::invalid_argument("Kernel : Function "+f.uuid+" has no name");

	f.name =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("name"))->item(0))->getTextContent()));

	std::string x = XMLString::transcode( el.getAttribute( XMLString::transcode(  "x" )));
	if( x.size() == 0 ) throw std::invalid_argument("XML : Function have no X value");
	f.x = std::stoi(x);

	std::string y  = XMLString::transcode( el.getAttribute( XMLString::transcode(  "y" )));
	if( y.size() == 0 ) throw std::invalid_argument("XML : Function have no Y value");
	f.y = std::stoi(y);
}

void XmlConverter::__convertXmlToInput( const DOMElement &el, XInput &xi )
{
	if( el.getElementsByTagName(XMLString::transcode("name"))->getLength() == 0) throw std::invalid_argument("Kernel : Input has no name (Function "+xi.uuid_suc+")");

	xi.name =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("name"))->item(0))->getTextContent()));

	if( el.getElementsByTagName(XMLString::transcode("link"))->getLength() == 0) throw std::invalid_argument("Kernel : Input has no predecessor [link tag] (Function "+xi.uuid_suc+")");

	xi.uuid_pred =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("link"))->item(0))->getTextContent()));

}


void XmlConverter::getFunctions(std::vector<XFunction> &functions)
{
	DOMElement * xFunc;
	XFunction f;

	if( m_doc->getElementsByTagName(XMLString::transcode("functions"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find functions section");

	xFunc = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild()  ;
	if( xFunc == NULL) throw  std::invalid_argument("XML : functions section is empty, any function to load");

	do{
		__convertXmlToFunction( *xFunc, f);
		functions.push_back(f);

	}while(  (xFunc = xFunc->getNextElementSibling() ) != NULL);
}

void XmlConverter::getInputsUuid( std::string FunctUuid, std::vector<XInput> &xinputs  )
{
	if( m_doc->getElementsByTagName(XMLString::transcode("functions"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find functions section");

	DOMElement * function = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild();
	if (function == NULL)  throw  std::invalid_argument("XML : functions section is empty, any input to load ");

	do{
		if( XMLString::transcode(function->getAttribute( XMLString::transcode("uuid"))) == FunctUuid )
		{
			if(  function->getElementsByTagName(XMLString::transcode("inputs"))->getLength() == 0) return;
			DOMElement * input = dynamic_cast<DOMElement *>( function->getElementsByTagName(XMLString::transcode("inputs"))->item(0))->getFirstElementChild() ;
			if( input == NULL) return ;

			do{
				XInput xi;
				xi.uuid_suc = FunctUuid;
				__convertXmlToInput( *input, xi );
				xinputs.push_back(xi);

			}while( (input = input->getNextElementSibling()) != NULL  );

			return;
		}
	}while( (function = function->getNextElementSibling()) != NULL );
}


void XmlConverter::getInputs( std::vector<XInput> &xinputs)
{
	if( m_doc->getElementsByTagName(XMLString::transcode("functions"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find functions section");

	DOMElement * function = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild();

	if (function == NULL)  throw  std::invalid_argument("XML : functions section is empty, any input to load ");
	do{
		if(  function->getElementsByTagName(XMLString::transcode("inputs"))->getLength() > 0)
		{
			std::string FunctUuid = XMLString::transcode( function->getAttribute( XMLString::transcode(  "uuid" )));
			if( FunctUuid.size() == 0 ) throw std::invalid_argument("XML : Function uuid is empty");

			DOMElement * input = dynamic_cast<DOMElement *>( function->getElementsByTagName(XMLString::transcode("inputs"))->item(0))->getFirstElementChild() ;
			if( input != NULL)
			{
				do{
					XInput xi;
					xi.uuid_suc = FunctUuid;
					__convertXmlToInput( *input, xi );
					xinputs.push_back(xi);

				}while( (input = input->getNextElementSibling()) != NULL  );
			}
		}
	}while( (function = function->getNextElementSibling()) != NULL );
}

void XmlConverter::getScriptName(std::string &name)
{
	if(  m_doc->getElementsByTagName(XMLString::transcode("script"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find script tag ");

	DOMElement * el = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("script"))->item(0))->getFirstElementChild();
	if (el  == NULL) throw std::invalid_argument("XML : missing tag");

	do{
		std::string tname = XMLString::transcode( el->getTagName());
		if( !tname.compare( "name")) { break; }

	}while( (el = el->getNextElementSibling()) != NULL );

	if( el == NULL) throw std::invalid_argument("XML : enable to find script name");

	name=XMLString::transcode( el->getTextContent());
}

void XmlConverter::getRtToken( XRtToken & rt )
{
	if(  m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find \"rt_token\" tag");

	rt.value = std::stod(   XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getTextContent()));

	rt.unit = XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getAttribute( XMLString::transcode(  "unit" )));
}
