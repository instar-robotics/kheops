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
	f.uuid = XMLString::transcode( el.getAttribute( XMLString::transcode( "uuid" )));

	if( f.uuid.size() == 0 ) throw std::invalid_argument("XML : Function uuid is empty");

	if( el.getElementsByTagName(XMLString::transcode("name"))->getLength() == 0) throw std::invalid_argument("XML : Function "+f.uuid+" has no name");

	f.name =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("name"))->item(0))->getTextContent()));
	
	if( el.getElementsByTagName(XMLString::transcode("save"))->getLength() == 0)
	{
		f.save = false;
	}
	else
	{
		std::string save = XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("save"))->item(0))->getTextContent()));

		if( save == "true" ) f.save = true;
		else if( save == "false" ) f.save = false;
		else throw std::invalid_argument("XML : \"save\" attribute is boolean, value must be \"true\" or \"false\" ");
	}
	
	if( el.getElementsByTagName(XMLString::transcode("publish"))->getLength() == 0)
	{
		f.publish = false;
	}
	else
	{
		std::string publish = XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("publish"))->item(0))->getTextContent()));

		if( publish == "true" ) f.publish = true;
		else if( publish == "false" ) f.publish = false;
		else throw std::invalid_argument("XML : \"publish\" attribute is boolean, value must be \"true\" or \"false\" ");

		if( f.publish )
		{
			DOMElement * ep = (dynamic_cast<DOMElement *> ( el.getElementsByTagName(XMLString::transcode("publish"))->item(0)));
			f.topic_name = XMLString::transcode( ep->getAttribute( XMLString::transcode( "topic" )));
			std::replace( f.topic_name.begin(), f.topic_name.end(), ' ', '_');
			std::replace( f.topic_name.begin(), f.topic_name.end(), '-', '_');
		}
	}

	//TODO : use output section as mandatory member ?
	if( el.getElementsByTagName(XMLString::transcode("output"))->getLength() == 0)
	{
		f.rows = 0;
		f.cols = 0;
	}
	else
	{
		DOMElement * eo = (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("output"))->item(0)));

		if( eo->getElementsByTagName( XMLString::transcode("rows") )->getLength() == 0) throw std::invalid_argument("XML : Function "+f.uuid+" has no rows size");
		if( eo->getElementsByTagName( XMLString::transcode("cols") )->getLength() == 0) throw std::invalid_argument("XML : Function "+f.uuid+" has no cols size");


		std::string rows = XMLString::transcode( eo->getElementsByTagName( XMLString::transcode("rows"))->item(0)->getTextContent() );
		f.rows = std::stoi(rows);

		std::string cols =  XMLString::transcode( eo->getElementsByTagName( XMLString::transcode("cols"))->item(0)->getTextContent() );
		f.cols = std::stoi(cols);
	}
}


void XmlConverter::__convertXmlToInput( const DOMElement &el, XInput &xi )
{
	xi.uuid = XMLString::transcode( el.getAttribute( XMLString::transcode( "uuid" )));

	if( xi.uuid.size() == 0 ) throw std::invalid_argument("XML : Input uuid is empty");

	if( el.getElementsByTagName(XMLString::transcode("name"))->getLength() == 0) throw std::invalid_argument("XML : Input has no name ");

	xi.name =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("name"))->item(0))->getTextContent()));

	
}

void XmlConverter::__convertXmlToLink( const DOMElement &el, XLink &xi )
{
	xi.uuid = XMLString::transcode( el.getAttribute( XMLString::transcode(  "uuid" )));

	if( xi.uuid.size() == 0 ) throw std::invalid_argument("XML : Link uuid is empty");

	std::string constant =  XMLString::transcode( el.getAttribute(  XMLString::transcode( "constant" )));
	if( constant.size() == 0)
	{
		xi.isCst = false;
	}
	else
	{
		if( constant == "true" ) xi.isCst = true;
		else if( constant == "false" ) xi.isCst = false;
		else throw std::invalid_argument("XML : \"constant\" attribute is boolean, value must be \"true\" or \"false\" ");
	}

	if( xi.isCst) 
	{
		if( el.getElementsByTagName(XMLString::transcode("from"))->getLength() != 0) throw std::invalid_argument("XML : Constant Input can't have a predecessor ");

		if(el.getElementsByTagName(XMLString::transcode("value"))->getLength() != 0 && el.getElementsByTagName(XMLString::transcode("weight"))->getLength() != 0) throw std::invalid_argument("XML : Constant Input can't be string and value : \"weight\" or \"value\" "); 

		if(el.getElementsByTagName(XMLString::transcode("value"))->getLength() == 0 && el.getElementsByTagName(XMLString::transcode("weight"))->getLength() == 0) throw std::invalid_argument("XML : Constant Input must have \"weight\" or \"value\" tag "); 

		if(el.getElementsByTagName(XMLString::transcode("value"))->getLength() != 0) xi.value = XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("value"))->item(0))->getTextContent()));
		if( el.getElementsByTagName(XMLString::transcode("weight"))->getLength() != 0)
		{ 
			std::string weight =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("weight"))->item(0))->getTextContent()));
			xi.weight = std::stod( weight);
		}
		
	}
	else
	{
		if( el.getElementsByTagName(XMLString::transcode("from"))->getLength() == 0) throw std::invalid_argument("XML : Input has no predecessor ");
		
		if(el.getElementsByTagName(XMLString::transcode("value"))->getLength() != 0) throw  std::invalid_argument("XML : a non constant input can't have default value ");
	
		xi.uuid_pred = XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("from"))->item(0))->getTextContent()));

		if( el.getElementsByTagName(XMLString::transcode("weight"))->getLength() == 0) throw std::invalid_argument("XML : Link has no weight ");

		std::string weight =  XMLString::transcode(  (dynamic_cast<DOMElement *> (el.getElementsByTagName(XMLString::transcode("weight"))->item(0))->getTextContent()));
		xi.weight = std::stod( weight);
	}

	std::string sparse =  XMLString::transcode( el.getAttribute(  XMLString::transcode( "sparse" )));
	if( sparse.size() == 0 ) 
	{
		xi.isSparse = false;
	}
	else
	{
		if( sparse == "true" ) xi.isSparse = true;
		else if( sparse == "false" ) xi.isSparse = false;
		else throw std::invalid_argument("XML : sparse attribute is boolean, value must be \"true\" or \"false\" ");
	}

	std::string secondary =  XMLString::transcode( el.getAttribute(  XMLString::transcode( "secondary" )));
	if( secondary.size() == 0 ) 
	{
		xi.isSecondary = false;
	}
	else
	{
		if( secondary == "true" ) xi.isSecondary = true;
		else if( secondary == "false" ) xi.isSecondary = false;
		else throw std::invalid_argument("XML : secondary attribute is boolean, value must be \"true\" or \"false\" ");
	}
}



void XmlConverter::loadScript(XScript &xs)
{
	__loadScriptName( xs.name );
	__loadRtToken( xs.rt );
	__loadFunctions( xs.functions );
}


void XmlConverter::__loadLinks( const DOMElement &el, std::vector<XLink> &links  )
{
	DOMElement * xLink;
	
	if( el.getElementsByTagName(XMLString::transcode("links"))->getLength() == 0)  throw  std::invalid_argument("XML : Unable to find links section");
	
	xLink = dynamic_cast<DOMElement *>( el.getElementsByTagName(XMLString::transcode("links"))->item(0))->getFirstElementChild();
	
	if( xLink == NULL)  throw  std::invalid_argument("XML : \"links\" section is empty, any link to load");

	do{
		XLink l;
		__convertXmlToLink(*xLink,l );		
		links.push_back(l);

	}while((xLink = xLink->getNextElementSibling()) != NULL);
}

void XmlConverter::__loadFunctions(std::map<std::string,XFunction> &functions)
{
	DOMElement * xFunc;

	if( m_doc->getElementsByTagName(XMLString::transcode("functions"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find functions section");

	xFunc = dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("functions"))->item(0))->getFirstElementChild()  ;
	if( xFunc == NULL) throw  std::invalid_argument("XML : \"functions\" section is empty, any function to load");

	do{
		XFunction f;
		__convertXmlToFunction( *xFunc, f);
		__loadInputs( *xFunc, f.inputs );
		
		if( functions.find( f.uuid ) == functions.end() ) functions[f.uuid] = f;
		else throw std::invalid_argument("XML : uuid function in xml file has to be unique");

	}while(  (xFunc = xFunc->getNextElementSibling() ) != NULL);
}

void XmlConverter::__loadInputs( const DOMElement &el, std::map<std::string, XInput> &inputs )
{
	DOMElement * xInput;

	
	if( el.getElementsByTagName(XMLString::transcode("inputs"))->getLength() == 0)  return;
	
	xInput = dynamic_cast<DOMElement *>( el.getElementsByTagName(XMLString::transcode("inputs"))->item(0))->getFirstElementChild();
	
	if( xInput == NULL) return;
	
	do{
		XInput i;
		__convertXmlToInput( *xInput, i);
		__loadLinks( *xInput , i.links );
		inputs[i.name] = i;
/*	
		if( i.isAnchor == false && i.links.size() != 1) throw std::invalid_argument("XML : you should give exactly one link on non anchor input : "+ std::to_string(i.links.size())+" given" );
		if( i.isAnchor == true && i.links.size() == 0) throw std::invalid_argument("XML : you should give at least one link on anchor input : "+ std::to_string(i.links.size())+" given" );
*/
	}while(  (xInput = xInput->getNextElementSibling()) != NULL);
}

void XmlConverter::__loadScriptName(std::string &name)
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

void XmlConverter::__loadRtToken( XRtToken & rt )
{
	if(  m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->getLength() == 0) throw  std::invalid_argument("XML : Unable to find \"rt_token\" tag");

	rt.value = std::stod(   XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getTextContent()));

	rt.unit = XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getAttribute( XMLString::transcode(  "unit" )));
	
	rt.uuid = XMLString::transcode( dynamic_cast<DOMElement *>( m_doc->getElementsByTagName(XMLString::transcode("rt_token"))->item(0))->getAttribute( XMLString::transcode(  "uuid" )));

	if( rt.uuid.size() == 0 ) throw std::invalid_argument("XML : RtToken uuid is empty");
}
