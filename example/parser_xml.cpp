#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <iostream>

using namespace std;
using namespace xercesc;


int main (int argc, char* args[]) {


	try {
	    XMLPlatformUtils::Initialize();
	}
	catch (const XMLException& toCatch) {
	    char* message = XMLString::transcode(toCatch.getMessage());
	    cout << "Error during initialization! :\n"
		 << message << "\n";
	    XMLString::release(&message);
	    return 1;
	}
	    XMLPlatformUtils::Initialize();
	    
	XercesDOMParser *parser = new XercesDOMParser();
//	parser->setValidationScheme(XercesDOMParser::Val_Always);
//	parser->setDoNamespaces(true);    // optional

	ErrorHandler* errHandler = (ErrorHandler*) new HandlerBase();
	parser->setErrorHandler(errHandler);

	//char* xmlFile = "example_script.xml";
	std::string xmlFile = "example_script.xml";

	try {
	    parser->parse(xmlFile.c_str());
	}
	catch (const XMLException& toCatch) {
	    char* message = XMLString::transcode(toCatch.getMessage());
	    cout << "Exception message is: \n"
		 << message << "\n";
	    XMLString::release(&message);
	    return -1;
	}
	catch (const DOMException& toCatch) {
	    char* message = XMLString::transcode(toCatch.msg);
	    cout << "Exception message is: \n"
		 << message << "\n";
	    XMLString::release(&message);
	    return -1;
	}
	catch (...) {
	    cout << "Unexpected Exception \n" ;
	    return -1;
	}

	DOMDocument* xmlDoc = parser->getDocument();

	return 0;

	DOMElement* elementRoot = xmlDoc->getDocumentElement();
        if( !elementRoot ) throw(std::runtime_error( "empty XML document" ));
	
	std::cout << "Name Root : " <<  XMLString::transcode(  elementRoot->getTagName()) << std::endl;

//	DOMNodeList* children = elementRoot->getChildNodes();
//	DOMNodeList* children = xmlDoc->getElementsByTagName(XMLString::transcode("boxes"));
//	DOMNodeList* children = xmlDoc->getElementsByTagName(XMLString::transcode("boxes"));
//	const  XMLSize_t nodeCount = children->getLength();

//	std::cout << "NB Node : " << nodeCount << std::endl;
	

	DOMNodeList * bl = xmlDoc->getElementsByTagName(XMLString::transcode("boxes"));
	DOMElement * parent =  dynamic_cast<DOMElement *>( bl->item(0) ) ;

	std::cout <<  XMLString::transcode( parent->getTagName()) << std::endl;

	DOMElement * child = parent->getFirstElementChild();	


	do
	{

	std::cout << XMLString::transcode( child->getTagName()) << std::endl;

	}while(  (child = child->getNextElementSibling()  ) !=NULL); 

/*
	for( XMLSize_t xx = 0; xx < nodeCount; ++xx )
	{
		DOMNode *  currentNode = children->item(xx);
		std::cout << currentNode->getNodeType() << " "  << DOMNode::ELEMENT_NODE   << std::endl;		

		
		if( currentNode->getNodeType() == DOMNode::ELEMENT_NODE  ) 
		{
			DOMElement* currentElement = dynamic_cast< xercesc::DOMElement* >( currentNode );
			std::string name =  XMLString::transcode( currentElement->getTagName()); 
			std::cout << "My name is : " << name << std::endl;
		
			DOMNodeList * schildren = currentElement->getChildNodes();
			const  XMLSize_t nodeCountChild = schildren->getLength();
			std::cout << "and i have " << nodeCountChild << " child" << std::endl; 

			for( XMLSize_t yy =0 ; yy < nodeCountChild; yy++  )
			{
				DOMNode *  currentNodeC = schildren->item(yy);
				std::cout << currentNodeC->getNodeType() << " "  << DOMNode::ELEMENT_NODE   << std::endl;		
				if( currentNodeC->getNodeType() == DOMNode::ELEMENT_NODE  ) 
				{
					DOMElement* currentElementChild = dynamic_cast< xercesc::DOMElement* >( currentNodeC );
                		        std::string name =  XMLString::transcode( currentElementChild->getTagName());
                        		std::cout << "My name is : " << name << std::endl;
					std::cout << "            " << XMLString::transcode( currentElementChild->getAttribute(  XMLString::transcode( "id")  )) << std::endl;
			

				}

			}

		}
		

	}
*/

	delete parser;
	delete errHandler;
	return 0;
}
