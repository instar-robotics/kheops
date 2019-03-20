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

#include "kheops/links/nei_parser.h"


bool NEI_Parser::isMatching(const std::string& nei)
{
        std::regex expr{NEI};
        return regex_match(nei,expr);
}

void NEI_Parser::parseOp(const std::string& nei)
{
        std::regex reg{OP};
        std::smatch m;
        std::regex_search (nei,m,reg);
        std::string OP = m.str();

        OP.erase(std::remove(OP.begin(), OP.end(), '('), OP.end());
        OP.erase(std::remove(OP.begin(), OP.end(), ')'), OP.end());

        op = OP[0];
}

void NEI_Parser::parseBlocks(const std::string& nei)
{
        std::regex cord{CORD};
        std::smatch m;

        std::regex_search (nei,m,cord);
        parseBlock(m.str(),src);

        std::string suffix = m.suffix().str();
        std::regex_search (suffix,m,cord);
        parseBlock(m.str(),dst);
}

void NEI_Parser::parseBlock(const std::string& nei,nBlock& b)
{
	std::string local = nei;

        local.erase(std::remove(local.begin(), local.end(), '('), local.end());
        local.erase(std::remove(local.begin(), local.end(), ')'), local.end());

        std::vector<std::string> strs;
        boost::split(strs,local,boost::is_any_of(","));

        b.row = std::atoi(strs[0].c_str());
        b.col = std::atoi(strs[1].c_str());
        b.height = std::atoi(strs[2].c_str());
        b.width = std::atoi(strs[3].c_str());
}

bool NEI_Parser::isPropagating(const std::string& nei)
{
        std::regex expr{NEI_WP};
        return regex_match(nei,expr);
}

void NEI_Parser::parsePropagation(const std::string& nei)
{
        std::regex reg{PROP};
        std::smatch m;
        std::regex_search (nei,m,reg);
        std::string sprop = m.str();

        std::regex rep{PROP_WR};

        if( regex_match(sprop,rep))
        {
                std::vector<std::string> strs;
                boost::split(strs,sprop,boost::is_any_of(")"));

                std::regex rs{START};
                if( regex_match( strs[1], rs ) )
                {
                        prop.nbr = -1;
                }
                else
                {
                        prop.nbr = std::atoi(strs[1].c_str());
                }
                sprop = strs[0];
        }
        else
        {
                sprop.erase(std::remove(sprop.begin(), sprop.end(), ')'), sprop.end());
                prop.nbr = 1;
        }

        sprop.erase(std::remove(sprop.begin(), sprop.end(), '('), sprop.end());

        std::vector<std::string> vpr;
        boost::split(vpr,sprop,boost::is_any_of(","));

        prop.src = vpr[0][0];
        prop.dst = vpr[1][0];
}

bool NEI_Parser::checkSize()
{
	if( getOp() == OTA_OP ) return true;

	bool ret = false;
	if( src.height * src.width == dst.height * dst.width)
	{
//		if( src.height == dst.height && src.width == dst.width 
//			|| src.height == dst.width && src.width == dst.height)	 ret = true;
		
		if( src.height == dst.height && src.width == dst.width) ret = true;
	}	
	return ret;
}

int NEI_Parser::getDim()
{
	if( getOp() == OTA_OP ) return -1;

	return src.width * src.height;
}

void NEI_Parser::parseExpr(const std::string& nei)
{
	if( isMatching(nei) )
        {
                parseOp(nei);
                parseBlocks(nei);

                if( isPropagating(nei) )
                {
                        parsePropagation(nei);
                }
        }
	else
	{
		throw std::invalid_argument("NEI_Parser : expression doesn't match ONE_TO_NEI connectivities rules");
	}

	if( !checkSize() ) throw std::invalid_argument("NEI_Parser : invalid size between source and destination for ONE TO ONE sub-projection");

}
