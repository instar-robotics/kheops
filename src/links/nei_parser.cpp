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
#include <string>

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

        b.row = std::stoi (strs[0],nullptr);
        b.col = std::stoi(strs[1],nullptr);
        b.height = std::stoi(strs[2],nullptr);
        b.width = std::stoi(strs[3],nullptr);
}

bool NEI_Parser::isPropagating(const std::string& nei)
{
        std::regex expr{NEI_WP};
        return regex_match(nei,expr);
}

void NEI_Parser::parsePropagation(const std::string& nei)
{
	prop.src_roffset = 0;
        prop.src_coffset = 0;

        prop.dst_roffset = 0;
        prop.dst_coffset = 0;


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
                        prop.nbr = RINFINITY;
                }
                else
                {
                        prop.nbr = std::stoi(strs[1],nullptr);
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

	std::regex rprop_base{PROP_RULEBASE};
        std::regex rprop_full{PROP_RULEFULL};

	if( regex_match(vpr[0],rprop_base) )
	{	
		if( prop.src == RCOL || prop.src == RDIA) prop.src_coffset = src.width;
		if( prop.src == RROW || prop.src == RDIA) prop.src_roffset = src.height;
	}
	else if( regex_match(vpr[0],rprop_full) )
	{
		int offset = std::stoi(vpr[0].substr(1),nullptr);
		if( prop.src == RCOL || prop.src == RDIA) prop.src_coffset = offset;
		if( prop.src == RROW || prop.src == RDIA) prop.src_roffset = offset;
	}

	if( regex_match(vpr[1],rprop_base) )
	{
		if( prop.dst == RCOL || prop.dst == RDIA) prop.dst_coffset = dst.width;
		if( prop.dst == RROW || prop.dst == RDIA) prop.dst_roffset = dst.height;
	}
	else if( regex_match(vpr[1],rprop_full) )
	{
		int offset = std::stoi(vpr[1].substr(1),nullptr);
		if( prop.dst == RCOL || prop.dst == RDIA) prop.dst_coffset = offset;
		if( prop.dst == RROW || prop.dst == RDIA) prop.dst_roffset = offset;
	}
}

bool NEI_Parser::checkProjSize()
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

bool NEI_Parser::checkSrcDim()
{
	// checkSize Src :
	if( src.row + src.height > iRows || src.col + src.width > iCols)
	{
		return false;
	}
	return true;
}

bool NEI_Parser::checkDstDim()
{
	// checksize dst
	if( dst.row + dst.height > oRows ||  dst.col + dst.width > oCols)
	{
		return false;
	}
	return true;
}

void NEI_Parser::checkPropagation()
{
	if( prop.nbr != RINFINITY)
	{
		if( (prop.src == RCOL || prop.src == RDIA)  && (src.width + (prop.nbr-1) * prop.src_coffset ) > iCols - src.col) throw std::invalid_argument("NEI_Parser : propagation is outside the source matrix ");
		if( (prop.src == RROW || prop.src == RDIA) && (src.height + (prop.nbr-1) * prop.src_roffset ) > iRows - src.row) throw std::invalid_argument("NEI_Parser : propagation is outside the source matrix ");
		if( (prop.dst == RCOL || prop.dst == RDIA) && (dst.width + (prop.nbr-1) * prop.dst_coffset ) > oCols - dst.col) throw std::invalid_argument("NEI_Parser : propagation is outside the source matrix ");
		if( (prop.dst == RROW || prop.dst == RDIA) && (dst.height + (prop.nbr-1) * prop.dst_roffset) > oRows - dst.row) throw std::invalid_argument("NEI_Parser : propagation is outside the source matrix ");
	}
	else
	{
		int nbrSrc_row=1,nbrSrc_col=1; 
		int nbrDst_row=1,nbrDst_col=1;

                if( prop.src == RCOL || prop.src == RDIA )
                {
			//if( (iCols - src.col) % src.width != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
			//if( (iCols - src.col - src.width) % prop.src_coffset != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
                         //nbrSrc = ((iCols - src.col) / src.width);
                         nbrSrc_col = ((iCols - src.col - src.width) / prop.src_coffset )+1;
                }
                if(prop.src == RROW || prop.src == RDIA) {
			//if( (iRows - src.row) % src.height != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
			//if( (iRows - src.row - src.height) % prop.src_roffset != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
		       	//nbrSrc = ((iRows - src.row) / src.height);
		       	nbrSrc_row = ((iRows - src.row - src.height)/ prop.src_roffset)+1;
		}

                if( prop.dst == RCOL || prop.dst == RDIA )
                {
			//if( (oCols - dst.col) % dst.width != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
			//if( ((oCols - dst.col) % prop.dst_coffset + dst.width) % prop.dst_coffset != 0 ) throw std::invalid_argument("NEI_Parser : destination propagation rule size not match");
 	
                        nbrDst_col = ((oCols - dst.col - dst.width) / prop.dst_coffset)+1 ;
                }
                if( prop.dst == RROW || prop.dst == RDIA)
		{
			//if( (oRows - dst.row) % dst.height != 0 ) throw std::invalid_argument("NEI_Parser : source propagation rule size not match");
			//if( (oRows - dst.row - dst.height) % prop.dst_roffset != 0 ) throw std::invalid_argument("NEI_Parser : destination propagation rule size not match PPP");
		       	nbrDst_row = ((oRows - dst.row - dst.height) / prop.dst_roffset)+1 ;
		}

		//if(nbrSrc != nbrDst && prop.src != RNOT && prop.dst != RNOT ) throw std::invalid_argument("NEI_Parser : source and destination dimension are wrong for propagation rule");

		if( prop.src == RNOT && prop.dst == RNOT)
		{	
			prop.nbr = 1 ; 
		}
		else if( prop.src == RNOT )
		{
			if( prop.dst == RDIA ) 	prop.nbr = std::min(nbrDst_row,nbrDst_col);
			else if( prop.dst == RCOL ) prop.nbr = nbrDst_col;
			else if( prop.dst == RROW ) prop.nbr = nbrDst_row;
		}
		else if( prop.dst == RNOT)
		{
			if( prop.src == RDIA ) prop.nbr = std::min(nbrSrc_row,nbrSrc_col);
			else if( prop.src == RCOL) prop.nbr = nbrSrc_col;
			else if( prop.src == RROW) prop.nbr = nbrSrc_row;
		}
		else{
			int nbrSrc=1,nbrDst=1;
			
			if( prop.dst == RDIA ) 	nbrDst = std::min(nbrDst_row,nbrDst_col);
			else if( prop.dst == RCOL ) nbrDst = nbrDst_col;
			else if( prop.dst == RROW ) nbrDst = nbrDst_row;

			if( prop.src == RDIA ) nbrSrc = std::min(nbrSrc_row,nbrSrc_col);
			else if( prop.src == RCOL) nbrSrc = nbrSrc_col;
			else if( prop.src == RROW) nbrSrc = nbrSrc_row;

		       	prop.nbr = std::min(nbrSrc,nbrDst);
		}
		//prop.nbr = std::max(nbrSrc,nbrDst);
	}
}


int NEI_Parser::getNbCon()
{
	if( getOp() == OTA_OP ) return -1;

	return src.width * src.height;
}

void NEI_Parser::parseExpr(const std::string& nei)
{
	prop.nbr = 1;

	if( isMatching(nei) )
        {
                parseOp(nei);
                parseBlocks(nei);

                if( isPropagating(nei) )
                {
                        parsePropagation(nei);
			checkPropagation();
                }
        }
	else
	{
		throw std::invalid_argument("NEI_Parser : expression doesn't match ONE_TO_NEI connectivities rules");
	}

	if( !checkProjSize() ) throw std::invalid_argument("NEI_Parser : invalid size between source and destination for ONE TO ONE sub-projection");

	if( !checkSrcDim() ) throw std::invalid_argument("NEI_Parser : incompatible size for source projection rule.");
	if( !checkDstDim() ) throw std::invalid_argument("NEI_Parser : incompatible size for destination projection rule.");

}

void NEI_Parser::nextBlock()
{
	if( prop.nbr > 0 )
	{
		prop.nbr--; 	

		src.row += prop.src_roffset;
		src.col += prop.src_coffset;

		dst.row += prop.dst_roffset;
		dst.col += prop.dst_coffset;
	}
}

bool NEI_Parser::hasBlock()
{
	return prop.nbr > 0;
}
