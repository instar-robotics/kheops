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

#ifndef __NEI_PARSER_H__
#define __NEI_PARSER_H__

#include <regex>
#include <boost/algorithm/string.hpp>
#include <vector>

// ONE_TO_NEI expression : [(src)op(dst)](d;d)r
//
//   src : int,int,int,int  -> row,col,height,width
//   dst : int,int,int,int  -> row,col,height,width
//
//   op : . or x
//   d : r or c
//   r : int or *

const std::string NEI = "\\[\\(([0-9]+[,]){3}[0-9]+\\)[\\.x]\\(([0-9]+[,]){3}[0-9]+\\)\\](\\([rc],[rc]\\)([0-9]*|\\*))?";
const std::string NEI_WP = "\\[\\(([0-9]+[,]){3}[0-9]+\\)[\\.x]\\(([0-9]+[,]){3}[0-9]+\\)\\](\\([rc],[rc]\\)([0-9]*|\\*)){1}";

const std::string CORD = "\\(([0-9]+[,]){3}[0-9]+\\)";
const std::string OP = "\\)[\\.x]\\(";
const std::string PROP = "(\\([rc],[rc]\\))(.*)$";
const std::string PROP_WR = "(\\([rc],[rc]\\)([0-9]+|\\*{1})){1}";
const std::string START = "^.*\\*{1}$";

const char RCOL = 'c';
const char RROW = 'r';
const int  RINFINITY = -1;

const char OTO_OP = '.';
const char OTA_OP = 'x';

struct nBlock
{
        int row;
        int col;
        int height;
        int width;
};

struct nPropagation
{
        char src;
        char dst;
        int nbr;
};

class NEI_Parser
{
	private :
	
		char op;
		nBlock src;	
		nBlock dst;	
		nPropagation prop;

	public : 

		NEI_Parser(){}
		~NEI_Parser(){}

		void parseExpr(const std::string& nei);
		void parseOp(const std::string& nei);
		void parseBlocks(const std::string& nei);
		void parseBlock(const std::string& nei,nBlock& b);
		void parsePropagation(const std::string& nei);

		bool isPropagating(const std::string& nei);
		bool isMatching(const std::string& nei);

		const nBlock& getSrc(){return src;}
		const nBlock& getDst(){return dst;}
		const nPropagation& getnPropagation(){return prop;}
		char getOp(){ return op;}

		bool checkSize();
		int getDim();
};

#endif // __NEI_PARSER_H__
