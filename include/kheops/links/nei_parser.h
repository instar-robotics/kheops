/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of kheops <https://github.com/instar-robotics/kheops>.
 
  kheops is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  kheops is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
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
//   d : r or c or d or n
//   r : int or *
//

const std::string NEI = "\\[\\(([0-9]+[,]){3}[0-9]+\\)[\\.x]\\(([0-9]+[,]){3}[0-9]+\\)\\](\\([rcnd][0-9]*,[rcnd][0-9]*\\)([0-9]*|\\*))?";
const std::string NEI_WP = "\\[\\(([0-9]+[,]){3}[0-9]+\\)[\\.x]\\(([0-9]+[,]){3}[0-9]+\\)\\](\\([rcnd][0-9]*,[rcnd][0-9]*\\)([0-9]*|\\*)){1}";

const std::string CORD = "\\(([0-9]+[,]){3}[0-9]+\\)";
const std::string OP = "\\)[\\.x]\\(";
const std::string PROP = "(\\([rcnd][0-9]*,[rcnd][0-9]*\\))(.*)$";
const std::string PROP_WR = "(\\([rcnd][0-9]*,[rcnd][0-9]*\\)([0-9]+|\\*{1})){1}";
const std::string START = "^.*\\*{1}$";
const std::string PROP_RULEBASE= "^[rcnd]$";
const std::string PROP_RULEFULL = "^[rcnd][0-9]+$";

const char RNOT = 'n';
const char RCOL = 'c';
const char RROW = 'r';
const char RDIA = 'd';
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

	int src_roffset;
	int src_coffset;
	
	int dst_roffset;
	int dst_coffset;
};

class NEI_Parser
{
	private :
	
		char op;
		nBlock src;	
		nBlock dst;	
		nPropagation prop;

		int iRows;
		int iCols;
		int oRows;
		int oCols;

	public : 

		NEI_Parser() : iRows(0),iCols(0),oRows(0),oCols(0) {
			prop.nbr = 0;
		}
		NEI_Parser(int iRows,int iCols,int oRows, int oCols) : iRows(iRows),iCols(iCols),oRows(oRows),oCols(oCols) {
			prop.nbr = 0;
		}
		~NEI_Parser(){}

		void setDimension(int iRows,int iCols, int oRows, int oCols);

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

		void nextBlock();
		bool hasBlock();

		void checkPropagation();
		bool checkSrcDim();
		bool checkDstDim();
		bool checkProjSize();
		int getNbCon();
};

#endif // __NEI_PARSER_H__
