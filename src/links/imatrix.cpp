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


#include "kheops/links/imatrix.h"
#include "kheops/links/nei_parser.h"
#include "kheops/ros/rospublisher.h"

/*******************************************************************************************************/
/*****************                   iScalarMatrix Section                           *******************/
/*******************************************************************************************************/

iScalarMatrix::iScalarMatrix() : iMatrix()
{
	o_pub = new RosScalarPublisher(1); 
}

iScalarMatrix::iScalarMatrix( MatrixXd const *  i) : iMatrix(i)
{
	o_pub = new RosScalarPublisher(1); 
}

iScalarMatrix::~iScalarMatrix()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() ) o_pub->close();
		delete(o_pub);
	}
}

Ref<MatrixXd> iScalarMatrix::accumulate(Ref<MatrixXd> res)
{
	return res = (*input) * weight;
}

Ref<MatrixXd> iScalarMatrix::mul_accumulate(Ref<MatrixXd> res)
{
	return res=res.cwiseProduct( (*input) * weight );
}

Ref<MatrixXd> iScalarMatrix::sum_accumulate(Ref<MatrixXd> res)
{
	res.array() += (*input).array() * weight;
	return res;
}


Ref<MatrixXd> iScalarMatrix::sub_accumulate(Ref<MatrixXd> res)
{
	res.array()-= (*input).array() * weight;
	return res;
}

Ref<MatrixXd> iScalarMatrix::div_accumulate(Ref<MatrixXd> res)
{
	return res=res.cwiseQuotient( (*input) * weight );
}


/*******************************************************************************************************/
/******************                      iMMatrix Section                            *******************/
/*******************************************************************************************************/

Map<MatrixXd> getMapRow(MatrixXd & m)
{
        return Map<MatrixXd>( m.data() , 1 , m.size() ) ;
}

Map<MatrixXd> getMapCol(MatrixXd & m)
{
        return Map<MatrixXd> ( m.data(), m.size() , 1 ) ;
}

Map<VectorXd> getMapVect(MatrixXd & m)
{
        return Map<VectorXd> ( m.data(), m.size() ) ;
}

Map<const MatrixXd> getCMapRow(const MatrixXd & m)
{
        return Map<const MatrixXd>( m.data() , 1 , m.size() ) ;
}

Map<const MatrixXd> getCMapCol(const MatrixXd & m)
{
        return Map<const MatrixXd> ( m.data(), m.size() , 1 ) ;
}

Map<const VectorXd> getCMapVect(const MatrixXd & m)
{
        return Map<const VectorXd> ( m.data(), m.size() ) ;
}
/***********************************************************************/
/*************************  Constructor Section  ***********************/
/***********************************************************************/

iMMatrix::iMMatrix(unsigned int oRow, unsigned int oCol) : iMatrix(), oRow(oRow),oCol(oCol)
{
        o_pub = new RosMatrixPublisher(1);
}

iMMatrix::iMMatrix( MatrixXd const *  i, unsigned int oRow , unsigned int oCol) : iMatrix(i), oRow(oRow), oCol(oCol)
{
        o_pub = new RosMatrixPublisher(1);
}

iMMatrix::iMMatrix( MatrixXd const *  i, unsigned int oRow , unsigned int oCol, double value) : iMatrix(i), oRow(oRow), oCol(oCol)
{
        o_pub = new RosMatrixPublisher(1);
        resizeWeight();
        initWeight(value);
}

iMMatrix::~iMMatrix()
{
        if( o_pub != NULL )
        {
                if( o_pub->is_open() ) o_pub->close();
                delete(o_pub);
        }
}

/***********************************************************************/
/************************  Weight Management API  **********************/
/***********************************************************************/

void iMMatrix::resizeWeight()
{
        if( iRows()==0 || iCols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing input size");
        if( oRows()==0 || oCols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing output size");

        weight.resize(  getInitWRows() , getInitWCols() );
}

void iMMatrix::initWeight(double w)
{
        weight << MatrixXd::Constant( weight.rows() , weight.cols(),w);
}

bool iMMatrix::checkWeightSize(unsigned int rows, unsigned int cols )
{
        if( rows == weight.rows() && cols == weight.cols() ) return true;
        return false;
}

/***********************************************************************/
/*************************  Weight Access API  *************************/
/***********************************************************************/

void iMMatrix::w(const MatrixXd& weight)
{
        if(weight.rows() != getInitWRows() || weight.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Weight Matrix dimension");
        this->weight = weight;
}


void iMMatrix::wref(const Ref<const MatrixXd>& weight)
{
        if(weight.rows() != getInitWRows() || weight.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Weight Matrix dimension");
        this->weight = weight;
}

Map<MatrixXd> iMMatrix::wm()
{
        return Map<MatrixXd>( weight.data() , weight.rows() , weight.cols() ) ;
}

Map<MatrixXd> iMMatrix::wj(unsigned int oRow,unsigned int oCol)
{
        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXd>( weight.data() + offset , iRows() , iCols()) ;
}

Map<MatrixXd> iMMatrix::wj(unsigned int wCol)
{
        if( wCol >= wCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (wCol)  ;

        return Map<MatrixXd>( weight.data() + offset , iRows() , iCols()) ;
}

Map<MatrixXd> iMMatrix::wj_row(unsigned int oRow,unsigned int oCol)
{
        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXd>( weight.data() + offset , 1 , iSize()) ;
}

Map<MatrixXd> iMMatrix::wj_row(unsigned int wCol)
{
        if( wCol >= wCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (wCol)  ;

        return Map<MatrixXd>( weight.data() + offset , 1 , iSize()) ;
}

Map<MatrixXd> iMMatrix::wj_col(unsigned int oRow,unsigned int oCol)
{
        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("PUTEiLink : try to get weight for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXd>( weight.data() + offset ,  iSize(), 1) ;
}

Map<MatrixXd> iMMatrix::wj_col(unsigned int wCol)
{
        if( wCol >= wCols() ) throw std::invalid_argument("PUTEiLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (wCol)  ;

        return Map<MatrixXd>( weight.data() + offset , iSize(), 1) ;
}

Map<VectorXd> iMMatrix::wj_vect(unsigned int oRow,unsigned int oCol)
{
        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        
	unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;
        
	return Map<VectorXd>( weight.data() + offset , iSize()) ;
}

Map<VectorXd> iMMatrix::wj_vect(unsigned int wCol)
{
        if( wCol >= wCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (wCol)  ;

        return Map<VectorXd>( weight.data() + offset , iSize()) ;
}

double iMMatrix::wij(unsigned int wRow,unsigned int wCol)
{
        if( wRow >= wRows() || wCol >= wCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron for Weight Matrix");
        return weight(wRow,wCol);
}

void iMMatrix::wij(double weight, unsigned int wRow, unsigned int wCol)
{
        if( wRow >= wRows() || wCol >= wCols()) throw std::invalid_argument("iLink : row,col index are outside the Weight Matrix boundaries");

        this->weight(wRow,wCol) = weight;
}

void iMMatrix::wj(const Ref<VectorXd> &weight,unsigned int wCol)
{
        if( wCol >= wCols()) throw std::invalid_argument("iLink : col index is outside the Weight Matrix boundaries");
        this->weight.col(wCol) = weight;
}

/***********************************************************************/
/*****************************  Input API  *****************************/
/***********************************************************************/

Map<const MatrixXd> iMMatrix::irow()
{
        return Map<const MatrixXd> ( i().data(),1, iSize() ) ;
}

Map<const MatrixXd> iMMatrix::icol()
{
        return Map<const MatrixXd> ( i().data(), iSize(), 1 ) ;
}

Map<const VectorXd> iMMatrix::ivec()
{
        return Map<const VectorXd> ( i().data(),iSize() ) ;
}

/***********************************************************************/
/****************************  Filter API  *****************************/
/***********************************************************************/


void iMMatrix::f(const MatrixXb &filter)
{
        if(filter.rows() != getInitWRows() || filter.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Filter Matrix dimension");
        this->filter = filter;
}

Map<MatrixXb> iMMatrix::fm()
{
        return Map<MatrixXb>( filter.data() , filter.rows() , filter.cols() ) ;
}

void iMMatrix::fref(const Ref<const MatrixXb>& filter)
{
        if(filter.rows() != getInitWRows() || filter.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Filter Matrix dimension");
        this->filter = filter;
}

void iMMatrix::fij(typename MatrixXb::Scalar weight, unsigned int fRow, unsigned int fCol)
{
        if( fRow >= getFRows() || fCol >= getFCols()) throw std::invalid_argument("iLink : row,cols index are outside the Filter matrix boundaries");

        this->filter(fRow,fCol) = weight;
}

void iMMatrix::fj(const Ref<VectorXb> &weight,unsigned int fCol)
{
        if( fCol >= getFCols()) throw std::invalid_argument("iLink : col index is outside the Weight Matrix boundaries");
        this->filter.col(fCol) = weight;
}

Map<MatrixXb> iMMatrix::fj(unsigned int oRow,unsigned int oCol)
{

        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get filter for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXb>( filter.data() + offset , iRows() , iCols()) ;
}

Map<MatrixXb> iMMatrix::fj(unsigned int o)
{
        if(o >= wCols()) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (o)  ;

        return Map<MatrixXb>( filter.data() + offset , iRows() , iCols()) ;
}

Map<MatrixXb> iMMatrix::fj_row(unsigned int oRow,unsigned int oCol)
{

        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get filter for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXb>( filter.data() + offset , 1 , iSize()) ;
}

Map<MatrixXb> iMMatrix::fj_row(unsigned int o)
{
        if(o >= wCols()) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (o)  ;

        return Map<MatrixXb>( filter.data() + offset ,1 ,  iSize()) ;
}

Map<MatrixXb> iMMatrix::fj_col(unsigned int oRow,unsigned int oCol)
{

        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get filter for an outbound neuron");

        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<MatrixXb>( filter.data() + offset ,  iSize(), 1) ;
}

Map<MatrixXb> iMMatrix::fj_col(unsigned int o)
{
        if(o >= wCols()) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iSize() * (o)  ;

        return Map<MatrixXb>( filter.data() + offset , iSize() , 1) ;
}

Map<VectorXb> iMMatrix::fj_vec(unsigned int oRow,unsigned int oCol)
{
        if( oRow >= oRows() || oCol >= oCols() ) throw std::invalid_argument("iLink : try to get filter for an outbound neuron");
        unsigned int offset = iSize() * (oRow * oCols() + oCol)  ;

        return Map<VectorXb>( filter.data() + offset , iSize()) ;
}

Map<VectorXb> iMMatrix::fj_vec(unsigned int o)
{
        if(o >= wCols()) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
        unsigned int offset = iRows() * iCols() * (o)  ;

        return Map<VectorXb>( filter.data() + offset , iSize()) ;
}

double iMMatrix::fij(unsigned int fRows,unsigned int fCols)
{
        if( fRows >= getFRows() || fCols >= getFCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron for Filter Matrix");
        return filter(fRows,fCols);
}

void iMMatrix::buildFilter(const XConnectivity& con)
{
        if( con.type == one_to_one )
        {
                //TODO : modulo or crash if dimension is not oK
                filter =  MatrixXb::Identity( getInitWRows(), getInitWCols()  );
        }
        else if( con.type == one_to_all )
        {
                filter =  MatrixXb::Constant( getInitWRows(), getInitWCols(),1 );
        }
        else if( con.type == one_to_nei)
        {
		filter = MatrixXb::Constant( getInitWRows(), getInitWCols(), 0 );

		NEI_Parser parser(iRows(),iCols(),oRows(), oCols());

		for( unsigned int i = 0; i < con.nei_expr.size(); i++)
		{
			try{
				parser.parseExpr(con.nei_expr[i]);

				while(parser.hasBlock())	 
				{
					//if one to one projection
					if( parser.getOp() == OTO_OP )
					{
						/* I = r + cxR + Dr + Dc * R 
						 * J = r + cxR + Dr + Dc * R 
						 *
						 * Dr = I % H;
						 * Dc = (int) i / H;
						 *
						 * Or
						 *
						 * Dr = i / W;
						 * Dc = i % W +1;
						 */
			
						MatrixXd::Index size = parser.getNbCon();
						for( MatrixXd::Index i = 0; i < size ; i++)
						{
							MatrixXd::Index dRsrc,dCsrc,dRdst,dCdst;

							dRsrc = i % parser.getSrc().height;
							dCsrc = (int)(i / parser.getSrc().height);
						
							dRdst = i % parser.getDst().height;
							dCdst = (int)(i / parser.getDst().height);

							MatrixXd::Index I = parser.getSrc().row + parser.getSrc().col * iRows() + dRsrc + dCsrc * iRows();
							MatrixXd::Index J = parser.getDst().row + parser.getDst().col * oRows() + dRdst + dCdst * oRows();
							filter(I,J) = 1;		
						}
					}
					else if( parser.getOp() == OTA_OP)
					{
						MatrixXb tmpFilter;	

						MatrixXb src = MatrixXb::Constant(iRows(),iCols(),0);
						src.block(parser.getSrc().row, parser.getSrc().col, parser.getSrc().height, parser.getSrc().width)  = MatrixXb::Constant(parser.getSrc().height,parser.getSrc().width, 1);

						MatrixXb dst = MatrixXb::Constant(oRows(),oCols(),0);
						dst.block(parser.getDst().row, parser.getDst().col, parser.getDst().height, parser.getDst().width)  = MatrixXb::Constant(parser.getDst().height,parser.getDst().width, 1);

						auto vSrc = Map<VectorXb>(src.data(), src.size());
						auto vDst = Map<RVectorXb>(dst.data(), dst.size());

						tmpFilter = vSrc * vDst;

						filter += tmpFilter;
					}
					parser.nextBlock();
				}
			}
			catch(...)
			{
				std::cout << "iMMatrix [" << getUuid() << "] : invalid ONE_TO_NEI connectivity rule !" << std::endl; 
				std::exception_ptr eptr = std::current_exception();;
				if( eptr)  std::rethrow_exception(eptr);
			}
		}
        }
        else throw std::invalid_argument("iMMatrix ["+getUuid()+"] : unknown connectivity : "+con.type+" , allowed : "+one_to_one+" , "+one_to_all+" or "+one_to_nei);
}

