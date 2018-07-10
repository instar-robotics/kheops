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

#include "ilink.h"
#include "rospublisher.h"

/********************************************************************************************************/
/******************                     ILinkBase Section                           *******************/
/********************************************************************************************************/

iLinkBase::~iLinkBase(){}

/********************************************************************************************************/
/******************                     iScalar Section                           *******************/
/********************************************************************************************************/

iScalar::iScalar() : iLink() 
{
	o_pub = new RosScalarPublisher(1); 
}    


iScalar::iScalar(double const * i) : iLink(i)
{
	o_pub = new RosScalarPublisher(1); 
}

iScalar::~iScalar()
{
	if( o_pub != NULL )
	{
		if( o_pub->is_open() ) o_pub->close();
		delete(o_pub);
	}
}

/********************************************************************************************************/
/******************                   iScalarMatrix Section                           *******************/
/********************************************************************************************************/

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

/********************************************************************************************************/
/******************                      iMMatrix Section                             *******************/
/********************************************************************************************************/

Map<MatrixXd> getMapRow(MatrixXd & m)
{
	return Map<MatrixXd>( m.data() , 1 , m.rows() * m.cols()) ;
}

Map<MatrixXd> getMapCol(MatrixXd & m)
{
	return Map<MatrixXd> ( m.data(), m.rows() * m.cols(), 1 ) ;
}

/***********************************************************************/
/*************************  Constructor Section  ***********************/
/***********************************************************************/

iMMatrix::iMMatrix(unsigned int oRows, unsigned int oCols) : iMatrix(), oRows(oRows),oCols(oCols)
{
	o_pub = new RosMatrixPublisher(1);
}

iMMatrix::iMMatrix( MatrixXd const *  i, unsigned int oRows , unsigned int oCols) : iMatrix(i), oRows(oRows), oCols(oCols)
{
	o_pub = new RosMatrixPublisher(1);
}

iMMatrix::iMMatrix( MatrixXd const *  i, unsigned int oRows , unsigned int oCols, double value) : iMatrix(i), oRows(oRows), oCols(oCols)
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
	if( getIRows()==0 || getICols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing input size");
	if( getORows()==0 || getOCols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing output size");

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

Map<MatrixXd> iMMatrix::wj(unsigned int oRows,unsigned int oCols)
{ 
	if( oRows >= getORows() || oCols >= getOCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");

	unsigned int offset = getIRows() * getICols() * (oRows * getOCols() + oCols)  ;

	return Map<MatrixXd>( weight.data() + offset , getIRows() , getICols()) ;
}

Map<MatrixXd> iMMatrix::wj(unsigned int wCols)
{ 
	if( wCols >= getWCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
	unsigned int offset = getIRows() * getICols() * (wCols)  ;

	return Map<MatrixXd>( weight.data() + offset , getIRows() * getICols() , 1) ;
}

double iMMatrix::wij(unsigned int wRows,unsigned int wCols)
{
	if( wRows >= getWRows() || wCols >= getWCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron for Weight Matrix");
	return weight(wRows,wCols);
}

void iMMatrix::wij(double weight, unsigned int wRow, unsigned int wCol) 
{
	if( wRow >= getWRows() || wCol >= getWCols()) throw std::invalid_argument("iLink : row,col index are outside the Weight Matrix boundaries");

	this->weight(wRow,wCol) = weight;
}

void iMMatrix::wj(const Ref<VectorXd> &weight,unsigned int wCol)
{
	if( wCol >= getWCols()) throw std::invalid_argument("iLink : col index is outside the Weight Matrix boundaries");
	this->weight.col(wCol) = weight;
}

/***********************************************************************/
/*****************************  Input API  *****************************/
/***********************************************************************/

Map<const MatrixXd> iMMatrix::irow()
{
	return Map<const MatrixXd> ( i().data(),1, getIRows()* getICols() ) ;
}

Map<const MatrixXd> iMMatrix::icol()
{
	return Map<const MatrixXd> ( i().data(),getIRows()* getICols(), 1 ) ;
}

/***********************************************************************/
/****************************  Filter API  *****************************/
/***********************************************************************/

void iMMatrix::f(const MatrixXd &filter)
{
	if(filter.rows() != getInitWRows() || filter.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Filter Matrix dimension");
	this->filter = filter;
}

void iMMatrix::fref(const Ref<const MatrixXd>& filter)
{
	if(filter.rows() != getInitWRows() || filter.cols() != getInitWCols()) throw std::invalid_argument("iLink : Matrix size doesn't match with Filter Matrix dimension");
	this->filter = filter;

}

void iMMatrix::fij(double weight, unsigned int fRow, unsigned int fCol)
{
	if( fRow >= getFRows() || fCol >= getFCols()) throw std::invalid_argument("iLink : row,cols index are outside the Filter matrix boundaries");

	this->filter(fRow,fCol) = weight;
}

void iMMatrix::fj(const Ref<VectorXd> &weight,unsigned int fCol)
{
	if( fCol >= getFCols()) throw std::invalid_argument("iLink : col index is outside the Weight Matrix boundaries");
	this->filter.col(fCol) = weight;
}

Map<const MatrixXd> iMMatrix::fm()
{
	return Map<const MatrixXd>( filter.data() , filter.rows() , filter.cols() ) ;
}

Map<const MatrixXd> iMMatrix::fj(unsigned int oRows,unsigned int oCols)
{

	if( oRows >= getORows() || oCols >= getOCols() ) throw std::invalid_argument("iLink : try to get filter for an outbound neuron");

	unsigned int offset = getIRows() * getICols() * (oRows * getOCols() + oCols)  ;

	return Map<const MatrixXd>( filter.data() + offset , getIRows() , getICols()) ;
}

Map<const MatrixXd> iMMatrix::fj(unsigned int wCols)
{
	if( wCols >= getWCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron");
	unsigned int offset = getIRows() * getICols() * (wCols)  ;

	return Map<const MatrixXd>( filter.data() + offset , getIRows() * getICols() , 1) ;
}

double iMMatrix::fij(unsigned int fRows,unsigned int fCols)
{
	if( fRows >= getFRows() || fCols >= getFCols() ) throw std::invalid_argument("iLink : try to get weight for an outbound neuron for Filter Matrix");
	return filter(fRows,fCols);
}

void iMMatrix::buildFilter(const std::string& con)
{
	if( con == one_to_one )
	{
		filter =  MatrixXd::Identity( getInitWRows(), getInitWCols()  );
	}
	else if( con == one_to_all )
	{
		filter =  MatrixXd::Constant( getInitWRows(), getInitWCols(),1 );
	}
	else if( con == one_to_nei)
	{
	
	}
	else  std::invalid_argument("iLink : unknown connectivity : "+con+" , allowed : "+one_to_one+" , "+one_to_all+" or "+one_to_nei);
}


/*


Ref<MatrixXd> iMMatrix::weigthedSum(Ref<MatrixXd> out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : iMMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs = weight * ve;

	return out;
}

Ref<MatrixXd> iMMatrix::weigthedSumAccu(Ref<MatrixXd> out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : iMMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs += weight * ve;

	return out;
}

{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : DenseMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs += weight.cwiseProduct(filter) * ve;

	return out;
}

void iSparseMatrix::buildFilter(const std::string& con)
{
	if( con == one_to_one ) 
	{
		//BUILD ID Matrix
	}
}
*/


/*
value compute_val()


for(int i = 0;  i < w.cols(); i++)
{
s(i) = (ve + w.col(i)).sum() ;
}

std::cout << s << std::endl;

s.colwise().sum();
*/

