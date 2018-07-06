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

iScalar::iScalar() : iLink(), iCombinator() 
{
	o_pub = new RosScalarPublisher(1); 
}    


iScalar::iScalar(double const * i) : iLink(i), iCombinator() 
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

iScalarMatrix::iScalarMatrix() : iMatrix(),iCombinator() 
{
	o_pub = new RosScalarPublisher(1); 
}

iScalarMatrix::iScalarMatrix( MatrixXd const *  i) : iMatrix(i), iCombinator()
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

MatrixXd& iScalarMatrix::accumulate(MatrixXd& res)
{
	return res = (*input) * weight;
}

MatrixXd& iScalarMatrix::mul_accumulate(MatrixXd& res)
{
	return res=res.cwiseProduct( (*input) * weight );
}

MatrixXd& iScalarMatrix::sum_accumulate(MatrixXd& res)
{
	res.array() += (*input).array() * weight;
	return res;
}


MatrixXd& iScalarMatrix::sub_accumulate(MatrixXd& res)
{
	res.array()-= (*input).array() - weight;
	return res;
}

MatrixXd& iScalarMatrix::div_accumulate(MatrixXd& res)
{
	return res=res.cwiseQuotient( (*input) * weight );
}

/********************************************************************************************************/
/******************                   iMMatrix Section                           *******************/
/********************************************************************************************************/

iMMatrix::iMMatrix(unsigned int rows, unsigned int cols, double dvalue) : iMatrix(), orows(rows),ocols(cols),dvalue(dvalue) 
{
	o_pub = new RosMatrixPublisher(1);
}

iMMatrix::iMMatrix( MatrixXd const *  i, unsigned int rows , unsigned int cols, double dvalue ) : iMatrix(i), orows(rows), ocols(cols), dvalue(dvalue) 
{
	o_pub = new RosMatrixPublisher(1);
}

iMMatrix::~iMMatrix()
{
        if( o_pub != NULL )
        {
                if( o_pub->is_open() ) o_pub->close();
                delete(o_pub);
        }
}

void iMMatrix::resizeWeight()
{
	if( getIRows()==0 || getICols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing input size");
	if( getORows()==0 || getOCols()==0) throw std::invalid_argument("iLink : Matrix try to resize weight without knowing output size");

	weight.resize(  getORows() * getOCols() , getIRows() * getICols() );
}

void iMMatrix::initWeight(double w)
{
	weight << MatrixXd::Constant( weight.rows() , weight.cols(),w);
}

bool iMMatrix::checkWeightSize(unsigned int rows, unsigned int cols )
{
	if( weight.rows() == rows  && weight.cols() == cols) return true;
	else return false;

}

double iMMatrix::w(unsigned int rows, unsigned int cols)
{
	if(rows > getORows() || cols > getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");
	return weight(rows,cols);
}

/********************************************************************************************************/
/******************                   iDenseMatrix Section                           *******************/
/********************************************************************************************************/

void iDenseMatrix::w(VectorXd &weight,unsigned int col)
{
	if(col > getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");
	this->weight.col(col) = weight;
}


void iDenseMatrix::w(const MatrixXd &weight)
{
	if(weight.rows() != getORows() || weight.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");
	this->weight = weight;
}


void iDenseMatrix::w(double weight, unsigned int rows, unsigned int cols)
{
	if(rows != getORows() || cols != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	this->weight(rows,cols) = weight;
}


MatrixXd& iDenseMatrix::add(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out = weight + ve;

	return out;
}

MatrixXd& iDenseMatrix::diff(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out = weight - ve;

	return out;
}

MatrixXd& iDenseMatrix::prod(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out = weight.cwiseProduct(ve);

	return out;
}

MatrixXd& iDenseMatrix::quot(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out = weight.cwiseQuotient(ve);

	return out;
}

MatrixXd& iDenseMatrix::weigthedSum(MatrixXd& out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : DenseMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs = weight * ve;

	return out;
}

MatrixXd& iDenseMatrix::weigthedSumAccu(MatrixXd& out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : DenseMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs += weight * ve;

	return out;
}


/********************************************************************************************************/
/******************                   iSparseMatrix Section                           *******************/
/********************************************************************************************************/

void iSparseMatrix::w(VectorXd &weight,unsigned int col)
{
	if(col > getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");
	this->weight.col(col) = weight ; 
	this->weight.col(col) =  this->weight.col(col) * filter.col(col);
}


void iSparseMatrix::w(const MatrixXd &weight)
{
	if(weight.rows() != getORows() || weight.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");
	this->weight = weight.cwiseProduct(filter);
}


void iSparseMatrix::w(double weight, unsigned int rows, unsigned int cols)
{
	if(rows != getORows() || cols != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	this->weight(rows,cols) = weight * filter.coeffRef(rows,cols);  //filter(row,col);
}

MatrixXd& iSparseMatrix::add(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out =  filter.cwiseProduct(weight + ve);

	return out;
}

MatrixXd& iSparseMatrix::diff(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out =  filter.cwiseProduct(weight - ve);

	return out;
}

MatrixXd& iSparseMatrix::prod(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out =  filter.cwiseProduct(weight.cwiseProduct(ve));

	return out;
}

MatrixXd& iSparseMatrix::quot(MatrixXd& out)
{
	if(  out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : Matrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;

	out =  filter.cwiseProduct(weight.cwiseQuotient(ve));

	return out;
}

MatrixXd& iSparseMatrix::weigthedSum(MatrixXd& out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : DenseMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs =  weight.cwiseProduct(filter) * ve;

	return out;
}

MatrixXd& iSparseMatrix::weigthedSumAccu(MatrixXd& out)
{
	if(out.rows() != getORows() || out.cols() != getOCols()) throw std::invalid_argument("iLink : DenseMatrix output size doesn't match with weight size");

	Map<const MatrixXd> ve( i().data(),1, getIRows()* getICols() ) ;
	Map<MatrixXd> vs(out.data(), out.cols()*out.rows(),1) ;

	vs += weight.cwiseProduct(filter) * ve;

	return out;
}

/*
value compute_val()


for(int i = 0;  i < w.cols(); i++)
{
s(i) = (ve + w.col(i)).sum() ;
}

std::cout << s << std::endl;

s.colwise().sum();
*/

