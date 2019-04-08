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


#ifndef __IMATRIX_H__
#define __IMATRIX_H__

#include "kheops/kernel/ilinkbase.h"
#include "kheops/kernel/inputbase.h"
#include "kheops/iostream/xmlconverter.h"

using Eigen::Ref;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;
using Eigen::EigenBase;

using namespace Eigen;

template<class W>
class iMatrix : public iLink<MatrixXd,W>
{
        public :

                iMatrix() : iLink<MatrixXd,W>() {}
                iMatrix(MatrixXd const * i) : iLink<MatrixXd,W>(i){}
                virtual ~iMatrix() {}

                inline double operator()(int x,int y) const {return (*iLink<MatrixXd,W>::input)(x,y);}

                inline auto iRows(){return (*iLink<MatrixXd,W>::input).rows();}
                inline auto iCols(){return (*iLink<MatrixXd,W>::input).cols();}
                inline auto iSize(){return (*iLink<MatrixXd,W>::input).size();}

		inline bool isPoint(){return iSize() == 1;}
                inline bool isVect(){ return iRows() == 1 || iCols() == 1;}
                inline bool isRowVect(){ return iRows() == 1 && iCols() > 1;}
                inline bool isColVect(){ return iRows() > 1 && iCols() == 1;}
};


/*******************************************************************************************************/
/*                                              SCALAR_MATRIX Link				       */
/*******************************************************************************************************/

class iScalarMatrix : public iMatrix<double>
{
        public :
                iScalarMatrix();
                iScalarMatrix( MatrixXd const *  i);
                virtual ~iScalarMatrix();

                Ref<MatrixXd> operator()(Ref<MatrixXd> res){return accumulate(res);}
                auto operator()(){return (*input) * weight;}

                virtual Ref<MatrixXd> accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> mul_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> sum_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> sub_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> div_accumulate(Ref<MatrixXd> res);

                friend Ref<MatrixXd> operator+=(Ref<MatrixXd> res,iScalarMatrix& val){return val.sum_accumulate(res);}
                friend Ref<MatrixXd> operator-=(Ref<MatrixXd> res,iScalarMatrix& val){return val.sub_accumulate(res);}
                friend Ref<MatrixXd> operator/=(Ref<MatrixXd> res,iScalarMatrix& val){return val.div_accumulate(res);}
                friend Ref<MatrixXd> operator*=(Ref<MatrixXd> res,iScalarMatrix& val){return val.mul_accumulate(res);}
};

typedef Input<iScalarMatrix> ISMInput;

/*******************************************************************************************************/
/*                                              MATRIX_MATRIX Link
********************************************************************************************************/

// Connectivity Rules :
const std::string one_to_one = "ONE_TO_ONE";
const std::string one_to_all = "ONE_TO_ALL";
const std::string one_to_nei = "ONE_TO_NEI";

/* Return Map in a one row Matrix  */
Map<MatrixXd> getMapRow(MatrixXd & m);

/* Return Map in a one row Matrix  */
Map<MatrixXd> getMapCol(MatrixXd & m);

/* Return Map in a one row Matrix  */
Map<VectorXd> getMapVect(MatrixXd & m);

/* Return Const Map in a one row Matrix  */
Map<const MatrixXd> getCMapRow(const MatrixXd & m);

/* Return Map in a one row Matrix  */
Map<const MatrixXd> getCMapCol(const MatrixXd & m);

/* Return Map in a one row Matrix  */
Map<const VectorXd> getCMapVect(const MatrixXd & m);

typedef Matrix<uint8_t , Dynamic, Dynamic> MatrixXb;
typedef Matrix<uint8_t, Dynamic,1> VectorXb;
typedef Matrix<uint8_t, 1,Dynamic> RVectorXb;


class iMMatrix : public iMatrix<MatrixXd>
{
        protected :

                MatrixXb filter;
                unsigned int oRow;
                unsigned int oCol;

        public :
                iMMatrix(unsigned int oRow=0, unsigned int oCol=0);
                iMMatrix( MatrixXd const *  i, unsigned int oRow = 0, unsigned int oCol=0);
                iMMatrix( MatrixXd const *  i, unsigned int oRow , unsigned int oCol, double value);
                virtual ~iMMatrix();


                /***********************************************************************/
                /*****************************  Output API  ****************************/
                /***********************************************************************/
                inline unsigned int oRows(){return oRow;}
                inline unsigned int oCols(){return oCol;}
                inline unsigned int oSize(){return oRow * oCol;}

                /***********************************************************************/
                /************************  Weight Management API  **********************/
                /***********************************************************************/
                virtual void initWeight(double weight);
                virtual void resizeWeight();
                virtual bool checkWeightSize(unsigned int rows, unsigned int cols );

                inline unsigned int wRows(){return weight.rows();}
                inline unsigned int wCols(){return weight.cols();}
                inline unsigned int wSize(){return weight.size();}

                inline unsigned int getInitWRows(){return iRows() * iCols();}
                inline unsigned int getInitWCols(){return oRows() * oCols();}
                inline unsigned int getInitWSize(){return oSize() * iSize();}

                /***********************************************************************/
                /*************************  Weight Access API  *************************/
                /***********************************************************************/

                /*
                 *      iMMatrix link have MatrixXd& w() and w(const MatrixXd&) API
                 *      This functions could be use but :
                 *      1- The function could be not works with all Eigen function
                 *      2- Dont't use Lazy evaluation
                 *      Better to use : wm or wref functions
                 */
                virtual void w(const MatrixXd& w);
                virtual MatrixXd& w() {return weight;}

                // Use to replace w(const MatrixXd& ) to use Eigen::Ref
                // Eigen::Ref gives more generalization abilities and better performance
                void wref(const Ref<const MatrixXd>& weight);

                // Return the map of the weight Matrix
                // In user functions, use like this :
                //
                //  iMMatrix imm;
                //
                //  Map<MatrixXd> map = imm.wm();
                //
                //  ... (do things)
                //
                Map<MatrixXd> wm();

                // Get Weight matrix for the output neuron (oRows,oCols)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj(unsigned int oRows,unsigned int oCols);
                
		// Get Weight matrix for the output neuron (o)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj(unsigned int o);

                // Get Weight row for the output neuron (oRows,oCols)
                // The Matrix have (1,wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj_row(unsigned int oRows, unsigned int oCols);

                // Get Weight row for the output neuron (o)
                // The Matrix have (1, wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj_row(unsigned int o);
                
		// Get Weight colomn for the output neuron (oRows,oCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj_col(unsigned int oRows, unsigned int oCols);

                // Get Weight colomn for the output neuron (o)
                // The Matrix have (wRows, 1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj_col(unsigned int o);

                // Get Weight vector for the output neuron (oRows,oCols)
                // Becareful : the returned Map is writable !
                Map<VectorXd> wj_vect(unsigned int wRows, unsigned int wCols);

                // Get Weight vector for the output neuron (o)
                // Becareful : the returned Map is writable !
                Map<VectorXd> wj_vect(unsigned int o);

                double wij(unsigned int wRows,unsigned int wCols);

                //Set Weight Value
                void wij(double weight, unsigned int wRow, unsigned int wCol);
                void wj(const Ref<VectorXd> &weight,unsigned int wCol);
                /***********************************************************************/
                /*****************************  Input API  *****************************/
                /***********************************************************************/

                // Return a const map of the input Matrix in row form
                Map<const MatrixXd> irow();

                // Return a const map of the input Matrix in colon form
                Map<const MatrixXd> icol();

                // Return a const map of the input Matrix in vector form
                Map<const VectorXd> ivec();

                /***********************************************************************/
                /****************************  Filter API  *****************************/
                /***********************************************************************/
                inline unsigned int getFRows(){return filter.rows();}
                inline unsigned int getFCols(){return filter.cols();}

                MatrixXb& f() { return filter;}
                void f(const MatrixXb &filter);
                
		Map<MatrixXb> fm();

                // Use Eigen::Ref
                // Eigen::Ref gives more generalization abilities and better performance
                void fref(const Ref<const MatrixXb>& filter);

                //Set Weight Value
                void fij(typename MatrixXb::Scalar weight, unsigned int fRow, unsigned int fCol);
                void fj(const Ref<VectorXb> &weight,unsigned int fCol);

                // Get Filter matrix for the output neuron (oRows,oCols)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj(unsigned int oRows,unsigned int oCols);

                // Get Filter matrix for the output neuron (o)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj(unsigned int o);
                
		// Get Filter row vector for the output neuron (oRows,oCols)
                // The Matrix have (1,wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj_row(unsigned int oRows,unsigned int oCols);

                // Get Filter row vector for the output neuron (o)
                // The Matrix have (1,wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj_row(unsigned int o);
                
		// Get Filter colmn vector for the output neuron (oRows,oCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj_col(unsigned int oRows,unsigned int oCols);

                // Get Filter colomn vector for the output neuron (o)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXb> fj_col(unsigned int o);

                // Get Filter vector for the output neuron (oRows,oCols)
                // Becareful : the returned Map is writable !
                Map<VectorXb> fj_vec(unsigned int oRows,unsigned int oCols);
                
		// Get Filter colomns for the output neuron (wCols)
                // Becareful : the returned Map is writable !
                Map<VectorXb> fj_vec(unsigned int wCols);

                double fij(unsigned int wRows,unsigned int wCols);

                void buildFilter(const XConnectivity& con );
};

typedef Input<iMMatrix> IMMInput;


/*******************************************************************************************************/
/*                                              FILTER OPERATOR
********************************************************************************************************/


// Functor definition : apply unitary expression
template<class ArgType, class FilterType>
class filter_functor {
  const ArgType &m_arg;
  const FilterType &m_filter;
public:
  typedef Matrix<typename ArgType::Scalar,
                 ArgType::SizeAtCompileTime,
                 ArgType::SizeAtCompileTime,
                 ArgType::Flags&RowMajorBit?RowMajor:ColMajor,
                 ArgType::MaxSizeAtCompileTime,
                 ArgType::MaxSizeAtCompileTime> MatrixType;

  filter_functor(const ArgType& arg, const FilterType& filter)
    : m_arg(arg), m_filter(filter)
  {}
  typename ArgType::Scalar operator() (Index row, Index col) const {

          return m_filter(row,col) ? m_arg(row,col) : 0.0 ;
  }
};

// Filter Function 
template <class ArgType, class FilterType>
CwiseNullaryOp<filter_functor<ArgType,FilterType>, typename filter_functor<ArgType,FilterType>::MatrixType>
filter(const MatrixBase<ArgType>& arg, const FilterType& filter)
{
  typedef filter_functor<ArgType,FilterType> Func;
  typedef typename Func::MatrixType MatrixType;
  return MatrixType::NullaryExpr(arg.rows(), arg.cols(), Func(arg.derived(), filter));
}

#endif // __IMATRIX_H__
