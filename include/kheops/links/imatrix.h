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
using Eigen::Map;
using Eigen::EigenBase;

using namespace Eigen;

template<class W>
class iMatrix : public iLink<MATRIX,W>
{
        public :

                iMatrix() : iLink<MATRIX,W>() {}
                iMatrix(MATRIX const * i) : iLink<MATRIX,W>(i){}
                virtual ~iMatrix() {}

                inline SCALAR operator()(int x,int y) const {return (*iLink<MATRIX,W>::input)(x,y);}

                inline auto iRows(){return (*iLink<MATRIX,W>::input).rows();}
                inline auto iCols(){return (*iLink<MATRIX,W>::input).cols();}
                inline auto iSize(){return (*iLink<MATRIX,W>::input).size();}

		inline bool isPoint(){return iSize() == 1;}
                inline bool isVect(){ return iRows() == 1 || iCols() == 1;}
                inline bool isRowVect(){ return iRows() == 1 && iCols() > 1;}
                inline bool isColVect(){ return iRows() > 1 && iCols() == 1;}
};


/*******************************************************************************************************/
/*                                              SCALAR_MATRIX Link				       */
/*******************************************************************************************************/

class iScalarMatrix : public iMatrix<SCALAR>
{
        public :
                iScalarMatrix();
                iScalarMatrix( MATRIX const *  i);
                virtual ~iScalarMatrix();

                Ref<MATRIX> operator()(Ref<MATRIX> res){return accumulate(res);}
                auto operator()(){return (*input) * weight;}

                virtual Ref<MATRIX> accumulate(Ref<MATRIX> res);
                virtual Ref<MATRIX> mul_accumulate(Ref<MATRIX> res);
                virtual Ref<MATRIX> sum_accumulate(Ref<MATRIX> res);
                virtual Ref<MATRIX> sub_accumulate(Ref<MATRIX> res);
                virtual Ref<MATRIX> div_accumulate(Ref<MATRIX> res);

                friend Ref<MATRIX> operator+=(Ref<MATRIX> res,iScalarMatrix& val){return val.sum_accumulate(res);}
                friend Ref<MATRIX> operator-=(Ref<MATRIX> res,iScalarMatrix& val){return val.sub_accumulate(res);}
                friend Ref<MATRIX> operator/=(Ref<MATRIX> res,iScalarMatrix& val){return val.div_accumulate(res);}
                friend Ref<MATRIX> operator*=(Ref<MATRIX> res,iScalarMatrix& val){return val.mul_accumulate(res);}
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
Map<MATRIX> getMapRow(MATRIX & m);

/* Return Map in a one row Matrix  */
Map<MATRIX> getMapCol(MATRIX & m);

/* Return Map in a one row Matrix  */
Map<VectorXs> getMapVect(MATRIX & m);

/* Return Const Map in a one row Matrix  */
Map<const MATRIX> getCMapRow(const MATRIX & m);

/* Return Map in a one row Matrix  */
Map<const MATRIX> getCMapCol(const MATRIX & m);

/* Return Map in a one row Matrix  */
Map<const VectorXs> getCMapVect(const MATRIX & m);

typedef Matrix<uint8_t , Dynamic, Dynamic> MatrixXb;
typedef Matrix<uint8_t, Dynamic,1> VectorXb;
typedef Matrix<uint8_t, 1,Dynamic> RVectorXb;


class iMMatrix : public iMatrix<MATRIX>
{
        protected :

                MatrixXb filter;
                unsigned int oRow;
                unsigned int oCol;

        public :
                iMMatrix(unsigned int oRow=0, unsigned int oCol=0);
                iMMatrix( MATRIX const *  i, unsigned int oRow = 0, unsigned int oCol=0);
                iMMatrix( MATRIX const *  i, unsigned int oRow , unsigned int oCol, SCALAR value);
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
                virtual void initWeight(SCALAR weight);
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
                 *      iMMatrix link have MATRIX& w() and w(const MATRIX&) API
                 *      This functions could be use but :
                 *      1- The function could be not works with all Eigen function
                 *      2- Dont't use Lazy evaluation
                 *      Better to use : wm or wref functions
                 */
                virtual void w(const MATRIX& w);
                virtual MATRIX& w() {return weight;}

                // Use to replace w(const MATRIX& ) to use Eigen::Ref
                // Eigen::Ref gives more generalization abilities and better performance
                void wref(const Ref<const MATRIX>& weight);

                // Return the map of the weight Matrix
                // In user functions, use like this :
                //
                //  iMMatrix imm;
                //
                //  Map<MATRIX> map = imm.wm();
                //
                //  ... (do things)
                //
                Map<MATRIX> wm();

                // Get Weight matrix for the output neuron (oRows,oCols)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj(unsigned int oRows,unsigned int oCols);
                
		// Get Weight matrix for the output neuron (o)
                // The Matrix have (iRows,iCols) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj(unsigned int o);

                // Get Weight row for the output neuron (oRows,oCols)
                // The Matrix have (1,wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj_row(unsigned int oRows, unsigned int oCols);

                // Get Weight row for the output neuron (o)
                // The Matrix have (1, wCols) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj_row(unsigned int o);
                
		// Get Weight colomn for the output neuron (oRows,oCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj_col(unsigned int oRows, unsigned int oCols);

                // Get Weight colomn for the output neuron (o)
                // The Matrix have (wRows, 1) dimension
                // Becareful : the returned Map is writable !
                Map<MATRIX> wj_col(unsigned int o);

                // Get Weight vector for the output neuron (oRows,oCols)
                // Becareful : the returned Map is writable !
                Map<VectorXs> wj_vect(unsigned int wRows, unsigned int wCols);

                // Get Weight vector for the output neuron (o)
                // Becareful : the returned Map is writable !
                Map<VectorXs> wj_vect(unsigned int o);

                SCALAR wij(unsigned int wRows,unsigned int wCols);

                //Set Weight Value
                void wij(SCALAR weight, unsigned int wRow, unsigned int wCol);
                void wj(const Ref<VectorXs> &weight,unsigned int wCol);
                /***********************************************************************/
                /*****************************  Input API  *****************************/
                /***********************************************************************/

                // Return a const map of the input Matrix in row form
                Map<const MATRIX> irow();

                // Return a const map of the input Matrix in colon form
                Map<const MATRIX> icol();

                // Return a const map of the input Matrix in vector form
                Map<const VectorXs> ivec();

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

                typename MatrixXb::Scalar fij(unsigned int wRows,unsigned int wCols);

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
