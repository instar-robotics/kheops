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

#ifndef __IMATRIX_H__
#define __IMATRIX_H__

#include "kernel/ilinkbase.h"
#include "kernel/inputbase.h"

using Eigen::Ref;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;


template<class W>
class iMatrix : public iLink<MatrixXd,W>
{
        public :

                iMatrix() : iLink<MatrixXd,W>() {}
                iMatrix(MatrixXd const * i) : iLink<MatrixXd,W>(i){}
                virtual ~iMatrix() {}

                inline double operator()(int x,int y) const {return (*iLink<MatrixXd,W>::input)(x,y);}

                inline unsigned int getIRows(){return (*iLink<MatrixXd,W>::input).cols();}
                inline unsigned int getICols(){return (*iLink<MatrixXd,W>::input).rows();}
                inline unsigned int getISize(){return (*iLink<MatrixXd,W>::input).size();}
};


/**************************************************************************************************************/
/*                                              SCALAR_MATRIX Link
***************************************************************************************************************/

class iScalarMatrix : public iMatrix<double>
{
        public :
                iScalarMatrix();
                iScalarMatrix( MatrixXd const *  i);
                virtual ~iScalarMatrix();

                Ref<MatrixXd> operator()(Ref<MatrixXd> res){return accumulate(res);}
                virtual MatrixXd operator()(){return (*input) * weight;}

                virtual Ref<MatrixXd> accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> mul_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> sum_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> sub_accumulate(Ref<MatrixXd> res);
                virtual Ref<MatrixXd> div_accumulate(Ref<MatrixXd> res);

                friend Ref<MatrixXd> operator+=(Ref<MatrixXd> res,iScalarMatrix& val){return val.sum_accumulate(res);}
                friend Ref<MatrixXd> operator-=(Ref<MatrixXd> res,iScalarMatrix& val){return val.sub_accumulate(res);}
                friend Ref<MatrixXd> operator/=(Ref<MatrixXd> res,iScalarMatrix& val){return val.div_accumulate(res);}
                friend Ref<MatrixXd> operator*=(Ref<MatrixXd> res,iScalarMatrix& val){return val.sub_accumulate(res);}
};

typedef Input<iScalarMatrix> ISMInput;

/**************************************************************************************************************/
/*                                              MATRIX_MATRIX Link
***************************************************************************************************************/

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


class iMMatrix : public iMatrix<MatrixXd>
{
        protected :

                MatrixXd filter;
                unsigned int oRows;
                unsigned int oCols;

        public :
                iMMatrix(unsigned int oRows=0, unsigned int oCols=0);
                iMMatrix( MatrixXd const *  i, unsigned int oRows = 0, unsigned int oCols=0);
                iMMatrix( MatrixXd const *  i, unsigned int oRows , unsigned int oCols, double value);
                virtual ~iMMatrix();


                /***********************************************************************/
                /*****************************  Output API  ****************************/
                /***********************************************************************/
                inline unsigned int getORows(){return oCols;}
                inline unsigned int getOCols(){return oRows;}
                inline unsigned int getOSize(){return oRows * oCols;}

                /***********************************************************************/
                /************************  Weight Management API  **********************/
                /***********************************************************************/
                virtual void initWeight(double weight);
                virtual void resizeWeight();
                virtual bool checkWeightSize(unsigned int rows, unsigned int cols );

                inline unsigned int getWRows(){return weight.rows();}
                inline unsigned int getWCols(){return weight.cols();}
                inline unsigned int getWSize(){return weight.size();}

                inline unsigned int getInitWRows(){return getIRows() * getICols();}
                inline unsigned int getInitWCols(){return getORows() * getOCols();}
                inline unsigned int getInitWSize(){return getOSize() * getISize();}

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
                // In user functions, use link this :
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

                // Get Weight colons for the output neuron (wCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<MatrixXd> wj(unsigned int wCols);

                // Get Weight colons for the output neuron (wCols)
                // Becareful : the returned Map is writable !
                Map<VectorXd> wj_vect(unsigned int wCols);

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

                // Basic accessor : just in case we want update Filter (add or delete connectivity at runtime)
                MatrixXd& f() { return filter;}
                void f(const MatrixXd &filter);

                // Use Eigen::Ref
                // Eigen::Ref gives more generalization abilities and better performance
                void fref(const Ref<const MatrixXd>& filter);

                //Set Weight Value
                void fij(double weight, unsigned int fRow, unsigned int fCol);
                void fj(const Ref<VectorXd> &weight,unsigned int fCol);

                // Return the map of the weight Matrix
                // In user functions, use link this :
                Map<const MatrixXd> fm();

                // Get filter matrix for the output neuron (oRows,oCols)
                // The Matrix have (iRows,iCols) dimension
                Map<const MatrixXd> fj(unsigned int oRows,unsigned int oCols);

                // Get Weight colons for the output neuron (wCols)
                // The Matrix have (wRows,1) dimension
                // Becareful : the returned Map is writable !
                Map<const MatrixXd> fj(unsigned int wCols);

                // Get Weight colons for the output neuron (wCols)
                // Becareful : the returned Map is writable !
                Map<const VectorXd> fj_vec(unsigned int wCols);

                double fij(unsigned int wRows,unsigned int wCols);

                void buildFilter(const std::string& con);

};

typedef Input<iMMatrix> IMMInput;

#endif // __IMATRIX_H__
