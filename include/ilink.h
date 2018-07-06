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

#ifndef __ILINK_H__
#define __ILINK_H__

#include "publisher.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>

using Eigen::SparseMatrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;

class iLinkBase
{
	protected : 
		
		std::string uuid;

	public : 
		iLinkBase(){}
		virtual ~iLinkBase() = 0;
		
		inline const std::string& getUuid() { return uuid; }
                virtual void setUuid(const std::string& uuid) = 0;
		
		/***********************************************************************/
		/*****************************  Buffer API *****************************/
		/***********************************************************************/
		
		virtual void activateBuffer() = 0;
		virtual void deactivateBuffer() = 0;
		virtual void copyBuffer() = 0;

		/***********************************************************************/
		/************************  Weight publish API  *************************/
		/***********************************************************************/
	        
		virtual bool is_publish_active() = 0;
                virtual void active_publish(bool state) = 0;
                virtual void publish_message() = 0;

		/***********************************************************************/
                /************************  Input Accessor API  *************************/
                /***********************************************************************/

                virtual size_t type() = 0;
                virtual std::string type_name() = 0;
};


template<class I, class W>
class iLink : public iLinkBase
{
	protected :
		I cvalue;
		I const * input;
		I const * b_input;
		bool buffer;
	
		W weight;
		DataPublisher<W>* o_pub;

	public : 
		iLink() : input(NULL), b_input(NULL), buffer(false){}
		iLink(I const * i) : input(i), b_input(NULL),buffer(false) {}
		virtual ~iLink(){}
	

                inline virtual void setUuid(const std::string& uuid)
		{
			this->uuid = uuid;
        		o_pub->setPubName("ilink_"+getUuid());
		}

		/***********************************************************************/
		/*************************  Constant Value API *************************/
		/***********************************************************************/

		virtual inline void setCValue(const I& cv){cvalue = cv; input = &cvalue;}
		virtual inline const I& getCValue(){ return cvalue;}
		virtual inline bool isSet(){return input!=NULL;}
		virtual inline bool isCValue(){return input==&cvalue;}


		/***********************************************************************/
		/*****************************  Buffer API *****************************/
		/***********************************************************************/

		virtual inline bool isBuffer(){return buffer;}
		virtual void activateBuffer()
		{
			buffer = true;
			cvalue = *input;
			b_input = input;
			input = &cvalue;
		}
		
		virtual void deactivateBuffer()
		{
			buffer = false;
			input = b_input;
			b_input = NULL;
		}

		virtual void copyBuffer()
		{
			if( buffer )
			{
				cvalue = *b_input;
			}
		}
		
		/***********************************************************************/
		/************************  Weight publish API  *************************/
		/***********************************************************************/
		
		inline virtual  bool is_publish_active(){return o_pub->is_open();}

                virtual void active_publish(bool state)
		{
			if( state )
			{
				if( o_pub != NULL  )
				{
					if( !o_pub->is_open() )  o_pub->open();
				}
				else throw std::invalid_argument("Weight : failed to open output publisher");
			}
			else
			{
				if( o_pub != NULL)
				{
					if( o_pub->is_open()) o_pub->close();
				}
			}
		}

		virtual void publish_message()
		{
			if( o_pub->is_open() )
			{
				o_pub->setMessage(weight);
				o_pub->publish();
			}
		}
		
		/***********************************************************************/
		/************************  Input Accessor API  *************************/
		/***********************************************************************/

		typedef I type_i;
		virtual size_t type() { return typeid(I).hash_code();}
		virtual std::string type_name() { return typeid(I).name();}

		virtual inline void i(I const *i){input=i;}
		virtual inline const I& i() const {return *input;}
		
		/***********************************************************************/
		/************************  Weight Accessor API  *************************/
		/***********************************************************************/

		typedef W type_w;
		virtual size_t w_type() { return typeid(W).hash_code();}
		virtual std::string w_type_name() { return typeid(W).name();}
		
		inline virtual W& w() {return weight;}
                inline virtual void w(const W& w) { weight = w; }
};


template<class I>
class iCombinator
{
	public : 	

		iCombinator(){}
		virtual ~iCombinator() {}
		
		virtual I& accumulate(I&)=0;
		virtual I& mul_accumulate(I&)=0;
		virtual I& sum_accumulate(I&)=0;
		virtual I& sub_accumulate(I&)=0;
		virtual I& div_accumulate(I&)=0;
		
		virtual I operator()() = 0;
		
		friend I& operator+=(I& res, iCombinator<I>& val) {return val.sum_accumulate(res);}
		friend I& operator-=(I& res, iCombinator<I>& val) {return val.sub_accumulate(res);}
		friend I& operator/=(I& res, iCombinator<I>& val) {return val.div_accumulate(res);}
		friend I& operator*=(I& res, iCombinator<I>& val) {return val.sub_accumulate(res);}
};

		
class iScalar : public iLink<double,double>, public iCombinator<double>
{
        public:
		
		iScalar();
		iScalar(double const * i);
		virtual ~iScalar();
                
		virtual double operator()(){return (*input) * weight;}

		inline virtual double& accumulate(double& res){return res = (*input) * weight;}
		inline virtual double& mul_accumulate(double& res){return res *= (*input) * weight;}
		inline virtual double& sum_accumulate(double& res){return res += (*input) * weight;}
		inline virtual double& sub_accumulate(double& res){return res -= (*input) * weight;}
		inline virtual double& div_accumulate(double& res){return res /= (*input) * weight;}
		
		//TODO : add operator for Matrix and iScalar ? 
		// Perhaps not a good idee ... 
		// Force developpers to use ()() operator to get a double and to use classical Eigen function...
};


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
};

class iScalarMatrix : public iMatrix<double>, public iCombinator<MatrixXd>
{
	public : 

		iScalarMatrix();
                iScalarMatrix( MatrixXd const *  i);
                virtual ~iScalarMatrix();
		
		MatrixXd& operator()(MatrixXd& res){return accumulate(res);}
		virtual MatrixXd operator()(){return (*input) * weight;}
               
		virtual MatrixXd& accumulate(MatrixXd& res);
		virtual MatrixXd& mul_accumulate(MatrixXd& res);
		virtual MatrixXd& sum_accumulate(MatrixXd& res);
		virtual MatrixXd& sub_accumulate(MatrixXd& res); 
		virtual MatrixXd& div_accumulate(MatrixXd& res);

};

/**************************************************************************************************************/
/* 						MATRIX_MATRIX Link
***************************************************************************************************************/

// Connectivity Rules :
const std::string one_to_one = "ONE_TO_ONE";
const std::string one_to_all = "ONE_TO_ALL";
const std::string one_to_nei = "ONE_TO_NEI";


class iMMatrix : public iMatrix<MatrixXd>
{
	protected : 

		unsigned int orows;	
		unsigned int ocols;
		double dvalue;
		
	public : 
		iMMatrix(unsigned int rows=0, unsigned int cols=0, double dvalue=0.);
                iMMatrix( MatrixXd const *  i, unsigned int rows = 0, unsigned int cols=0, double dvalue =0 );
                virtual ~iMMatrix();
		
		inline void setOSize(unsigned int rows, unsigned int cols){ orows = rows; ocols=cols;}
		inline void setDValue(double dvalue){this->dvalue=dvalue;}

		inline unsigned int getORows(){return ocols;}
		inline unsigned int getOCols(){return orows;}

		inline unsigned int getWRows(){return weight.rows();}
		inline unsigned int getWCols(){return weight.cols();}

		virtual void initWeight(double weight);
		virtual void resizeWeight(); 
		virtual bool checkWeightSize(unsigned int rows, unsigned int cols );
		
		// out is an Matrix with output matrix size
		virtual MatrixXd& weigthedSum(MatrixXd& out) = 0;
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out) = 0;
			
		inline virtual MatrixXd& w() {return  iMatrix<MatrixXd>::w(); }
		virtual double w(unsigned int rows, unsigned int cols);
		virtual void w(double weight, unsigned int row, unsigned int col) = 0;
		virtual void w(VectorXd &weight,unsigned int col) = 0;

		// wout = weight op input
		// here wout is an Matrix with the size of the weighted matrix  		
		virtual MatrixXd& add(MatrixXd& wout) = 0;
		virtual MatrixXd& diff(MatrixXd& wout) = 0;
		virtual MatrixXd& prod(MatrixXd& wout) = 0;
		virtual MatrixXd& quot(MatrixXd& wout) = 0;
};

class iDenseMatrix : public iMMatrix
{
        public:

		iDenseMatrix(unsigned int rows=0, unsigned int cols=0, double dvalue=0.) : iMMatrix(rows,cols,dvalue){}
                iDenseMatrix( MatrixXd const *  i,  unsigned int rows = 0, unsigned int cols=0, double dvalue = 0 ) : iMMatrix(i,rows, cols, dvalue) {}

                virtual ~iDenseMatrix(){}

		virtual void w(VectorXd &weight,unsigned int col);
		virtual void w(const MatrixXd &weight);
		virtual void w(double weight, unsigned int rows, unsigned int cols);
		
		virtual MatrixXd& add(MatrixXd& wout);
		virtual MatrixXd& diff(MatrixXd& wout);
		virtual MatrixXd& prod(MatrixXd& wout);
		virtual MatrixXd& quot(MatrixXd& wout);

		virtual MatrixXd& weigthedSum(MatrixXd& out);
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out);
};

// SparseMatrix use a Eigen SparseMatrix to filter connectivity
// Two options to apply the filter :
// 1- When acces to weight
// 2- When modify weight value
// 
// We choose option 2 : because, with this solution, we don't have to reimplement accessor and publish
// This is not sur, that 2 is the better solution. 
// Perhaps, we update weight more often that acces to the weight ... 
class iSparseMatrix : public iMMatrix
{
	private : 	
	
		SparseMatrix<double> filter;
		int type;
        
	public:
		iSparseMatrix(unsigned int row=0, unsigned int col=0, double dvalue=0.) : iMMatrix(row,col,dvalue) {}
                iSparseMatrix( MatrixXd const *  i, unsigned int row = 0, unsigned int col=0, double dvalue = 0 ) : iMMatrix(i,row, col, dvalue) {}
                virtual ~iSparseMatrix(){}
		
		virtual void w(VectorXd &weight,unsigned int col);
		virtual void w(const MatrixXd &weight);
		virtual void w(double weight, unsigned int rows, unsigned int cols);

		SparseMatrix<double> & f(){ return filter;}

		virtual MatrixXd& add(MatrixXd& wout);
		virtual MatrixXd& diff(MatrixXd& wout);
		virtual MatrixXd& prod(MatrixXd& wout);
		virtual MatrixXd& quot(MatrixXd& wout);

		virtual MatrixXd& weigthedSum(MatrixXd& out);
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out);
};

#endif // __ILINK_H__
