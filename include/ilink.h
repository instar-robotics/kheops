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


using Eigen::Ref;
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


class iScalar : public iLink<double,double>
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
		
		friend double& operator+=(double& res, iScalar& val) {return val.sum_accumulate(res);}
		friend double& operator-=(double& res, iScalar& val) {return val.sub_accumulate(res);}
		friend double& operator/=(double& res, iScalar& val) {return val.div_accumulate(res);}
		friend double& operator*=(double& res, iScalar& val) {return val.sub_accumulate(res);}
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

/**************************************************************************************************************/
/* 						MATRIX_MATRIX Link
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

		inline void setOSize(unsigned int oRows, unsigned int oCols){ oRows = oRows; oCols=oCols;}

		/***********************************************************************/
		/************************  Weight Management API  **********************/
		/***********************************************************************/
		virtual void initWeight(double weight);
		virtual void resizeWeight(); 
		virtual bool checkWeightSize(unsigned int rows, unsigned int cols );
		
		inline unsigned int getWRows(){return weight.rows();}
		inline unsigned int getWCols(){return weight.cols();}
		
		inline unsigned int getInitWRows(){return getIRows() * getICols();}
		inline unsigned int getInitWCols(){return getORows() * getOCols();}
	
		/***********************************************************************/
		/*************************  Weight Access API  *************************/
		/***********************************************************************/

		/*
		 * 	iMMatrix link have MatrixXd& w() and w(const MatrixXd&) API
		 *	This functions could be use but :
		 *	1- The function could be not works with all Eigen function 
		 *	2- Dont't use Lazy evaluation
		 *	Better to use : wm or wref functions
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

#endif // __ILINK_H__
