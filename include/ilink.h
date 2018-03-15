/*
Copyright Enacted Robotics

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

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>

enum OPERATOR{NONE=0,ADDITION=1,SUBSTRACTION=2,DIVISION=3,MULTIPLICATION=4};

using Eigen::SparseMatrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Map;

template<class T>
class iLink
{
	protected :
		T cvalue;
		T const * input;

		T const * b_input;
		bool buffer;

		std::string uuid;

	public : 

		typedef T type_i;
		size_t type() { return typeid(T).hash_code();}

		iLink() : buffer(false) {input = NULL; b_input = NULL;}	
		iLink(T const * i) : input(i),buffer(false) { b_input = NULL; }
		virtual ~iLink(){}
	
		inline const std::string& getUuid() { return uuid;  }
                inline void setUuid(const std::string& uuid  ) { this->uuid = uuid;}

		virtual inline void setCValue(const T& cv){cvalue = cv; input = &cvalue;}
		virtual inline const T& getCValue(){ return cvalue;}

		virtual inline void i(T const *i){input=i;}
		virtual inline const T& i() const {return *input;}

		virtual inline bool isSet(){return input!=NULL;}
		virtual inline bool isCValue(){return input==&cvalue;}

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
};


template<class T>
class ICombinator
{
	private :
		
		OPERATOR op;

	public : 	

		ICombinator( OPERATOR op = MULTIPLICATION ) : op(op)  {}
		virtual ~ICombinator() {}
		
		inline OPERATOR getOp() {return op;}
		virtual void setOp(OPERATOR OP){ op=OP;}
		virtual void setOp(std::string sOp)
		{
			if( sOp == "+" ) setOp(ADDITION);
        		else if( sOp == "*" ) setOp(MULTIPLICATION);
        		else if( sOp == "-" ) setOp(SUBSTRACTION);
        		else if( sOp == "/" ) setOp(DIVISION);
        		else throw  std::invalid_argument( "ICombinator : unkown operator : "+sOp+". Legal operators are +, *, / or -");
		}

		virtual T& accumulate(T&)=0;
		virtual T& mul_accumulate(T&)=0;
		virtual T& sum_accumulate(T&)=0;
		virtual T& sub_accumulate(T&)=0;
		virtual T& div_accumulate(T&)=0;
		
};



class IScalar : public iLink<double>, public ICombinator<double>
{
	private :

		double weight;

        public:
		
		IScalar(OPERATOR op = MULTIPLICATION) : iLink(),ICombinator(op) {setOp(op);}	
		IScalar(double const * i, OPERATOR op = MULTIPLICATION ) : iLink(i), ICombinator(op) {setOp(op);}
		~IScalar(){}

		inline double& w() {return weight;}
                inline void w(const double& w) { weight = w; }

		inline auto sum(){ return (*input) + weight; }
		inline auto sub(){ return (*input) - weight; }
		inline auto mul(){ return (*input) * weight; }
		inline auto div(){ return (*input) / weight; }
		
                std::function<double()> operate;
		double operator()(){return operate();}

		inline virtual void setOp(std::string sOp){ICombinator::setOp(sOp);}

		virtual void setOp(OPERATOR OP);

		inline virtual double& accumulate(double& res){return res = operate(); }
		inline virtual double& mul_accumulate(double& res){return res *= operate(); }
		inline virtual double& sum_accumulate(double& res){return res += operate(); }
		inline virtual double& sub_accumulate(double& res){return res -= operate(); }
		inline virtual double& div_accumulate(double& res){return res /= operate(); }

};


class IMatrix : public iLink<MatrixXd>
{
	public : 
                
		IMatrix() : iLink() {}
                IMatrix(MatrixXd const * i) : iLink(i){}
                virtual ~IMatrix() {}

		inline double operator()(int x,int y) const {return (*iLink<MatrixXd>::input)(x,y);}
		
		inline unsigned int getIRows(){return (*iLink<MatrixXd>::input).cols();}
		inline unsigned int getICols(){return (*iLink<MatrixXd>::input).rows();}
};

class IScalarMatrix : public IMatrix, public ICombinator<MatrixXd>
{
	private :
		double weight;
	public : 

		IScalarMatrix(OPERATOR op = MULTIPLICATION) : IMatrix(),ICombinator(op) {}
                IScalarMatrix( MatrixXd const *  i, OPERATOR op = MULTIPLICATION ) : IMatrix(i), ICombinator(op){}
                virtual ~IScalarMatrix() {}

		inline double& w() {return weight;}
                inline void w(const double& w) { weight = w; }
		
		inline virtual void setOp(std::string sOp){ICombinator::setOp(sOp);}
               
		inline auto sum(){ return (*input).array() + weight; }
		inline auto sub(){ return (*input).array() - weight; }
		inline auto mul(){ return (*input) * weight; }
		inline auto div(){ return (*input) / weight; }

		virtual MatrixXd& accumulate(MatrixXd& res);
		virtual MatrixXd& mul_accumulate(MatrixXd& res);
		virtual MatrixXd& sum_accumulate(MatrixXd& res);
		virtual MatrixXd& sub_accumulate(MatrixXd& res); 
		virtual MatrixXd& div_accumulate(MatrixXd& res);
};

class IMMatrix : public IMatrix
{
	protected : 

		unsigned int orows;	
		unsigned int ocols;
		double dvalue;
		
		MatrixXd weight;		
		
	public : 
		IMMatrix(unsigned int rows=0, unsigned int cols=0, double dvalue=0.) : IMatrix(), orows(rows),ocols(cols),dvalue(dvalue) {}
                IMMatrix( MatrixXd const *  i, unsigned int rows = 0, unsigned int cols=0, double dvalue =0 ) : IMatrix(i), orows(rows), ocols(cols), dvalue(dvalue) {}

                virtual ~IMMatrix(){}
		
		inline void setOSize(unsigned int rows, unsigned int cols){ orows = rows; ocols=cols;}
		inline void setDValue(double dvalue){this->dvalue=dvalue;}

		inline unsigned int getORows(){return ocols;}
		inline unsigned int getOCols(){return orows;}

		virtual void resizeWeight(); 
		
		// out is an Matrix with output matrix size
		virtual MatrixXd& weigthedSum(MatrixXd& out) = 0;
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out) = 0;
			
		virtual double w(unsigned int rows, unsigned int cols);
		virtual inline MatrixXd& w() {	return weight;}
		
		virtual void w(double weight, unsigned int row, unsigned int col) = 0;
		virtual void w(VectorXd &weight,unsigned int col) = 0;
		virtual void w(MatrixXd &weight) = 0;

		// wout = weight op input
		// here wout is an Matrix with weight matrix size 		
		virtual MatrixXd& add(MatrixXd& wout) = 0;
		virtual MatrixXd& diff(MatrixXd& wout) = 0;
		virtual MatrixXd& prod(MatrixXd& wout) = 0;
		virtual MatrixXd& quot(MatrixXd& wout) = 0;

};

class IDenseMatrix : public IMMatrix
{
        public:

		IDenseMatrix(unsigned int rows=0, unsigned int cols=0, double dvalue=0.) : IMMatrix(rows,cols,dvalue){}
                IDenseMatrix( MatrixXd const *  i,  unsigned int rows = 0, unsigned int cols=0, double dvalue = 0 ) : IMMatrix(i,rows, cols, dvalue) {}

                virtual ~IDenseMatrix(){}
		
		virtual void w(VectorXd &weight,unsigned int col);
		virtual void w(MatrixXd &weight);
		virtual void w(double weight, unsigned int rows, unsigned int cols);
		
		virtual MatrixXd& add(MatrixXd& wout);
		virtual MatrixXd& diff(MatrixXd& wout);
		virtual MatrixXd& prod(MatrixXd& wout);
		virtual MatrixXd& quot(MatrixXd& wout);

		virtual MatrixXd& weigthedSum(MatrixXd& out);
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out);
};

class ISparseMatrix : public IMMatrix
{
	private : 	
	
		SparseMatrix<double> filter;
		int type;
        
	public:
		ISparseMatrix(unsigned int row=0, unsigned int col=0, double dvalue=0.) : IMMatrix(row,col,dvalue) {}
                ISparseMatrix( MatrixXd const *  i, unsigned int row = 0, unsigned int col=0, double dvalue = 0 ) : IMMatrix(i,row, col, dvalue) {}
                virtual ~ISparseMatrix(){}
		
		virtual void w(VectorXd &weight,unsigned int col);
		virtual void w(MatrixXd &weight);
		virtual void w(double weight, unsigned int rows, unsigned int cols);

		virtual MatrixXd& add(MatrixXd& wout);
		virtual MatrixXd& diff(MatrixXd& wout);
		virtual MatrixXd& prod(MatrixXd& wout);
		virtual MatrixXd& quot(MatrixXd& wout);

		virtual MatrixXd& weigthedSum(MatrixXd& out);
		virtual MatrixXd& weigthedSumAccu(MatrixXd& out);
};

#endif // __ILINK_H__
