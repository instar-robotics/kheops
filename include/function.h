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

#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include <Eigen/Dense>
#include <string>

#include "factory.h"
#include "input.h"
#include "anchor.h"

#define REGISTER_FUNCTION(classname) \
   static const BuilderImpl<classname,Function> classname(#classname); 


class Function
{
	private : 
		std::string uuid;

	public : 
		Function(){}
		virtual ~Function(){}

                virtual void compute() = 0;
		virtual void setparameters() = 0;
		virtual size_t type() = 0;
		//virtual void setValue(double dvalue, int x,int y) = 0;
		//virtual void setSize(int x,int y) = 0;
		virtual int getRows()=0;
		virtual int getCols()=0;
		
		inline const std::string& getUuid() { return uuid;  }
		inline void setUuid(const std::string& uuid  ) { this->uuid = uuid;}
};

template<class T>
class FTemplate : public Function
{
	protected : 

		T output;

	public : 

		FTemplate(){}
		virtual ~FTemplate(){}
                
		virtual void compute() = 0;
		virtual void setparameters() = 0;
		virtual size_t type(){ return typeid(T).hash_code();}
	
		inline const T& getOutput() const { return output;}
                operator  T& () { return output; }
                
		inline const T& operator()() const
                {
                        return output;
                }
};

class FMatrix : public FTemplate<MatrixXd>
{
	public : 
		FMatrix() : FTemplate()  {}
		FMatrix( int X,int Y) : FTemplate()  {
                        output = MatrixXd::Constant( X , Y ,0);
                }
                FMatrix(int X,int Y, double dvalue) : FTemplate() {
                        output = MatrixXd::Constant( X , Y ,dvalue);
                }
		virtual ~FMatrix() {}

                virtual void compute() = 0;
		virtual void setparameters() = 0;
		inline virtual int getRows(){return output.rows();}
		inline virtual int getCols(){return output.cols();}

		inline virtual void setValue(double dvalue, int row,int col) { output = MatrixXd::Constant(row,col,dvalue);}
		inline virtual void setSize(int rows, int cols){ output = MatrixXd::Constant( rows , cols ,0); }

};


class FScalar : public FTemplate<double>
{
	public : 
		FScalar() : FTemplate() {
                        output = 0;
                }
                FScalar( double dvalue) : FTemplate() {
                        output = dvalue;
                }
		virtual ~FScalar() {}
                
		virtual void compute() = 0;
		virtual void setparameters() = 0;

		inline void setValue(double dvalue){ output = dvalue; }
	//	inline virtual void setSize(int x=1, int y=1){}
	//	inline virtual void setValue(double dvalue, int x =1 ,int y =1){ output = dvalue; }
		inline virtual int getRows(){return 1;}
		inline virtual int getCols(){return 1;}
};

#endif  // __FUNCTION_H__
