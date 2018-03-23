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

#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>

#include "factory.h"
#include "input.h"


#define REGISTER_FUNCTION(classname) \
   static const BuilderImpl<classname,Function> classname(#classname); 

class Function
{
	private : 
		std::string uuid;

		std::vector<ISInput*> is_input;
                std::vector<IMInput*> im_input;
                std::vector<ISMInput*> ism_input;
                std::vector<IMMInput*> imm_input;

	public : 
		Function(){}
		virtual ~Function();

                virtual void compute() = 0;
		virtual void setparameters() = 0;
		virtual size_t type() = 0;
		virtual std::string type_name() = 0;
		//virtual void setValue(double dvalue, int x,int y) = 0;
		//virtual void setSize(int x,int y) = 0;
		virtual int getRows()=0;
		virtual int getCols()=0;
		
		inline const std::string& getUuid() { return uuid;  }
		inline void setUuid(const std::string& uuid  ) { this->uuid = uuid;}

		inline void add_input(ISInput * is) { is_input.push_back(is);}
		inline void add_input(ISMInput * ism) { ism_input.push_back(ism);}
		inline void add_input(IMInput * im) { im_input.push_back(im);}
		inline void add_input(IMMInput * imm) { imm_input.push_back(imm);}

		inline std::vector<ISInput*>& get_isinput(){ return is_input;}
		inline std::vector<IMInput*>& get_iminput(){ return im_input;}
		inline std::vector<ISMInput*>& get_isminput(){ return ism_input;}
		inline std::vector<IMMInput*>& get_imminput(){ return imm_input;}

		virtual void nsync_afterCompute();
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
		std::string type_name() { return typeid(T).name();}
	
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
