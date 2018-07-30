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
#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>

#include "kheops/kernel/factory.h"
#include "kheops/kernel/inputbase.h"
#include "kheops/kernel/publisher.h"
#include "kheops/iostream/shmserializer.h"

#define REGISTER_FUNCTION(classname) \
   static const BuilderImpl<classname,Function> classname(#classname); 

class Function
{
	private : 
		std::string uuid;

		std::vector<InputBase*> input;

	protected:

		bool publish;
		bool save;

	public : 
		Function(): publish(false),save(false){}
		virtual ~Function();

		virtual void exec();
		virtual size_t type() = 0;
		virtual std::string type_name() = 0;
		//virtual void setValue(double dvalue, int x,int y) = 0;
		//virtual void setSize(int x,int y) = 0;
		virtual int getRows()=0;
		virtual int getCols()=0;
		
		inline const std::string& getUuid() { return uuid;  }
		inline virtual void setUuid(const std::string& uuid  ) { this->uuid = uuid;}

		inline void add_input(InputBase * is) { input.push_back(is);}
		inline std::vector<InputBase*>& get_input(){ return input;}

		void copy_buffer();
		void publish_data();

		// This function is called by the runner after free mutex
		virtual void exec_afterCompute();
		// This function can be overloaded by user to add traitment after compute
		// This functions is called by nsync_afterCompute
		virtual void uexec_afterCompute() = 0;
                
		// This function is called by the runner just before the main loop
		virtual void prerun() = 0;
		// This function is called by prerun : can be overloaded by user
		virtual void uprerun() = 0;

		//this two functions must be overloaded for each Function 
		// Payload of the function
		virtual void compute() = 0;
		// Called by kernel to set Input to the current Function
		virtual void setparameters() = 0;

		inline bool is_publish_active(){return publish;}
		inline void set_publish(bool state){publish = state;}
                virtual void active_publish(bool state) = 0; 
		virtual void set_topic_name(const std::string &topic) = 0;
		
		inline bool is_save_active(){return save;}
		inline void set_save(bool state){save = state;}
                virtual void active_save(bool state) = 0; 
};

template<class T>
class FTemplate : public Function
{
	protected : 

		T output;
		DataPublisher<T>* o_pub;
		ShmSerializer<T> serializer; 

	public : 

		FTemplate() : o_pub(NULL) {}
		virtual ~FTemplate();
                
		virtual size_t type(){ return typeid(T).hash_code();}
		std::string type_name() { return typeid(T).name();}
		
		inline const T& getOutput() const { return output;}
                operator  T& () { return output; }
                
		inline const T& operator()() const
                {
                        return output;
                }
		
		//this two functions must be overloaded for each Function 
		// Payload of the function
		virtual void compute() = 0;
		// Called by kernel to set Input to the current Function
		virtual void setparameters() = 0;
		
		// Do nothing, can be overloaded by end users
		virtual void uexec_afterCompute(){}
		
		// This function is called by the runner just before the main loop
		virtual void prerun();
		// Do nothing : can be overloaded by user
		virtual void uprerun(){}

		virtual void set_topic_name(const std::string &topic);
		virtual void active_publish(bool state);
		virtual void active_save(bool state);
		virtual void exec_afterCompute();
};

class FMatrix : public FTemplate<MatrixXd>
{
	public : 
		FMatrix();
		FMatrix(int X,int Y);
                FMatrix(int X,int Y, double dvalue);
		virtual ~FMatrix(){}

                virtual void compute() = 0;
		virtual void setparameters() = 0;
		inline virtual int getRows(){return output.rows();}
		inline virtual int getCols(){return output.cols();}
		inline virtual int getSize(){return output.size();}

		inline virtual void setValue(double dvalue, int row,int col) { output=MatrixXd::Constant(row,col,dvalue);}
		inline virtual void setSize(int rows, int cols){ output = MatrixXd::Constant( rows , cols ,0); }
};


class FScalar : public FTemplate<double>
{
	public : 
		FScalar();
                FScalar( double dvalue);
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
