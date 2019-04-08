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

	protected:

		std::vector<InputBase*> input;
		bool publish;
		bool save;

	public : 
		Function(): publish(false),save(false){}
		virtual ~Function();

		virtual size_t type() = 0;
		virtual std::string type_name() = 0;
		std::string class_name(){ return abi::__cxa_demangle(typeid(*this).name(),0,0,NULL) ;}

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


		/*  Kernel Part function : */

		// Called by kernel to set Input to the current Function and to control Input type
		virtual void ksetparameters();
		// This function is called by the runner after free mutex
		virtual void kexec_afterCompute();
		// This function is called before starting the Runner (kernel part)
		virtual void kprerun();

		/*  USER Part function : */

		// This function can be overloaded by user to add traitment after the compute function
		// This functions is called by nsync_afterCompute
		// Operation perform by this functions is out the RT token control
		// Run operation which no requiert synchronisation
		// AND keep traitement shorter than the dead time 
		virtual void exec_afterCompute() = 0;

		// This function is called before starting the Runner (user part)
		// It 's useful to set ressources when all functions and links are created
		virtual void prerun() = 0;

		//this two functions must be overloaded for each Function 
		// Payload of the function
		virtual void compute() = 0;
		// Called by kernel to set Input to the current Function
		// This function is called before building links between functions 
		// So at this moment you can't acces to input information
		// If you want perform control to the input use "uprerun" function instead
		virtual void setparameters() = 0;

		// This function are called after starting the runner
		// On each state changement, the function is executed
		// Note : onRun function are always run ones when starting the runner
		// call when the Runner move to QUIT state
		virtual void onQuit() = 0;
		// call when the Runner move to RUN state
		virtual void onRun() = 0;
		// call when the Runner move to PAUSE state
		virtual void onPause() = 0;


		inline bool is_publish_active(){return publish;}
		inline void set_publish(bool state){publish = state;}
                virtual void active_publish(bool state) = 0; 
		virtual void set_pub_name(const std::string &name) = 0;
		virtual void publish_activity() = 0;
		
		inline bool is_save_active(){return save;}
		inline void set_save(bool state){save = state;}
                virtual void active_save(bool state) = 0; 
		virtual void load_save() = 0;
		virtual void save_activity() = 0;

		// Perhaps move this function in a intermediate class between FTemplate and FMatrix : 
		// FCollection and put all the interface to check Size, dimension, etc ...
		// FMatrix and futur class (FImage, etc ... ) should extend FCollection
		// Could avoid to implement CompareSize in FScalar ... 
		virtual void checkShape() = 0;
		virtual void checkSize() = 0;
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
		virtual std::string type_name() { return typeid(T).name();}
		
		inline const T& getOutput() const { return output;}
                operator  T& () { return output; }
                
		inline const T& operator()() const
                {
                        return output;
                }
		
		/*  USER Part function : */

		// Called by kernel to set Input to the current Function
		virtual void setparameters() = 0;

		//this two functions must be overloaded for each Function 
		// Payload of the function
		virtual void compute() = 0;
		
		// Do nothing, can be overloaded by end users
		virtual void exec_afterCompute(){}

		// Do nothing : can be overloaded by user
		virtual void prerun(){}
		
		virtual void onQuit() {}
		virtual void onRun() {}
		virtual void onPause() {}

		virtual void set_pub_name(const std::string &pub);
		virtual void active_publish(bool state);
		virtual void publish_activity();
		virtual void active_save(bool state);
		virtual void load_save();
		virtual void save_activity();

		// Perhaps move this function in a intermediate class between FTemplate and FMatrix : 
		// FCollection and put all the interface to check Size, dimension, etc ...
		// FMatrix and futur class (FImage, etc ... ) should extend FCollection
		// Could avoid to implement CompareSize in FScalar ... 
		virtual void checkShape(){}
		virtual void checkSize();
		virtual void compareSize(iLinkBase& i){ (void)(i);}
};


enum MATRIXSHAPE{POINT,VECTOR,RVECTOR,CVECTOR,MATRIX};

//Note : should be possible to use eigen template power to get FVector class
// Could be a better option than using flag into FMatrix Constructor to build Vector Function
class FMatrix : public FTemplate<MatrixXd>
{
	private : 

		unsigned int shape;

	public : 
		FMatrix(unsigned int shape = MATRIX);
		virtual ~FMatrix(){}

                virtual void compute() = 0;
		virtual void setparameters() = 0;
		
		inline virtual void setValue(double dvalue, int row,int col) { output=MatrixXd::Constant(row,col,dvalue);}
		inline virtual void setSize(int rows, int cols){ output = MatrixXd::Constant( rows , cols ,0); }
		virtual void compareSize(iLinkBase& link);

		inline int getRows(){return output.rows();}
		inline int getCols(){return output.cols();}
		inline int getSize(){return output.size();}

		inline bool isPoint(){ return output.size() == 1;}
		inline bool isVect(){ return output.rows() == 1 || output.cols() == 1;}
		inline bool isRowVect(){ return output.rows() == 1 && output.cols() > 1;}
		inline bool isColVect(){ return output.rows() > 1 && output.cols() == 1;}

		inline unsigned int getShape(){return shape;}
		virtual void checkShape();
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
		inline virtual int getRows(){return 1;}
		inline virtual int getCols(){return 1;}
};

#endif  // __FUNCTION_H__
