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

#ifndef _TRIGO_H_
#define _TRIGO_H_

#include "function.h"
#include "kernel.h"

/********************************************************************************************************/
/*****************************************  SIN, COS, TAN   *********************************************/
/********************************************************************************************************/

class MCos : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MCos(){}

                virtual void compute();
                virtual void setparameters();
};

class SCos : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCos(){}

                virtual void compute();
                virtual void setparameters();
};

class MSin : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MSin(){}

                virtual void compute();
                virtual void setparameters();
};

class SSin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSin(){}

                virtual void compute();
                virtual void setparameters();
};

class MTan : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MTan(){}

                virtual void compute();
                virtual void setparameters();
};

class STan : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~STan(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*****************************************  ASIN, ACOS, ATAN  *******************************************/
/********************************************************************************************************/

class MAcos : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAcos(){}

                virtual void compute();
                virtual void setparameters();
};

class SAcos : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAcos(){}

                virtual void compute();
                virtual void setparameters();
};

class MAsin : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAsin(){}

                virtual void compute();
                virtual void setparameters();
};

class SAsin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAsin(){}

                virtual void compute();
                virtual void setparameters();
};

class MAtan : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAtan(){}

                virtual void compute();
                virtual void setparameters();
};

class SAtan : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAtan(){}

                virtual void compute();
                virtual void setparameters();
};


/********************************************************************************************************/
/*****************************************  SINH, COSH, TANH  *******************************************/
/********************************************************************************************************/

class MCosh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MCosh(){}

                virtual void compute();
                virtual void setparameters();
};

class SCosh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCosh(){}

                virtual void compute();
                virtual void setparameters();
};

class MSinh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MSinh(){}

                virtual void compute();
                virtual void setparameters();
};

class SSinh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSinh(){}

                virtual void compute();
                virtual void setparameters();
};

class MTanh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MTanh(){}

                virtual void compute();
                virtual void setparameters();
};

class STanh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~STanh(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _TRIGO_H_
