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

#ifndef _POW_H_
#define _POW_H_

#include "kernel/function.h"
#include "kernel/kernel.h"

/********************************************************************************************************/
/*************************************************  Exp   ***********************************************/
/********************************************************************************************************/

class MExp : public FMatrix
{
        private :

                ISMInput exponent;

        public :

                virtual ~MExp(){}

                virtual void compute();
                virtual void setparameters();
};

class SExp : public FScalar
{
        private :

                ISInput exponent;

        public :

                virtual ~SExp(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/***********************************************   LOG   ************************************************/
/********************************************************************************************************/

class MLog : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MLog(){}

                virtual void compute();
                virtual void setparameters();
};

class SLog : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SLog(){}

                virtual void compute();
                virtual void setparameters();
};

class MLog10 : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MLog10(){}

                virtual void compute();
                virtual void setparameters();
};

class SLog10 : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SLog10(){}

                virtual void compute();
                virtual void setparameters();
};


/********************************************************************************************************/
/*************************************************  POW   ***********************************************/
/********************************************************************************************************/

class MPow : public FMatrix
{
        private :

                ISMInput base;
                ISMInput exponent;

        public :

                virtual ~MPow(){}

                virtual void compute();
                virtual void setparameters();
};

class SPow : public FScalar
{
        private :

                ISInput base;
                ISInput exponent;

        public :

                virtual ~SPow(){}

                virtual void compute();
                virtual void setparameters();

};

class MSPow : public FMatrix
{
        private :

                ISMInput base;
                ISInput exponent;

        public :

                virtual ~MSPow(){}

                virtual void compute();
                virtual void setparameters();
};

class SMPow : public FMatrix
{
        private :

                ISInput base;
                ISMInput exponent;

        public :

                virtual ~SMPow(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/***********************************************   SQRT   ***********************************************/
/********************************************************************************************************/

class MSqrt : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSqrt(){}

                virtual void compute();
                virtual void setparameters();
};

class SSqrt : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSqrt(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*********************************************   SQUARE   ***********************************************/
/********************************************************************************************************/

class MSquare : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSquare(){}

                virtual void compute();
                virtual void setparameters();
};

class SSquare : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSquare(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/**********************************************   CUBE   ************************************************/
/********************************************************************************************************/

class MCube : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MCube(){}

                virtual void compute();
                virtual void setparameters();
};

class SCube : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCube(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*********************************************   Inverse   **********************************************/
/********************************************************************************************************/

class MInverse : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MInverse(){}

                virtual void compute();
                virtual void setparameters();
};

class SInverse : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SInverse(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _POW_H_
