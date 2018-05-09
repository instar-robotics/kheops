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


#ifndef _BASIC_MATH_H_
#define _BASIC_MATH_H_

#include "function.h"
#include "kernel.h"

/*
	max , min  [Matrix -> Scalar] , peut aussi faire comme un WTA 
	argmax, argmin [Matrix -> Scalar ] la sortie scalar conserve l'id du max ? 
*/


/********************************************************************************************************/
/*************************************************  Max   ***********************************************/
/********************************************************************************************************/

class Max : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~Max(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*************************************************  Min   ***********************************************/
/********************************************************************************************************/

class Min : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~Min(){}

                virtual void compute();
                virtual void setparameters();
};


/********************************************************************************************************/
/*************************************************  Abs   ***********************************************/
/********************************************************************************************************/

class MAbs : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MAbs(){}

                virtual void compute();
                virtual void setparameters();
};

class SAbs : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAbs(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/**********************************************  Modulo   ***********************************************/
/********************************************************************************************************/

class MModulo : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput modulo;

        public :

                virtual ~MModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class MSModulo : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput modulo;

        public :

                virtual ~MModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class SModulo : public FScalar
{
        private :

                ISInput inScalar;
                ISInput modulo;

        public :

                virtual ~SModulo(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*********************************************  Threshold   *********************************************/
/********************************************************************************************************/

// bounded [0,1]
class MThreshold : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MThreshold(){}

                virtual void compute();
                virtual void setparameters();
};

class SThreshold : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SModulo(){}

                virtual void compute();
                virtual void setparameters();
};

// bounded [min,max]
class MThreshold_Custom : public FMatrix
{
        private :

                ISMInput thresMin;
                ISMInput thresMax;

                ISMInput inMatrix;

        public :

                virtual ~MThreshold(){}

                virtual void compute();
                virtual void setparameters();
};

class MSThreshold_Custom : public FMatrix
{
        private :

                ISInput thresMin;
                ISInput thresMax;

                ISMInput inMatrix;

        public :

                virtual ~MModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class SThreshold_Custom : public FScalar
{
        private :

                ISInput thresMin;
                ISInput thresMax;

                ISInput inScalar;

        public :

                virtual ~SModulo(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/********************************************  Derivative   *********************************************/
/********************************************************************************************************/

class MDerivative : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MDerivative(){}

                virtual void compute();
                virtual void setparameters();
};

class SDerivative : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class MZ_1 : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MZ_1(){}

                virtual void compute();
                virtual void setparameters();
};

class SZ_1 : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SZ_1(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _BASIC_MATH_H_
