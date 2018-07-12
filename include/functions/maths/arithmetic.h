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

#ifndef _ARITHMETIC_H_
#define _ARITHMETIC_H_

#include "kernel/function.h"
#include "kernel/kernel.h"

/********************************************************************************************************/
/**********************************************   SUM   *************************************************/
/********************************************************************************************************/

class MSum : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSum(){}

                virtual void compute();
                virtual void setparameters();
};

class SSum : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSum(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSum : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSSum(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*****************************************   Substraction   *********************************************/
/********************************************************************************************************/

class MMSub : public FMatrix
{
        private :

		// Inputs are unique !
                ISMInput diminuende;
                ISMInput subtrahend;

        public :

                virtual ~MMSub(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSub : public FMatrix
{
        private :

		// Inputs are unique !
                ISMInput diminuende;
                ISInput subtrahend;

        public :

                virtual ~MSSub(){}

                virtual void compute();
                virtual void setparameters();
};

class SSSub : public FScalar
{
        private :

		// Inputs are unique !
                ISInput diminuende;
                ISInput subtrahend;

        public :

                virtual ~SSSub(){}

                virtual void compute();
                virtual void setparameters();
};


/********************************************************************************************************/
/*******************************************   Product   ************************************************/
/********************************************************************************************************/

// Wise Product : coefficient-wise operator 
class MMul : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :
                virtual ~MMul(){}

                virtual void compute();
                virtual void setparameters();
};

// Scalar Multiplication
class SMul : public FScalar
{
        private :

                ISInput inScalar;

        public :
                virtual ~SMul(){}

                virtual void compute();
                virtual void setparameters();
};

//Combination of Wise Product and Scalar product 
// For example : 
// With m1, m2 two matrix
// And s1, s2 two scalar 
// output = m1 cwiseProduct m2 * s1 * s2 
class MSMul : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :
                virtual ~MSMul(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/******************************************   Division   ************************************************/
/********************************************************************************************************/

class MMDiv : public FMatrix
{
        private :
		// Inputs are unique
                ISMInput numerator;
                ISMInput denumerator;

        public :

                virtual ~MMDiv(){}

                virtual void compute();
                virtual void setparameters();
};

class MSDiv : public FMatrix
{
        private :

		// Inputs are unique
                ISMInput numerator;
                ISInput denumerator;

        public :

                virtual ~MSDiv(){}

                virtual void compute();
                virtual void setparameters();
};

class SSDiv : public FScalar
{
        private :

		// Inputs are unique
                ISInput numerator;
                ISInput denumerator;

        public :

                virtual ~SSDiv(){}

                virtual void compute();
                virtual void setparameters();
};


#endif // _ARITHMETIC_H_
