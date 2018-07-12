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

#ifndef _BOOLEAN_H_
#define _BOOLEAN_H_

#include "kernel/function.h"
#include "kernel/kernel.h"

/********************************************************************************************************/
/**************************************************  AND  ***********************************************/
/********************************************************************************************************/

class MAND :  public FMatrix
{
	private :

                ISMInput inMatrix;

        public :

                virtual ~MAND(){}

                virtual void compute();
                virtual void setparameters();
};

class MSAND :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSAND(){}

                virtual void compute();
                virtual void setparameters();
};


class SAND :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAND(){}

                virtual void compute();
                virtual void setparameters();

};

/********************************************************************************************************/
/**************************************************  OR  ************************************************/
/********************************************************************************************************/

class MOR :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MOR(){}

                virtual void compute();
                virtual void setparameters();
};

class MSOR :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSOR(){}

                virtual void compute();
                virtual void setparameters();
};


class SOR :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SOR(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/**************************************************  XOR  ***********************************************/
/********************************************************************************************************/

class MXOR :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MXOR(){}

                virtual void compute();
                virtual void setparameters();
};

class MSXOR :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSXOR(){}

                virtual void compute();
                virtual void setparameters();
};


class SXOR :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SXOR(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/**************************************************  NOT  ***********************************************/
/********************************************************************************************************/

class MNOT :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MNOT(){}

                virtual void compute();
                virtual void setparameters();
};

class SNOT :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SNOT(){}

                virtual void compute();
                virtual void setparameters();
};


#endif // _BOOLEAN_H_
