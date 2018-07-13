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


#ifndef _SIGNAL_H_
#define _SIGNAL_H_

#include "kernel/function.h"
#include "kernel/kernel.h"


/********************************************************************************************************/
/*******************************************  Front Detection   *****************************************/
/********************************************************************************************************/

const IString FD_SUP = "up" ;
const IString FD_SDOWN = "down" ;
const IString FD_SBOTH = "both" ;

const unsigned int  FD_IUP = 1 ;
const unsigned int FD_IDOWN = 0 ;
const unsigned int FD_IBOTH = -1 ;

bool checkMode(const std::string & smode, int &mode);

class SFrontDetection : public FScalar
{
        private :

                ISInput inScalar;
                ISInput threshold;
		IString mode;
		
		double z_1;
		int imode;

        public :

                virtual ~SFrontDetection(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();
};

class MFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput threshold;
		IString mode;

		MatrixXd z_1;
		int imode;

        public :

                virtual ~MFrontDetection(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();
};

class MMFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput threshold;
		IString mode;
		
		MatrixXd z_1;
		int imode;

        public :

                virtual ~MMFrontDetection(){}

                virtual void uprerun();
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

                virtual ~SThreshold(){}

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

                virtual ~MThreshold_Custom(){}

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

                virtual ~SThreshold_Custom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSThreshold_Custom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MSSThreshold_Custom(){}

                virtual void compute();
                virtual void setparameters();
};


class MMSThreshold_Custom : public FMatrix
{
        private :
                ISMInput inMatrix;

                ISMInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MMSThreshold_Custom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSMThreshold_Custom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISMInput thresMax;

        public :

                virtual ~MSMThreshold_Custom(){}

                virtual void compute();
                virtual void setparameters();
};


#endif // _SIGNAL_H_
