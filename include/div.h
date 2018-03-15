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

#ifndef _DIV_H_
#define _DIV_H_

#include "function.h"
#include "kernel.h"
#include <iostream>

class MMDiv : public FMatrix
{
        private :

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

                ISInput numerator;
                ISInput denumerator;

        public :

                virtual ~SSDiv(){}

                virtual void compute();
                virtual void setparameters();

};

#endif // _DIV_H_
