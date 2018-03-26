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

#include "mul.h"

REGISTER_FUNCTION(MMul);
REGISTER_FUNCTION(SMul);
REGISTER_FUNCTION(MSMul);

void MMul::compute()
{
	inMatrix[0].accumulate(output);	

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		inMatrix[i].mul_accumulate(output);	
	}

}

void  MMul::setparameters()
{
	inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SMul::compute()
{
	output =  inScalar[0]();
	for(unsigned int i=1; i < inScalar.size(); i++)
	{
		output *= inScalar[i]();
	}
}

void  SMul::setparameters()
{
	inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void MSMul::compute()
{
	double sMul=0;

        sMul = inScalar[0]();

        for(unsigned int i=1; i < inScalar.size(); i++)
        {
        	sMul *= inScalar[i]();
        }
		
	inMatrix[0].accumulate(output);	

	for(unsigned int i=1; i < inMatrix.size(); i++)
	{
		inMatrix[i].mul_accumulate(output);	
	}
	output *= sMul;
}

void  MSMul::setparameters()
{
	inScalar.setMultiple(true);
	inMatrix.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

