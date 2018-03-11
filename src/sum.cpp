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

#include "sum.h"

REGISTER_FUNCTION(MSum);
REGISTER_FUNCTION(SSum);
REGISTER_FUNCTION(MSSum);

void MSum::compute()
{
	if( inMatrix.size() == 0) return;

	inMatrix[0].accumulate(output);	

	for(unsigned int i=0; i < inMatrix.size(); i++)
	{
		inMatrix[0].sum_accumulate(output);	
	}
}

void  MSum::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());

}

void SSum::compute()
{
	std::cout << getUuid() << std::endl;
	if( inScalar.size() == 0) return;

	inScalar[0].accumulate(output);	

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		inScalar[0].sum_accumulate(output);	
	}
	
	std::cout << getUuid() << "" << output << std::endl;
}

void  SSum::setparameters()
{
	std::cout << "ZGEG " << std::endl;
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MSSum::compute()
{
	double sSum=0;

	if( inMatrix.size() == 0 || inScalar.size() == 0) return;

	inMatrix[0].accumulate(output);	

	for(unsigned int i=0; i < inMatrix.size(); i++)
	{
		inMatrix[0].sum_accumulate(output);	
	}

	inScalar[0].accumulate(sSum);	

	for(unsigned int i=0; i < inScalar.size(); i++)
	{
		inScalar[0].sum_accumulate(sSum);	
	}

	output.array()+=sSum;
}

void  MSSum::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}
