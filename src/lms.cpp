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

#include "lms.h"

REGISTER_FUNCTION(LMS);

void LMS::compute()
{	
	Map<MatrixXd> mout = getMapRow(output);

	mout = conditionnals[0].w().cwiseProduct(conditionnals[0].f()) * conditionnals[0].irow() ;
	for(unsigned int i=1; i < conditionnals.size(); i++)
        {
		mout += conditionnals[i].w().cwiseProduct(conditionnals[i].f()) * conditionnals[i].irow() ;
        }

	// Update weight
	MatrixXd grad ; 
	grad = learning_rate()() * (unconditionnal()(grad) - output);

	Map<MatrixXd> lgrad = getMapRow(grad); 
	for(unsigned int i=0; i < conditionnals.size(); i++)
	{
		Map<const MatrixXd> ve = conditionnals[i].irow(); 
		Map<MatrixXd> weight = conditionnals[i].wm(); 
		Map<const MatrixXd> filter = conditionnals[i].fm(); 

		for( unsigned int j = 0; j < weight.cols() ; j++)
		{
			weight.col(j) =  weight.col(j) * ve(1,j) * lgrad(1,j) * filter.col(j) ; 
		}
	}
}

void  LMS::setparameters()
{
	conditionnals.setMultiple(true);

        Kernel::instance().bind(learning_rate,"learning_rate", getUuid());
        Kernel::instance().bind(unconditionnal,"unconditionnal", getUuid());
        Kernel::instance().bind(conditionnals,"conditionnals", getUuid());
}

