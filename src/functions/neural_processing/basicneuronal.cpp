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

#include "functions/neural_processing/basicneuronal.h"

/********************************************************************************************************/
/************************************************ KeepMax  *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(KeepMax);

void KeepMax::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMax()(); i++)
	{
		MatrixXd::Index maxRow, maxCol;
  		double max = inMatrix().i().maxCoeff(&maxRow, &maxCol);
	
		output(maxRow,maxCol) = max * inMatrix().w();
	}
}

void KeepMax::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(nMax,"nMax", getUuid());
}

/********************************************************************************************************/
/************************************************ KeepMin  *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(KeepMin);

void KeepMin::compute()
{
	output = MatrixXd::Constant(output.rows(),output.cols(), 0);
	for(unsigned int i = 0; i < nMin()(); i++)
	{
		MatrixXd::Index minRow, minCol;
  		double min = inMatrix().i().minCoeff(&minRow, &minCol);
	
		output(minRow,minCol) = min * inMatrix().w();
	}
}

void KeepMin::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(nMin,"nMax", getUuid());
}


/********************************************************************************************************/
/**********************************************  ActToPop   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(ActToPop);

void ActToPop::compute()
{
	static auto vect = getMapVect(output);
	double value = activity()();

	// Threshold betwen 0 and 1
	if( value < 0 ) value = 0;
	if( value >= 1 ) value = 1 - std::numeric_limits<double>::epsilon();

	unsigned int index = (unsigned int)(value * vect.size());

	vect(index) = activity().w();
}

void ActToPop::setparameters()
{
        Kernel::instance().bind(activity,"activity", getUuid());

	// Check Output dimension : 
	if( output.rows() != 1 && output.cols() != 1 ) 
	{
		throw std::invalid_argument("ActToPop : Output must be a vector (Rows or Cols)");	
	}
}

/********************************************************************************************************/
/**********************************************  VActToPop   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(VActToPop);

void VActToPop::compute()
{
}

void VActToPop::setparameters()
{
        Kernel::instance().bind(activity,"activity", getUuid());
}

void VActToPop::uprerun()
{
}

/********************************************************************************************************/
/**********************************************  PopToAct   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(PopToAct);

void PopToAct::compute()
{
	static auto vect = getMapVect(population.i());

//	output = 
}

void PopToAct::setparameters()
{
        Kernel::instance().bind(population,"population", getUuid());
}

void PopToAct::uprerun()
{
	// Check Output dimension : 
	if( population().i().rows() != 1 && population().i().cols() != 1 ) 
	{
		throw std::invalid_argument("PopToAct : Input \"population\" must be a vector (Rows or Cols)");	
	}
}

/********************************************************************************************************/
/**********************************************  PopToVAct   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(PopToVAct);

void PopToVAct::compute()
{
}

void PopToVAct::setparameters()
{
        Kernel::instance().bind(population,"population", getUuid());
}

void PopToVAct::uprerun()
{
}
