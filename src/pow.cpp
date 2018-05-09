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

#include "pow.h"
#include <cmath>

/********************************************************************************************************/
/*************************************************  Exp   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MExp);
REGISTER_FUNCTION(SExp);

void MExp::compute()
{
	//TODO : Could be that : (optimal code)
	//	output = (exponent().w() * exponent().i()).array().exp();
	output = exponent()(output).array().exp() ;
}

void  MExp::setparameters()
{
        Kernel::instance().bind(exponent,"exponent", getUuid());
}

void SExp::compute()
{
	output = exp(exponent()());
}

void  SExp::setparameters()
{
        Kernel::instance().bind(exponent,"exponent", getUuid());
}


/********************************************************************************************************/
/*************************************************  Log   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MLog);
REGISTER_FUNCTION(SLog);
REGISTER_FUNCTION(MLog10);
REGISTER_FUNCTION(SLog10);

void MLog::compute()
{
	output = inMatrix()(output).array().log() ;
}

void MLog::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SLog::compute()
{
	output = log(inScalar()());
}

void SLog::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MLog10::compute()
{
	output = inMatrix()(output).array().log10();
}

void MLog10::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SLog10::compute()
{
	output = log10(inScalar()());
}

void SLog10::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/*************************************************  POW   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MPow);
REGISTER_FUNCTION(SPow);
REGISTER_FUNCTION(MSPow);
REGISTER_FUNCTION(SMPow);


void MPow::compute()
{
	output = base()(output).array().pow( exponent()().array() );
}

void MPow::setparameters()
{
        Kernel::instance().bind(base,"base", getUuid());
        Kernel::instance().bind(exponent,"exponent", getUuid());
}

void SPow::compute()
{
	output = pow( base()() , exponent()() );
}

void SPow::setparameters()
{
        Kernel::instance().bind(base,"base", getUuid());
        Kernel::instance().bind(exponent,"exponent", getUuid());
}

void MSPow::compute()
{
	output = base()(output).array().pow(  exponent()() );
}

void MSPow::setparameters()
{
        Kernel::instance().bind(base,"base", getUuid());
        Kernel::instance().bind(exponent,"exponent", getUuid());
}

void SMPow::compute()
{
	output = pow( base()(), exponent()().array() );
}

void SMPow::setparameters()
{
        Kernel::instance().bind(base,"base", getUuid());
        Kernel::instance().bind(exponent,"exponent", getUuid());
}


/********************************************************************************************************/
/***********************************************   SQRT   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MSqrt);
REGISTER_FUNCTION(SSqrt);


void MSqrt::compute()
{
	output = inMatrix()(output).array().sqrt();
}

void MSqrt::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SSqrt::compute()
{
	output =  sqrt(inScalar()());
}

void SSqrt::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/*********************************************   SQUARE   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MSquare);
REGISTER_FUNCTION(SSquare);


void MSquare::compute()
{
	output = inMatrix()(output).array().square();
}

void MSquare::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SSquare::compute()
{
	output = pow( inScalar()(), 2);
}

void SSquare::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/**********************************************   CUBE   ************************************************/
/********************************************************************************************************/


REGISTER_FUNCTION(MCube);
REGISTER_FUNCTION(SCube);


void MCube::compute()
{
	output = inMatrix()(output).array().cube();
}

void MCube::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SCube::compute()
{
	output = pow( inScalar()(), 3);
}

void SCube::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/*********************************************   Inverse   **********************************************/
/********************************************************************************************************/


REGISTER_FUNCTION(MInverse);
REGISTER_FUNCTION(SInverse);


void MInverse::compute()
{
	output = inMatrix()(output).array().inverse();
}

void MInverse::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SInverse::compute()
{
	output = 1 / inScalar()();
}

void SInverse::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

