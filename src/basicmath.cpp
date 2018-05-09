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

#include "basicmath.h"
#include <cmath>

/********************************************************************************************************/
/*************************************************  Max   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(Max);


void Max::compute()
{
	output = inMatrix().i().minCoeff() * inMatrix().w() ;
}

void Max::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

/********************************************************************************************************/
/*************************************************  Min   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(Min);

void Min::compute()
{
	output = inMatrix().i().maxCoeff() * inMatrix().w() ;
}

void Min::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

/********************************************************************************************************/
/*************************************************  Abs   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MAbs);
REGISTER_FUNCTION(SAbs);

void MAbs::compute()
{
	output = inMatrix()(output).array().abs() ;
}

void MAbs::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SAbs::compute()
{
	output = fabs(inScalar()());
}

void SAbs::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/***********************************************   SQRT   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MModulo);
REGISTER_FUNCTION(MSModulo);
REGISTER_FUNCTION(SModulo);

void MModulo::compute()
{
}

void MModulo::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
}

void MSModulo::compute()
{
}

void MSModulo::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
}

void SModulo::compute()
{
}

void SModulo::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
}

/********************************************************************************************************/
/********************************************   THRESOLD   **********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MThreshold);
REGISTER_FUNCTION(SThreshold);
REGISTER_FUNCTION(MThreshold_Custom);
REGISTER_FUNCTION(MSThreshold_Custom);
REGISTER_FUNCTION(SThreshold_Custom);

void MThreshold::compute()
{
}

void MThreshold::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SThreshold::compute()
{
}

void SThreshold::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MThreshold_Custom::compute()
{
}

void MThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void MSThreshold_Custom::compute()
{
}

void MSThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void SThreshold_Custom::compute()
{
}

void SThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

/********************************************************************************************************/
/*******************************************   Derivative   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MDerivative);
REGISTER_FUNCTION(SDerivative);
REGISTER_FUNCTION(MZ_1);
REGISTER_FUNCTION(SZ_1);

void MDerivative::compute()
{
}

void MDerivative::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SDerivative::compute()
{
}

void SDerivative::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MZ_1::compute()
{
}

void MZ_1::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SZ_1::compute()
{
}

void SZ_1::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

