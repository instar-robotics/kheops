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

#include "boolean.h"

/********************************************************************************************************/
/*************************************************  AND   ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MAND);
REGISTER_FUNCTION(MSAND);
REGISTER_FUNCTION(SAND);

//TODO
void MAND::compute()
{
}

void MAND::setparameters()
{
	inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

//TODO
void MSAND::compute()
{
}

void MSAND::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

//TODO
void SAND::compute()
{
}

void SAND::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


/********************************************************************************************************/
/*************************************************  OR   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MOR);
REGISTER_FUNCTION(MSOR);
REGISTER_FUNCTION(SOR);

//TODO
void MOR::compute()
{
}

void MOR::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

//TODO
void MSOR::compute()
{
}

void MSOR::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

//TODO
void SOR::compute()
{
}

void SOR::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/************************************************  XOR   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MXOR);
REGISTER_FUNCTION(MSXOR);
REGISTER_FUNCTION(SXOR);

//TODO
void MXOR::compute()
{
}

void MXOR::setparameters()
{
        inMatrix.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

//TODO
void MSXOR::compute()
{
}

void MSXOR::setparameters()
{
        inMatrix.setMultiple(true);
        inScalar.setMultiple(true);
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

//TODO
void SXOR::compute()
{
}

void SXOR::setparameters()
{
        inScalar.setMultiple(true);
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/************************************************  NOT   ************************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MNOT);
REGISTER_FUNCTION(SNOT);

//TODO
void MNOT::compute()
{
}

void MNOT::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

//TODO
void SNOT::compute()
{
}

void SNOT::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

