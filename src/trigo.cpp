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

#include <cmath>
#include "trigo.h"


/********************************************************************************************************/
/*****************************************  SIN, COS, TAN   *********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MCos);
REGISTER_FUNCTION(SCos);
REGISTER_FUNCTION(MSin);
REGISTER_FUNCTION(SSin);
REGISTER_FUNCTION(MTan);
REGISTER_FUNCTION(STan);

void MCos::compute()
{
       output = (inMatrix()(output)).array().cos();
}

void MCos::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SCos::compute()
{
	output = cos(inScalar()()); 
}

void SCos::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MSin::compute()
{
       output = (inMatrix()(output)).array().sin();
}

void MSin::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SSin::compute()
{
	output = sin(inScalar()()); 
}

void SSin::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MTan::compute()
{
       output = (inMatrix()(output)).array().tan();
}

void MTan::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void STan::compute()
{
	output = tan(inScalar()()); 
}

void STan::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/****************************************  ASIN, ACOS, ATAN   *******************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MAcos);
REGISTER_FUNCTION(SAcos);
REGISTER_FUNCTION(MAsin);
REGISTER_FUNCTION(SAsin);
REGISTER_FUNCTION(MAtan);
REGISTER_FUNCTION(SAtan);

void MAcos::compute()
{
       output = (inMatrix()(output)).array().acos();
}

void MAcos::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SAcos::compute()
{
	output = acos(inScalar()()); 
}

void SAcos::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MAsin::compute()
{
       output = (inMatrix()(output)).array().asin();
}

void MAsin::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SAsin::compute()
{
	output = asin(inScalar()()); 
}

void SAsin::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MAtan::compute()
{
       output = (inMatrix()(output)).array().atan();
}

void MAtan::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SAtan::compute()
{
	output = atan(inScalar()()); 
}

void SAtan::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

/********************************************************************************************************/
/****************************************  SINH, COSH, TANH   *******************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MCosh);
REGISTER_FUNCTION(SCosh);
REGISTER_FUNCTION(MSinh);
REGISTER_FUNCTION(SSinh);
REGISTER_FUNCTION(MTanh);
REGISTER_FUNCTION(STanh);

void MCosh::compute()
{
       output = (inMatrix()(output)).array().cosh();
}

void MCosh::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SCosh::compute()
{
	output = cosh(inScalar()()); 
}

void SCosh::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MSinh::compute()
{
       output = (inMatrix()(output)).array().sinh();
}

void MSinh::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void SSinh::compute()
{
	output = sinh(inScalar()()); 
}

void SSinh::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

void MTanh::compute()
{
       output = (inMatrix()(output)).array().tanh();
}

void MTanh::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

void STanh::compute()
{
	output = tanh(inScalar()()); 
}

void STanh::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}
