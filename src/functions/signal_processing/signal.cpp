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

#include "functions/signal_processing/signal.h"


/********************************************************************************************************/
/*******************************************  Front Detection   *****************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(SFrontDetection);
REGISTER_FUNCTION(MFrontDetection);

//TODO
void SFrontDetection::compute()
{
}

void SFrontDetection::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}

//TODO
void MFrontDetection::compute()
{
}

void MFrontDetection::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}


/********************************************************************************************************/
/********************************************   THRESOLD   **********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MThreshold);
REGISTER_FUNCTION(SThreshold);
REGISTER_FUNCTION(MThreshold_Custom);
REGISTER_FUNCTION(SThreshold_Custom);
REGISTER_FUNCTION(MSSThreshold_Custom);
REGISTER_FUNCTION(MMSThreshold_Custom);
REGISTER_FUNCTION(MSMThreshold_Custom);

void MThreshold::compute()
{
        output = (inMatrix()(output).array().max(0)).min(1)  ;
}

void MThreshold::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}


void SThreshold::compute()
{
        if( inScalar()() < 0 ) output = 0;
        else if( inScalar()() > 1 ) output = 1;
        else output = inScalar()();
}

void SThreshold::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
}


void MThreshold_Custom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()().array() )).min( thresMax()().array() );
}

void MThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void SThreshold_Custom::compute()
{
        if( inScalar()() < thresMin()() ) output = thresMin()();
        else if( inScalar()() > thresMax()() ) output = thresMax()() ;
        else output = inScalar()();
}

void SThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void MSSThreshold_Custom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()()  )).min( thresMax()() )  ;
}

void MSSThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void MMSThreshold_Custom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()().array() )).min( thresMax()() );
}

void MMSThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

void MSMThreshold_Custom::compute()
{
        output = (inMatrix()(output).array().max( thresMin()() )).min( thresMax()().array() );
}

void MSMThreshold_Custom::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(thresMin,"thresMin", getUuid());
        Kernel::instance().bind(thresMax,"thresMax", getUuid());
}

