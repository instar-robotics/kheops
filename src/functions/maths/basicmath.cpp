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

#include "functions/maths/basicmath.h"
#include <cmath>
#include <algorithm>

/********************************************************************************************************/
/***********************************************  ArgMax  ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(ArgMax1D);

void ArgMax1D::upreload()
{
	if( inMatrix().i().rows() != 1 && inMatrix().i().cols() != 1 ) 
	{
		throw std::invalid_argument("ArgMax1D Function : input have to be a vector (1D Matrix) !");
	}
}

void ArgMax1D::compute()
{
	MatrixXd::Index maxRow, maxCol;

	inMatrix().i().maxCoeff(&maxRow, &maxCol);

        output = std::max(maxRow,maxCol) ;
}

void ArgMax1D::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

REGISTER_FUNCTION(ArgMax2D);

void ArgMax2D::upreload()
{
	if( inMatrix().i().rows() <= 1 || inMatrix().i().cols() <= 1 ) 
	{
		throw std::invalid_argument("ArgMax2D Function : input have to be a 2D Matrix !");
	}
}

void ArgMax2D::compute()
{
	MatrixXd::Index maxRow, maxCol;
 	auto mOut = getMapVect(output) ;

        inMatrix().i().maxCoeff(&maxRow, &maxCol);
	mOut(0) = maxRow;
	mOut(1) = maxCol;
}

void ArgMax2D::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
	
	if( output.size() != 2 ) 
	{
		throw std::invalid_argument("ArgMax2D Function : output must have only 2 neurons !");
	}
}


/********************************************************************************************************/
/***********************************************  ArgMin  ***********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(ArgMin1D);

void ArgMin1D::upreload()
{
	if( inMatrix().i().rows() != 1 && inMatrix().i().cols() != 1 )
        {
                throw std::invalid_argument("ArgMin1D Function : input have to be a vector (1D Matrix) !");
        }
}

void ArgMin1D::compute()
{
	MatrixXd::Index minRow, minCol;

        inMatrix().i().minCoeff(&minRow, &minCol);

        output = std::max(minRow,minCol) ;
}

void ArgMin1D::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
}

REGISTER_FUNCTION(ArgMin2D);

void ArgMin2D::upreload()
{
	if( inMatrix().i().rows() <= 1 || inMatrix().i().cols() <= 1 )
        {
                throw std::invalid_argument("ArgMin2D Function : input have to be a 2D Matrix !");
        }
}

void ArgMin2D::compute()
{
	MatrixXd::Index minRow, minCol;
        auto mOut = getMapVect(output) ;

        inMatrix().i().minCoeff(&minRow, &minCol);
        mOut(0) = minRow;
        mOut(1) = minCol;
}

void ArgMin2D::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        
	if( output.size() != 2 )
        {
                throw std::invalid_argument("ArgMin2D Function : output must have only 2 neurons !");
        }
}

/********************************************************************************************************/
/*************************************************  Max  ************************************************/
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
/**********************************************   MODULO   **********************************************/
/********************************************************************************************************/

REGISTER_FUNCTION(MModulo);
REGISTER_FUNCTION(MSModulo);
REGISTER_FUNCTION(SModulo);

void MModulo::compute()
{
  output.array() = inMatrix()().array() - (modulo()().array() * ((inMatrix()().array()/modulo()().array()).cast<int>()).cast<double>());	    
}

void MModulo::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
}

void MSModulo::compute()
{
  output.array() = inMatrix()().array() - (modulo()() * ((inMatrix()().array()/modulo()()).cast<int>()).cast<double>());	    

}

void MSModulo::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
}

void SModulo::compute()
{
  output  =  inScalar()()  - ( modulo()()  * (int)( inScalar()() / modulo()()));
}

void SModulo::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
        Kernel::instance().bind(modulo,"modulo", getUuid());
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
	output =  inMatrix()(output) - z_1;
	inMatrix()(z_1); 
}

void MDerivative::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
}

void SDerivative::compute()
{
	output = inScalar()() - z_1 ;
	z_1 = inScalar()();
}

void SDerivative::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
	z_1 = 0;
}

void MZ_1::compute()
{
	output = z_1 ; 
	inMatrix()(z_1);
}

void MZ_1::setparameters()
{
        Kernel::instance().bind(inMatrix,"inMatrix", getUuid());
        z_1 = MatrixXd::Constant( output.rows(), output.cols(), 0  );
}

void SZ_1::compute()
{
	output = z_1 ; 
	z_1 = inScalar()();
}

void SZ_1::setparameters()
{
        Kernel::instance().bind(inScalar,"inScalar", getUuid());
	z_1 = 0;
}


