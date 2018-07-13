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


#ifndef _BASIC_NEURONAL_H_
#define _BASIC_NEURONAL_H_

#include "kernel/function.h"
#include "kernel/kernel.h"

/*
activité -> population 
	(Neurons -> Vector )
	(Vector -> Matrix )
population -> activité  
	(vector -> neurons)
	(matrix -> vector)

// SPARSE_MATRIX -> Nouveau type de liens 
neurons concatenation -> OK 
extract neurons -> OK 

projection (
	neuron-to-vector -> SCALAR-SCALAR 
	vector-to-neuron  -> SCALAR_MATRIX
) (circularity option)

-> Concaténation + extraction : meme opération de projection : on fait une seule boite de projection qui prend en input une sparse matrice définissant les régles de projections 


convolution operator (circularity option) -> OK simple 

shift operator (circularity option) -> On attend discution avec ALEX

N_MAX/N_MIN (reviens à un sort si N = taille entrée)  -> implémenter plus tard si besoin
-> SORT ?? Est-ce que ce n'est pas plus malin de l'appeler SORT ? Avec N -> N premier (soit max soit min)

*/

/********************************************************************************************************/
/**********************************************  Keep_Max   *********************************************/
/********************************************************************************************************/

// Keep N max in the output Matrix
// Output must have same dimension that inMatrix
// nMax : number of max keep in output
class KeepMax : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput nMax;

        public :

                virtual ~KeepMax(){}

                virtual void compute();
                virtual void setparameters();
};

// Keep N max in the output Matrix
// Output must have same dimension that inMatrix
// nMax : number of min keep in output
class KeepMin : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput nMin;

        public :

                virtual ~KeepMin(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _BASIC_NEURONAL_H_
