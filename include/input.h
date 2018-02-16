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

#ifndef __INPUT_H__
#define __INPUT_H__

#include <Eigen/Dense>

using Eigen::MatrixXd;


class Input
{
        protected :

                MatrixXd const * input;
		std::string puuid;

        public:

                Input( ) {}
                Input( MatrixXd const *  i ) : input(i){}
                Input( const MatrixXd &  i ) { input = &i; }

                virtual ~Input() {}

                inline void setInput( MatrixXd const * i  ){ input = i;  }
                inline void setInput( const MatrixXd & i  ){ input = &i;  }

		inline std::string getUuid(){  return puuid; }

                inline const MatrixXd& getInput() const { return *input;}

                inline auto operator()(int x,int y) const
                {
                        return (*input)(x,y);
                }

                inline const MatrixXd& operator()() const
                {
                        return *input;
                }

                operator const  MatrixXd& () { return *input; }
};

#endif // __INPUT_H__
