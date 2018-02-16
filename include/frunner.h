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

#ifndef __FRUNNER_H__
#define __FRUNNER_H__

#include "runner.h"
#include "function.h"


class FRunner : public Runner
{
        protected :

                std::vector<Graph::vertex_descriptor const *> functions;

        public :
                FRunner(int id) : Runner(id) {}
                virtual ~FRunner() {}

                virtual void exec()
                {
			checkFunction();

                        while( !Runner::__is_stop()   )
                        {
                                for(auto it = functions.begin(); it != functions.end(); it++)
                                {

                                        wait_for_produce(**it);
				//TODO : décider si le runner attend la fin de l'execution de la boite successeur 
				//	Si UN seul manager et toutes les functions attachées au RT_TOKEN inutile
				// 	Si des functions peuvent ne pas être reliés au RT_TOKEN : 
				//		- Permet de synchroniser ou de ne pas synchroniser leurs executions
				// 		- Les deux comportements peuvent être intéressant.
                                //      wait_for_consume(**it);
		
				// Check Here NULL Pointer ? 
                                        ((*g)[**it])->compute();
                                        consume(**it);
                                        produce(**it);
                                }
                        }
                        for(auto it = functions.begin(); it != functions.end(); it++) {produce(**it); }
                }

		void checkFunction()
		{
			for(auto it = functions.begin(); it != functions.end(); it++)
			{
				if( ((*g)[**it]) == NULL) {
					 throw  std::invalid_argument("Runner "+std::to_string(id)+" : Function uninitialized");
				}
			} 
		}

                void add_node(Graph::vertex_descriptor const * node )
                {
                        functions.push_back(node);
                }

		//TODO : pour l'instant les functions sont pushées les unes à la suite des autres
		// 	Il faut faire attention de les pushés dans l'ordre d'exécution
		//
		// 	Faire une fonction qui utilise le graphe pour sélectionner automatiquement 
		//	le bon ordre d'éxecution

};

#endif //__FRUNNER_H__
