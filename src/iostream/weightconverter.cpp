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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include <iostream>
#include "kheops/iostream/weightconverter.h"
#include "kheops/iostream/serialization.h"
#include "kheops/links/imatrix.h"

void WeightConverter::load(std::map<std::string, InputBase*> &inputs, bool ignore_check_size )
{
	std::string in_uuid;
	unsigned int nb_link = 0;
	unsigned int nb_input = 0;
	std::ifstream in;
	in.exceptions ( std::ifstream::failbit | std::ifstream::badbit);

	try{

	in.open(file);
        boost::archive::binary_iarchive ia (in);
	
	ia >> nb_input;
	for(unsigned int i = 0; i < nb_input; i++)
	{
		ia >> in_uuid;
		ia >> nb_link;

		if(inputs.find(in_uuid) != inputs.end())
		{	
			for(unsigned int i = 0; i < nb_link; i++)
			{
				std::string il_uuid;
				MatrixXd tmpM;
				MatrixXb tmpF;

				ia >> il_uuid;

				boost::serialization::load( ia  , tmpM );
				boost::serialization::load( ia  , tmpF );


				for( unsigned int j = 0 ; j < inputs[in_uuid]->size(); j++)
				{
					if( (*inputs[in_uuid])[j].getUuid() == il_uuid)
					{
						if( typeid( (*inputs[in_uuid])[j] ).hash_code() == typeid( iMMatrix ).hash_code() )
						{
							iMMatrix * itmp =  dynamic_cast<iMMatrix*>(  (&(*inputs[in_uuid])[j])); 								

							if(tmpM.rows() != itmp->getInitWRows() || tmpM.cols() != itmp->getInitWCols()) 
							{
								if( ignore_check_size ) std::cout << "Warning Load Weight : Weight Matrix Dimension in weight file don't match with the expected Weight Matrix dimension" << std::endl;
								else  throw std::invalid_argument("Error Load Weight : Weight Matrix Dimension in weight file don't match with the expected Weight Matrix dimension");
							}

							unsigned int rows=std::min(tmpM.rows(), itmp->w().rows());
							unsigned int cols=std::min(tmpM.cols(), itmp->w().cols());
						 	itmp->w().topLeftCorner(rows,cols)=tmpM.topLeftCorner(rows, cols);
							
							if(tmpF.rows() != itmp->getInitWRows() || tmpF.cols() != itmp->getInitWCols()) 
							{
								if( ignore_check_size ) std::cout << "Warning Load Weight : Filter Matrix Dimension in weight file don't match with the expected Filter Matrix dimension" << std::endl;
								else  throw std::invalid_argument("Warning Load Weight : Filter Matrix Dimension in weight file don't match with the expected Filter Matrix dimension");
							}
							rows=std::min(tmpF.rows(), itmp->f().rows());
							cols=std::min(tmpF.cols(), itmp->f().cols());
						 	itmp->f().topLeftCorner(rows,cols)=tmpF.topLeftCorner(rows, cols);
						}
						else throw std::invalid_argument("Load Weight : try to bound iMMatrix link on a none Matrix Input");
					}
				}
			}
		}
		in.close();
	}
	}
 	catch (std::ifstream::failure e) {
    		std::cout << "Unable to open \""+file+"\" WEIGHT file : weight will be not loaded." << std::endl;
  	}
	catch(boost::archive::archive_exception e )
	{
		in.close();
    		std::cout << "Unable to read \""+file+"\" WEIGHT file : file is corrupted. weight will be not loaded." << std::endl;
		std::cout << e.what() << std::endl;
		exit(0);
	}
}

void WeightConverter::save(std::map<std::string, InputBase*> &inputs)
{
	std::ofstream out;
	out.exceptions ( std::ofstream::failbit | std::ofstream::badbit);
	try{
		out.open(file);
		boost::archive::binary_oarchive oa(out);

		//TODO : could be optimized : feed the vector at the launch for example
		// But if do : becareful because the function could be evolve during runtime !
		std::vector<IMMInput*> im_inputs;
		for( auto input = inputs.begin() ; input != inputs.end(); input++  )
		{
			if(  input->second->type() ==  typeid(iMMatrix).hash_code() )
			{
				im_inputs.push_back( dynamic_cast<IMMInput*>(input->second));
			}
		}

		unsigned int nb_input = im_inputs.size();
		oa <<  nb_input;

		for( auto input = im_inputs.begin() ; input != im_inputs.end(); input++  )
		{
			oa <<  (*input)->getUuid();
			unsigned int size = (*input)->size();
			oa <<  size;
			for( unsigned int i = 0 ; i < (*input)->size(); i++ )
			{
				oa <<  (**input)[i].getUuid();

				boost::serialization::save( oa, dynamic_cast<iMMatrix&>((**input)[i]).w());
				boost::serialization::save( oa, dynamic_cast<iMMatrix&>((**input)[i]).f() );
			}
		}
		out.close();
	}
 	catch (std::ofstream::failure e) {
    		std::cout << "Unable to write "+file+" RES file : weight will be not saved." << std::endl;
  	}
}
