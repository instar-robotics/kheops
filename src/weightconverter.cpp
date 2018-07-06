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
#include "weightconverter.h"
#include "serialization.h"

void WeightConverter::load(std::map<std::string, InputBase*> &inputs )
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
				unsigned int type;
				MatrixXd tmpM;
				SparseMatrix<double> tmpF;

				ia >> il_uuid;
				ia >> type;

				if( type == DENSE ) 
				{
					boost::serialization::load( ia  , tmpM );

				}
				else if( type == SPARSE) 
				{
					boost::serialization::load( ia  , tmpM );
					boost::serialization::load( ia  , tmpF );

				}
				else
				{
					boost::archive::archive_exception::exception_code ec =  boost::archive::archive_exception::array_size_too_short ;
					std::string msg = "Unknown Matrix type : weight file \""+file+"\" is corrupted";
					throw boost::archive::archive_exception(ec  ,"",msg.c_str());
				}

				for( unsigned int j = 0 ; j < inputs[in_uuid]->size(); j++)
				{
					if( (*inputs[in_uuid])(j).getUuid() == il_uuid)
					{
						if( typeid( (*inputs[in_uuid])(j) ).hash_code() == typeid( IDenseMatrix ).hash_code() && type == DENSE )
						{
							IMMatrix * itmp =  dynamic_cast<IDenseMatrix*>(  (&(*inputs[in_uuid])(j))); 								

							unsigned int rows=std::min(tmpM.rows(), itmp->w().rows());
							unsigned int cols=std::min(tmpM.cols(), itmp->w().cols());
						 	itmp->w().topLeftCorner(rows,cols)=tmpM.topLeftCorner(rows, cols);
						}
						else if( typeid((*inputs[in_uuid])(j)).hash_code() == typeid( ISparseMatrix ).hash_code() && type == SPARSE)
						{
							IMMatrix * itmp =  dynamic_cast<ISparseMatrix*>(  (&(*inputs[in_uuid])(j))); 								
							unsigned int rows=std::min(tmpM.rows(), itmp->w().rows());
							unsigned int cols=std::min(tmpM.cols(), itmp->w().cols());
							itmp->w().topLeftCorner(rows,cols)=tmpM.topLeftCorner(rows, cols);
						//	if( 
						//	 dynamic_cast<ISparseMatrix&>((*inputs[in_uuid])[j]).f() = tmpF;
						//	tmpF.resize(rows,cols);
						}
						//else throw std::exception("ilink type in res is not egal to ilink type in xml");
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
			if(  input->second->type() ==  typeid(IMMatrix).hash_code()  ||
				  input->second->type() ==  typeid(IDenseMatrix).hash_code()  ||
				 	 input->second->type() ==  typeid(ISparseMatrix).hash_code())
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
				oa <<  (**input)(i).getUuid();

				if( typeid((**input)(i)).hash_code()==typeid( ISparseMatrix ).hash_code() )
				{
					oa << SPARSE ; 
					boost::serialization::save( oa, dynamic_cast<ISparseMatrix&>((**input)(i)).f());
				}
				else	
				{
					oa << DENSE ; 
				}
				boost::serialization::save( oa, dynamic_cast<IMMatrix&>((**input)(i)).w() );
			}
		}
		out.close();
	}
 	catch (std::ofstream::failure e) {
    		std::cout << "Unable to write "+file+" RES file : weight will be not saved." << std::endl;
  	}
}
