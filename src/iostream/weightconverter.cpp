/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of kheops <https://github.com/instar-robotics/kheops>.
 
  kheops is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  kheops is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include "kheops/iostream/weightconverter.h"
#include "kheops/iostream/serialization.h"
#include "kheops/links/imatrix.h"
#include "ros/console.h"

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
				MATRIX tmpM;
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
								if( ignore_check_size ) ROS_WARN("Load Weight : Weight Matrix Dimension in weight file don't match with the expected Weight Matrix dimension");
								else  throw std::invalid_argument("Load Weight : Weight Matrix Dimension in weight file don't match with the expected Weight Matrix dimension");
							}

							unsigned int rows=std::min(tmpM.rows(), itmp->w().rows());
							unsigned int cols=std::min(tmpM.cols(), itmp->w().cols());
						 	itmp->w().topLeftCorner(rows,cols)=tmpM.topLeftCorner(rows, cols);
							
							if(tmpF.rows() != itmp->getInitWRows() || tmpF.cols() != itmp->getInitWCols()) 
							{
								if( ignore_check_size ) ROS_WARN("Load Weight : Filter Matrix Dimension in weight file don't match with the expected Filter Matrix dimension");
								else  throw std::invalid_argument("Load Weight : Filter Matrix Dimension in weight file don't match with the expected Filter Matrix dimension");
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
    		ROS_WARN_STREAM("Load Weight : Unable to open \"" << file << "\" WEIGHT file : weight will be not loaded.");
  	}
	catch(boost::archive::archive_exception e )
	{
		std::exception_ptr eptr = std::current_exception() ;

		in.close();
    		ROS_FATAL_STREAM("Unable to read \"" << file << "\" WEIGHT file : file is corrupted. weight will be not loaded.");
		std::rethrow_exception(eptr);
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
    		ROS_WARN_STREAM("Unable to write " << file << " RES file : weight will be not saved.");
  	}
}
