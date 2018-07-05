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

#ifndef __SHM_SERIALIZER__
#define __SHM_SERIALIZER__

#include "serialization.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>


namespace bip = boost::interprocess;

#define PADDING 40

template<class Data>
void shm_read(const std::string& name,Data &d)
{
        try{

            bip::shared_memory_object shm(bip::open_only, name.c_str(), bip::read_only);
            bip::mapped_region region(shm, bip::read_only);

            bip::bufferstream bs(std::ios::in);
            bs.buffer(reinterpret_cast<char*>(region.get_address()), region.get_size());

            boost::archive::binary_iarchive ia( dynamic_cast<std::iostream&>(bs));

            boost::serialization::load(ia,d);

        }
        catch(std::bad_alloc& ba)
        {}
        catch( bip::interprocess_exception &bi)
        {}
}

template<class Type>
class ShmSerializer
{
        private :

                boost::archive::binary_oarchive *oa;
                bip::shared_memory_object *shm ;
                bip::mapped_region *region;
                bip::bufferstream *bs;

        public :

                ShmSerializer() : oa(NULL), shm(NULL), region(NULL), bs(NULL) {}
                ~ShmSerializer()
                {
			free();
                }

		void free()
		{
			freeSHM();
			freeBinaryOArchive();
		}

                void buildSHM( const std::string name, const Type &T);
                void freeSHM()
		{
                        if( shm != NULL ) delete(shm);
                        if( region != NULL ) delete(region);
			shm = NULL;
			region = NULL;
		}

                void buildBinaryOArchive()
                {
                        bs = new bip::bufferstream (std::ios::out);

                        bs->buffer(reinterpret_cast<char*>(region->get_address()), region->get_size());

                        oa = new boost::archive::binary_oarchive(dynamic_cast<std::ostream&>(*bs));
                }

                void freeBinaryOArchive()
                {
                        if( oa != NULL ) delete(oa);
                        if( bs != NULL ) delete(bs);
                        oa = NULL;
                        bs = NULL;
                }

                void write(const Type& D )
                {
                        buildBinaryOArchive();
                        boost::serialization::save(*oa,D);
                        freeBinaryOArchive();
                }
};

template<class Type>
void ShmSerializer<Type>::buildSHM( const  std::string name, const Type &T)
{
        if( shm == NULL ) shm = new bip::shared_memory_object(bip::open_or_create, name.c_str(), bip::read_write);
        shm->truncate(sizeof(T)+PADDING);

        if( region == NULL ) region = new bip::mapped_region(*shm, bip::read_write);
}



#endif // __SHM_SERIALIZER__
