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
