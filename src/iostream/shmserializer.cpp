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


#include "kheops/iostream/shmserializer.h"

using namespace Eigen;

/*
template<class Type>
void ShmSerializer<Type>::buildSHM( const  std::string name, const Type &T)
{
        if( shm == NULL ) shm = new bip::shared_memory_object(bip::open_or_create, name.c_str(), bip::read_write);
        shm->truncate(sizeof(T)+PADDING);

        if( region == NULL ) region = new bip::mapped_region(*shm, bip::read_write);
}
*/

template<>
void ShmSerializer<MatrixXd>::buildSHM( const  std::string name, const MatrixXd& M )
{
        if( shm == NULL ) shm = new bip::shared_memory_object(bip::open_or_create, name.c_str(), bip::read_write);
        int seg_size = M.size()* sizeof(double) + sizeof(MatrixXd::Index) * 2+ sizeof(MatrixXd) + PADDING;
        shm->truncate(seg_size);

        if( region == NULL ) region = new bip::mapped_region(*shm, bip::read_write);
}

