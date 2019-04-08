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


#ifndef __SERIALIZER__
#define __SERIALIZER__

#include <vector>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <boost/serialization/array.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>


namespace boost
{
namespace serialization
{

template<class Archive, class Type>
void save( Archive& ar,const Type& D)
{
	ar << D;	
}

template<class Archive, class Type>
void load( Archive& ar, Type& D)
{
	ar >> D;
}

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save( Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& M)
{
	typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index rows = M.rows();
	typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index cols = M.cols();

	ar << rows;
	ar << cols;
	ar << make_array( M.data(), M.size() );
}

template<class Archive, typename _Scalar, int _Options, int _MaxRows, int _MaxCols>
void load( Archive& ar, Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& M)
{
	typename Eigen::Matrix<_Scalar,Eigen::Dynamic,Eigen::Dynamic,_Options,_MaxRows, _MaxCols>::Index rows;
	typename Eigen::Matrix<_Scalar,Eigen::Dynamic,Eigen::Dynamic,_Options,_MaxRows, _MaxCols>::Index cols;

	ar >> rows;
	ar >> cols;

	M.resize(rows, cols);

	ar >> make_array( M.data(), M.size() );
}

template <class Archive, typename _Scalar>
void save(Archive& ar, const Eigen::Triplet<_Scalar>& m, const unsigned int)
{
	ar << m.row();
	ar << m.col();
	ar << m.value();
}


template <class Archive, typename _Scalar>
void load(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int)
    {
     _Scalar value;
     unsigned int row,col;

      ar >> row;
      ar >> col;
      ar >> value;

      m = Eigen::Triplet<_Scalar>(row, col, value);
    }

    
template <class Archive, class _Scalar>
void serialize(Archive& ar, Eigen::Triplet<_Scalar>& m, const unsigned int version)
{
      split_free(ar, m, version);
}


template <class Archive, typename _Scalar, int _Options, typename _Index>
void save(Archive& ar, const Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
{
	_Index innerSize = m.innerSize();
	_Index outerSize = m.outerSize();

	typedef typename Eigen::Triplet<_Scalar> Triplet;
	std::vector<Triplet> triplets;

	for (_Index i=0; i < outerSize; ++i)
	for (typename Eigen::SparseMatrix<_Scalar, _Options, _Index>::InnerIterator it(m,i); it; ++it)
	  triplets.push_back( Triplet(it.row(), it.col(), it.value()) );

	ar << innerSize;
	ar << outerSize;
	ar << triplets;
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
void load(Archive& ar, Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
{
	_Index innerSize;
	_Index outerSize;

	ar >> innerSize;
	ar >> outerSize;

	_Index rows = (m.IsRowMajor)? outerSize : innerSize;
	_Index cols = (m.IsRowMajor)? innerSize : outerSize;

	m.resize(rows, cols);

	typedef typename Eigen::Triplet<_Scalar> Triplet;
	std::vector<Triplet> triplets;

	ar >> triplets;

	m.setFromTriplets(triplets.begin(), triplets.end());
}

template <class Archive, typename _Scalar, int _Options, typename _Index>
void serialize(Archive& ar, Eigen::SparseMatrix<_Scalar,_Options,_Index>& m, const unsigned int version)
{
	split_free(ar, m, version);
}



template<class Archive, typename _Scalar>
void serialize(Archive & ar, Eigen::Quaternion<_Scalar>& q, const unsigned int)
{
	ar & q.w();
	ar & q.x();
	ar & q.y();
	ar & q.z();
}

  
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& M,  const unsigned int file_version)
{
	split_free(ar, M, file_version);
}

template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
inline void serialize(Archive& ar, Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t, const unsigned int version)
{
	serialize(ar, t.matrix(), version);
}

} // namespace serialization
} // namespace boost

#endif // __SERIALIZE_EIGEN__
