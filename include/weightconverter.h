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

#ifndef __WEIGHT_CONVERTER__
#define __WEIGHT_CONVERTER__

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <boost/serialization/array.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_free.hpp>

#include <vector>
#include <map>
#include <string>
#include "input.h"
#include "ilink.h"

const int SPARSE = 0;
const int  DENSE = 1;

class WeightConverter
{
	private:
			
		std::string file;	

	public : 
		WeightConverter(const std::string& path) : file(path) {}
		~WeightConverter(){}

		void load(std::map<std::string, IMMInput*> &inputs);
		void save(std::map<std::string, IMMInput*> &inputs);

};

namespace boost
{
namespace serialization
{

    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void save( Archive& ar,
                      const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& M)
                  //    const unsigned int /* file_version */)
    {
      typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index rows = M.rows();
      typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index cols = M.cols();

      ar << rows;
      ar << cols;

      ar << make_array( M.data(), M.size() );
    }

    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void load( Archive& ar,
                      Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& M)
                      //const unsigned int /* file_version */)
    {
      typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index rows, cols;

      ar >> rows;
      ar >> cols;

      //if (rows=!_Rows) throw std::exception(/*"Unexpected number of rows"*/);
      //if (cols=!_Cols) throw std::exception(/*"Unexpected number of cols"*/);

      ar >> make_array( M.data(), M.size() );
    }

    template<class Archive, typename _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void load( Archive& ar,
                      Eigen::Matrix<_Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols>& M)
                   //   const unsigned int /* file_version */)
    {
      typename Eigen::Matrix<_Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols>::Index rows, cols;

      ar >> rows;
      ar >> cols;

      //if (cols=!_Cols) throw std::exception(/*"Unexpected number of cols"*/);

      M.resize(rows, Eigen::NoChange);

      ar >> make_array( M.data(), M.size() );
    }

    template<class Archive, typename _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
    inline void load( Archive& ar,
                      Eigen::Matrix<_Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& M)
                    //  const unsigned int /* file_version */)
    {
      typename Eigen::Matrix<_Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>::Index rows, cols;

      ar >> rows;
      ar >> cols;

      //if (rows=!_Rows) throw std::exception(/*"Unexpected number of rows"*/);

      M.resize(Eigen::NoChange, cols);

      ar >> make_array( M.data(), M.size() );
    }

    template<class Archive, typename _Scalar, int _Options, int _MaxRows, int _MaxCols>
    inline void load( Archive& ar,
                      Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& M)//                      const unsigned int /* file_version */)
    {
      typename Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>::Index rows, cols;

      ar >> rows;
      ar >> cols;

      M.resize(rows, cols);

      ar >> make_array( M.data(), M.size() );
    }

    template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline void serialize(Archive& ar,
                          Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& M,
                          const unsigned int file_version)
    {
      split_free(ar, M, file_version);
    }

    template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
    inline void serialize(Archive& ar,
                          Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t,
                          const unsigned int version)
    {
      serialize(ar, t.matrix(), version);
    }

    template <class Archive, typename _Scalar>
    void save(Archive& ar,
              const Eigen::Triplet<_Scalar>& m,
              const unsigned int /*version*/)
    {
      ar << m.row();
      ar << m.col();
      ar << m.value();
    }


    template <class Archive, typename _Scalar>
    void load(Archive& ar,
              Eigen::Triplet<_Scalar>& m,
              const unsigned int )
    {
//      typename Eigen::Triplet<_Scalar>::Index row, col;
      _Scalar value;

	unsigned int row,col;

     ar >> row;
      ar >> col;
      ar >> value;

      m = Eigen::Triplet<_Scalar>(row, col, value);
    }

    template <class Archive, class _Scalar>
    void serialize(Archive& ar,
                   Eigen::Triplet<_Scalar>& m,
                   const unsigned int version)
    {
      split_free(ar, m, version);
    }

    template <class Archive, typename _Scalar, int _Options, typename _Index>
    void save(Archive& ar,
              const Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
             // const unsigned int /*version*/)
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
    void load(Archive& ar,
              Eigen::SparseMatrix<_Scalar, _Options, _Index>& m)
         //     const unsigned int /*version*/)
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
    void serialize(Archive & ar, Eigen::Quaternion<_Scalar>& q, const unsigned int /*version*/)
    {
      ar & q.w();
      ar & q.x();
      ar & q.y();
      ar & q.z();
    }

} // namespace serialization
} // namespace boost

#endif // __RES_CONVERTER__