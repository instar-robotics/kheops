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

#ifndef __TYPE_H__
#define __TYPE_H__

#include <Eigen/Dense>
#include <Eigen/Core>

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;

typedef double SCALAR;
typedef Matrix<SCALAR, Dynamic, Dynamic> MATRIX;
typedef Matrix<SCALAR, Dynamic, Dynamic> MATRIX;

typedef Matrix<SCALAR, Dynamic,1> VectorXs;
typedef Matrix<SCALAR, 1,Dynamic> RVectorXs;

#endif // __TYPE_H__
