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


#ifndef __ISCALAR_H__
#define __ISCALAR_H__

#include "kheops/kernel/ilinkbase.h"
#include "kheops/kernel/inputbase.h"

class iScalar : public iLink<SCALAR,SCALAR>
{
        public:

                iScalar();
                iScalar(SCALAR const * i);
                virtual ~iScalar();

                SCALAR operator()(){return (*input) * weight;}

                inline virtual SCALAR& accumulate(SCALAR& res){return res = (*input) * weight;}
                inline virtual SCALAR& mul_accumulate(SCALAR& res){return res *= (*input) * weight;}
                inline virtual SCALAR& sum_accumulate(SCALAR& res){return res += (*input) * weight;}
                inline virtual SCALAR& sub_accumulate(SCALAR& res){return res -= (*input) * weight;}
                inline virtual SCALAR& div_accumulate(SCALAR& res){return res /= (*input) * weight;}

                friend SCALAR& operator+=(SCALAR& res, iScalar& val) {return val.sum_accumulate(res);}
                friend SCALAR& operator-=(SCALAR& res, iScalar& val) {return val.sub_accumulate(res);}
                friend SCALAR& operator/=(SCALAR& res, iScalar& val) {return val.div_accumulate(res);}
                friend SCALAR& operator*=(SCALAR& res, iScalar& val) {return val.mul_accumulate(res);}
};

typedef Input<iScalar> ISInput;

#endif //__ISCALAR_H__
