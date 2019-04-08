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


#ifndef __ILINK_BASE_H__
#define __ILINK_BASE_H__

#include "kheops/kernel/cominterface.h"
#include "kheops/kernel/publisher.h"

class iLinkBase
{
        protected :

                std::string uuid;

        public :
                iLinkBase(){}
                virtual ~iLinkBase() = 0;

                inline const std::string& getUuid() { return uuid; }
                virtual void setUuid(const std::string& uuid) = 0;

                /***********************************************************************/
                /*****************************  Buffer API *****************************/
                /***********************************************************************/

                virtual void activateBuffer() = 0;
                virtual void deactivateBuffer() = 0;
                virtual void copyBuffer() = 0;

                /***********************************************************************/
                /************************  Weight publish API  *************************/
                /***********************************************************************/

                virtual bool is_publish_active() = 0;
                virtual void active_publish(bool state) = 0;
                virtual void publish_message() = 0;
                
                virtual size_t w_type() = 0;
                virtual std::string w_type_name() = 0;

                /***********************************************************************/
                /************************  Input Accessor API  *************************/
                /***********************************************************************/

                virtual size_t i_type() = 0;
                virtual std::string i_type_name() = 0;
};

template<class I, class W>
class iLink : public iLinkBase
{
        protected :
                I cvalue;
                I const * input;
                I const * b_input;
                bool buffer;

                W weight;
                DataPublisher<W>* o_pub;

        public :
                iLink() : input(NULL), b_input(NULL), buffer(false){}
                iLink(I const * i) : input(i), b_input(NULL),buffer(false) {}
                virtual ~iLink(){}

                inline virtual void setUuid(const std::string& uuid)
                {
                        this->uuid = uuid;
			               
			std::string name = "ilink_"+getUuid();
                	ComInterface::setDefaultName(name);
                        o_pub->setPubName(name);
                }

                /***********************************************************************/
                /*************************  Constant Value API *************************/
                /***********************************************************************/

                virtual inline void setCValue(const I& cv){cvalue = cv; input = &cvalue;}
                virtual inline const I& getCValue(){ return cvalue;}
                virtual inline bool isSet(){return input!=NULL;}
                virtual inline bool isCValue(){return input==&cvalue;}

                /***********************************************************************/
                /*****************************  Buffer API *****************************/
                /***********************************************************************/

                virtual inline bool isBuffer(){return buffer;}
                virtual void activateBuffer()
                {
                        buffer = true;
                        cvalue = *input;
                        b_input = input;
                        input = &cvalue;
                }

                virtual void deactivateBuffer()
                {
                        buffer = false;
                        input = b_input;
                        b_input = NULL;
                }

                virtual void copyBuffer()
                {
                        if( buffer )
                        {
                                cvalue = *b_input;
                        }
                }

                /***********************************************************************/
                /************************  Weight publish API  *************************/
                /***********************************************************************/

                inline virtual  bool is_publish_active(){return o_pub->is_open();}

                virtual void active_publish(bool state)
                {
                        if( state )
                        {
                                if( o_pub != NULL  )
                                {
                                        if( !o_pub->is_open() )  o_pub->open();
                                }
                                else throw std::invalid_argument("Weight : failed to open output publisher");
                        }
                        else
                        {
                                if( o_pub != NULL)
                                {
                                        if( o_pub->is_open()) o_pub->close();
                                }
                        }
                }

                virtual void publish_message()
                {
                        if( o_pub->is_open() )
                        {
                                o_pub->setMessage(weight);
                                o_pub->publish();
                        }
                }

                /***********************************************************************/
                /************************  Input Accessor API  *************************/
                /***********************************************************************/

                typedef I type_i;
                virtual size_t i_type() { return typeid(I).hash_code();}
                virtual std::string i_type_name() { return typeid(I).name();}

                virtual inline void i(I const *i){input=i;}
                virtual inline const I& i() const {return *input;}

                /***********************************************************************/
                /************************  Weight Accessor API  *************************/
                /***********************************************************************/

                typedef W type_w;
                virtual size_t w_type() { return typeid(W).hash_code();}
                virtual std::string w_type_name() { return typeid(W).name();}

                inline virtual W& w() {return weight;}
                inline virtual void w(const W& w) { weight = w; }
};

#endif // __ILINK_BASE_H__
