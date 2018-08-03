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

#ifndef __ILINK_BASE_H__
#define __ILINK_BASE_H__

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
                        o_pub->setPubName("ilink_"+getUuid());
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
