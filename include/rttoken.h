/*
Copyright Enacted Robotics

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

#ifndef __RT_TOKEN_H__
#define __RT_TOKEN_H__

#include "runner.h"
#include "util.h"

#define CONV_S_TO_MS 1000000

const std::string second = "s";
const std::string ms = "ms";
const std::string hertz = "Hz";

class RtToken : public Runner
{
        private :
                // In second
                double period;
		
		//TODO : pour l'instant pas de gestion de rebouclage du compteur
                double means;
                unsigned long long nbrun;

                Graph::vertex_descriptor rt_node;

	public : 

		// Period in second
		RtToken(double period) : Runner(),period(period),means(0),nbrun(0)  {}
		RtToken(double value, std::string unit) : Runner(),means(0),nbrun(0) { setToken(value, unit);}
                virtual ~RtToken() {}

		inline void setRtNode( Graph::vertex_descriptor rt_node ) { this->rt_node = rt_node;}

		// Frequency in Hz
		inline void setFrequency( double frequency ) { period = convert_period_frequency(frequency);}
		
		// Period in second
		inline void setPeriod( double period ) {this->period = period;}
		
		// Period in second
		inline void setMsPeriod( double period ) {this->period = convert_ms_to_s(period);}

		inline double getPeriod(){return period;}
		inline double getMsPeriod(){return  convert_s_to_ms(period);}
		inline double getFrequency(){return  convert_period_frequency(period);}

		void setToken(double value, std::string unit);
		virtual void exec();
}; 

#endif // __RT_TOKEN_H__
