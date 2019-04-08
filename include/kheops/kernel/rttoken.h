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


#ifndef __RT_TOKEN_H__
#define __RT_TOKEN_H__

#include <condition_variable>
#include "kheops/kernel/runner.h"
#include "kheops/util/util.h"
#include "kheops/kernel/publisher.h"

#define CONV_S_TO_MS 1000000

const std::string second = "s";
const std::string ms = "ms";
const std::string hertz = "Hz";

class RtToken : public Runner
{
        private :
		std::string uuid;

                // In second
                double period;
		
		bool publish;
	
		OscilloPublisher *o_pub;
		RtTokenOutputPublisher *rt_pub;	


	public : 

		// Period in second
		RtToken();
		RtToken(double period);
		RtToken(double value, std::string unit);
                virtual ~RtToken();

                RtToken(const RtToken&) = delete;

		inline const std::string& getUuid() { return uuid; }
                inline void setUuid(const std::string& uuid  ) { this->uuid = uuid;}

		// Frequency in Hz
		inline void setFrequency(double frequency) {period=convert_period_frequency(frequency);}
		
		// Period in second
		inline void setPeriod( double period ) {this->period = period;}
		
		// Period in second
		inline void setMsPeriod( double period ) {this->period = convert_ms_to_s(period);}

		inline double getPeriod(){return period;}
		inline double getMsPeriod(){return  convert_s_to_ms(period);}
		inline double getFrequency(){return  convert_period_frequency(period);}

		void setToken(double value, std::string unit);
		virtual void exec();
		
		void sync_all();

		void set_rt_pub_name(const std::string& name);
		inline bool is_publish_active(){return publish;}
		void active_publish(bool state);
		void publish_message();

		void set_oscillo_pub_name(const std::string& name);
		void active_oscillo(bool state);
		void publish_oscillo();

}; 

#endif // __RT_TOKEN_H__
