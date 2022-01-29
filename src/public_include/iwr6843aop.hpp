#ifndef SRC_PUBLIC_INCLUDE_IWR6843AOP_HPP_
#define SRC_PUBLIC_INCLUDE_IWR6843AOP_HPP_

#include <radar_data.hpp>

#include <string>
#include <vector>

namespace iwr6843
{
	struct Iwr6843aop
	{
	    int init();
	    int configure(std::string configFile = "");

		/*Start and stop internal threads for continuously gathering data */
		int start();
		void stop();

		void getpointCloud(std::vector<radar::RadarPointCartesian>& radarData);
	};

}





#endif /* SRC_PUBLIC_INCLUDE_IWR6843AOP_HPP_ */
