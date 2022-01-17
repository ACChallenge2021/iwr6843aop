#ifndef SRC_PUBLIC_INCLUDE_IWR6843AOP_H_
#define SRC_PUBLIC_INCLUDE_IWR6843AOP_H_

#include <string>

namespace kria
{

	struct Iwr6843aop
	{
		int configure(std::string configFile = "");

		void getpointCloud();
	};

}





#endif /* SRC_PUBLIC_INCLUDE_IWR6843AOP_H_ */
