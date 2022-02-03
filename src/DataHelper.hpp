
#ifndef SRC_DATAHELPER_HPP_
#define SRC_DATAHELPER_HPP_

#include <radar_data.hpp>
#include <vector>

namespace iwr6843
{
    namespace DataHelper
    {
        void parseIncomingData(const std::vector<uint8_t>& currentFrame,
                             std::vector<radar::RadarPointCartesian>& currentPoints);
    }
} /* namespace iwr6843 */

#endif /* SRC_DATAHELPER_HPP_ */
