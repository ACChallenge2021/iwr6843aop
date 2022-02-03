#ifndef SRC_PUBLIC_INCLUDE_RADAR_DATA_HPP_
#define SRC_PUBLIC_INCLUDE_RADAR_DATA_HPP_

#include <cstdint>

namespace radar
{
    /**
    * @brief
    * Point cloud definition in Cartesian coordinate system - floating point format
    *
    */
    struct RadarPointCartesian
    {
            /*! @brief x - coordinate in meters */
            float x;

            /*! @brief y - coordinate in meters */
            float y;

            /*! @brief z - coordinate in meters */
            float z;

            /*! @brief Doppler velocity estimate in m/s. Positive velocity means target
             * is moving away from the sensor and negative velocity means target
             * is moving towards the sensor. */
            float velocity;

            /*! @brief snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
            int16_t snr;

            /*! @brief y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
            int16_t noise;

            /*! @brief at 0th Doppler (stationary objects). The point represent the sum of log2 magnitudes of
             * received antennas expressed in Q9 format.*/
            uint16_t magnitude;

    };
}



#endif /* SRC_PUBLIC_INCLUDE_RADAR_DATA_HPP_ */
