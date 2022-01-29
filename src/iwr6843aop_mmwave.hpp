#ifndef SRC_IWR6843AOP_MMWAVE_HPP_
#define SRC_IWR6843AOP_MMWAVE_HPP_

namespace iwr6843
{

    struct MmwDemo_output_message_header_t
    {
        /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
        uint32_t    version;

        /*! @brief   Total packet length including header in Bytes */
        uint32_t    totalPacketLen;

        /*! @brief   platform type */
        uint32_t    platform;

        /*! @brief   Frame number */
        uint32_t    frameNumber;

        /*! @brief   Time in CPU cycles when the message was created. For XWR16xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
        uint32_t    timeCpuCycles;

        /*! @brief   Number of detected objects */
        uint32_t    numDetectedObj;

        /*! @brief   Number of TLVs */
        uint32_t    numTLVs;

        /*! @brief   Sub-frame Number (not used with XWR14xx) */
        uint32_t    subFrameNumber;
    };

}

#endif /* SRC_IWR6843AOP_MMWAVE_HPP_ */
