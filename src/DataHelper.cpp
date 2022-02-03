#include <DataHelper.hpp>
#include <cstring>
#include <algorithm>
#include "iwr6843aop_mmwave.hpp"


namespace iwr6843
{
    namespace DataHelper
    {
        enum class ParserState {
             READ_HEADER,
             CHECK_TLV_TYPE,
             READ_OBJ_STRUCT,
             READ_LOG_MAG_RANGE,
             READ_NOISE,
             READ_AZIMUTH,
             READ_DOPPLER,
             READ_STATS,
             SWAP_BUFFERS,
             READ_SIDE_INFO
         };

        uint8_t parseHeader(const std::vector<uint8_t>& currentFrame,
                         uint32_t& currentDataPos,
                         MmwDemo_output_message_header_t& header)
        {

            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
            if(currentFrame.size() < 12)
            {
                return 0;
            }
            //printf("1: %u\n", static_cast<uint32_t>(currentFrame.size()));
            //get version (4 bytes)
            std::memcpy( &header.version, &currentFrame.data()[currentDataPos], sizeof(header.version));
            currentDataPos += ( sizeof(header.version) );

            //get totalPacketLen (4 bytes)
            std::memcpy( &header.totalPacketLen, &currentFrame.data()[currentDataPos], sizeof(header.totalPacketLen));
            currentDataPos += ( sizeof(header.totalPacketLen) );

            //get platform (4 bytes)
            std::memcpy( &header.platform, &currentFrame.data()[currentDataPos], sizeof(header.platform));
            currentDataPos += ( sizeof(header.platform) );
            //printf("2: %u\n", header.platform);
            //if packet doesn't have correct header size (which is based on platform), throw it away
            //  (does not include magicWord since it was already removed)
            uint32_t headerSize;

            if ((header.platform & 0xFFFF) == 0x1443)  // platform is xWR1443)
            {
               headerSize = 7 * 4;  // xWR1443 SDK demo header does not have subFrameNumber field
            }
            else
            {
               headerSize = 8 * 4;  // header includes subFrameNumber field
            }

            if(currentFrame.size() < headerSize)
            {
                //printf("Framesize: %u\n", static_cast<uint32_t>(currentFrame.size()));
                return 0;
            }

            //get frameNumber (4 bytes)
            memcpy( &header.frameNumber, &currentFrame.data()[currentDataPos], sizeof(header.frameNumber));
            currentDataPos += ( sizeof(header.frameNumber) );

            //get timeCpuCycles (4 bytes)
            memcpy( &header.timeCpuCycles, &currentFrame.data()[currentDataPos], sizeof(header.timeCpuCycles));
            currentDataPos += ( sizeof(header.timeCpuCycles) );

            //get numDetectedObj (4 bytes)
            memcpy( &header.numDetectedObj, &currentFrame.data()[currentDataPos], sizeof(header.numDetectedObj));
            currentDataPos += ( sizeof(header.numDetectedObj) );

            //get numTLVs (4 bytes)
            memcpy( &header.numTLVs, &currentFrame.data()[currentDataPos], sizeof(header.numTLVs));
            currentDataPos += ( sizeof(header.numTLVs) );

            //get subFrameNumber (4 bytes) (not used for XWR1443)
            if((header.platform & 0xFFFF) != 0x1443)
            {
               memcpy( &header.subFrameNumber, &currentFrame.data()[currentDataPos], sizeof(header.subFrameNumber));
               currentDataPos += ( sizeof(header.subFrameNumber) );
            }

            //printf("ObjN: %u\n", header.numDetectedObj);
            return 1;
        }

        ParserState checkTlvType(const std::vector<uint8_t>& currentFrame,
                             uint32_t& currentDataPos,
                             MmwDemo_output_message_header_t& header,
                             uint32_t& tlvCount,
                             uint32_t& tlvLen)
        {
            ParserState parserState = ParserState::CHECK_TLV_TYPE;
            if(tlvCount++ >= header.numTLVs)  // Done parsing all received TLV sections
            {
                  // Publish detected object pointcloud
    //              if (mmwData.numObjOut > 0)
    //              {
    //                  j = 0;
    //                  for (i = 0; i < mmwData.numObjOut; i++)
    //                  {
    //                      // Keep point if elevation and azimuth angles are less than specified max values
    //                      // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
    //                      if (((maxElevationAngleRatioSquared == -1) ||
    //                           (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
    //                                                                          RScan->points[i].y * RScan->points[i].y)
    //                            ) < maxElevationAngleRatioSquared)
    //                          ) &&
    //                          ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
    //                                  (RScan->points[i].x != 0)
    //                         )
    //                      {
    //                          //ROS_INFO("Kept point");
    //                          // copy: points[i] => points[j]
    //                          memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
    //                          j++;
    //                      }
    //                  }
    //                  mmwData.numObjOut = j;  // update number of objects as some points may have been removed
    //
    //                  // Resize point cloud since some points may have been removed
    //                  RScan->width = mmwData.numObjOut;
    //                  RScan->points.resize(RScan->width * RScan->height);
    //
    //                  //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
    //                  //ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
    //
    //                  DataUARTHandler_pub.publish(RScan);
    //              }

                  //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                parserState = ParserState::SWAP_BUFFERS;
            }
            else  // More TLV sections to parse
            {
                TlvTypes tlvType = TlvTypes::MMWDEMO_OUTPUT_MSG_NULL;
                //get tlvType (32 bits) & remove from queue
                std::memcpy( &tlvType, &currentFrame.data()[currentDataPos], sizeof(tlvType));
                currentDataPos += ( sizeof(tlvType) );

                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));

                //get tlvLen (32 bits) & remove from queue
                std::memcpy( &tlvLen, &currentFrame.data()[currentDataPos], sizeof(tlvLen));
                currentDataPos += ( sizeof(tlvLen) );

                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));

                //ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);

                switch(tlvType)
                {
                    case TlvTypes::MMWDEMO_OUTPUT_MSG_NULL:
                        break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                        //ROS_INFO("DataUARTHandler Sort Thread : Object TLV");
                        parserState = ParserState::READ_OBJ_STRUCT;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                        //ROS_INFO("DataUARTHandler Sort Thread : Range TLV");
                        parserState = ParserState::READ_LOG_MAG_RANGE;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                        //ROS_INFO("DataUARTHandler Sort Thread : Noise TLV");
                        parserState = ParserState::READ_NOISE;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                        //ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
                        parserState = ParserState::READ_AZIMUTH;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                        //ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
                        parserState = ParserState::READ_DOPPLER;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_STATS:
                        //ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
                        parserState = ParserState::READ_STATS;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                        //ROS_INFO("DataUARTHandler Sort Thread : Side info TLV");
                        parserState = ParserState::READ_SIDE_INFO;
                    break;

                    case TlvTypes::MMWDEMO_OUTPUT_MSG_MAX:
                        //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                        parserState = ParserState::READ_HEADER;
                    break;
                    default: break;
                  }
              }
            return parserState;
        }

        uint8_t readObject(const std::vector<uint8_t>& currentFrame,
                           std::vector<radar::RadarPointCartesian>& currentPoints,
                           uint32_t& currentDataPos,
                           MmwDemo_output_message_header_t& header)
        {
            // CHECK_TLV_TYPE code has already read tlvType and tlvLen
            //i = 0;
            //offset = 0;
            uint32_t numObjOut = 0;
            if (((header.version >> 24) & 0xFF) < 3)  // SDK version is older than 3.x
            {
                printf("Error: SDK version older than 3.x are not supported\n");
                return 0;
            }
            else  // SDK version is at least 3.x
            {
                numObjOut = header.numDetectedObj;
                //printf("ObjN: %u\n", numObjOut );
            }

            while( numObjOut != 0 )
            {
                radar::RadarPointCartesian point;
                //get object x-coordinate (meters)
                std::memcpy( &point.x, &currentFrame.data()[currentDataPos], sizeof(point.x));
                currentDataPos += ( sizeof(point.x) );

                //get object y-coordinate (meters)
                std::memcpy( &point.y, &currentFrame.data()[currentDataPos], sizeof(point.y));
                currentDataPos += ( sizeof(point.y) );

                //get object z-coordinate (meters)
                std::memcpy( &point.z, &currentFrame.data()[currentDataPos], sizeof(point.z));
                currentDataPos += ( sizeof(point.z) );

                //get object velocity (m/s)
                std::memcpy( &point.velocity, &currentFrame.data()[currentDataPos], sizeof(point.velocity));
                currentDataPos += ( sizeof(point.velocity) );

                currentPoints.emplace_back(point);
                // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
                numObjOut--;
            }
            return 1;
        }

        inline uint32_t readNoise(uint32_t& tlvLen)
        {
            uint32_t i = 0;

            while (i++ < tlvLen - 1)
            {
                 //ROS_INFO("DataUARTHandler Sort Thread : Parsing Side Info i=%d and tlvLen = %u", i, tlvLen);
            }
            return tlvLen;
        }

        void readSideInfo(const std::vector<uint8_t>& currentFrame,
                               std::vector<radar::RadarPointCartesian>& currentPoints,
                               uint32_t& currentDataPos,
                               uint32_t& tlvLen)
        {
            // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
            if (currentPoints.size() > 0)
            {
                std::for_each(currentPoints.begin(), currentPoints.end(), [currentFrame, &currentDataPos](radar::RadarPointCartesian &point)
                  {
                       //get snr (unit is 0.1 steps of dB)
                       std::memcpy( &point.snr, &currentFrame.data()[currentDataPos], sizeof(point.snr));
                       currentDataPos += ( sizeof(point.snr) );

                       //get noise (unit is 0.1 steps of dB)
                       std::memcpy( &point.noise, &currentFrame.data()[currentDataPos], sizeof(point.noise));
                       currentDataPos += ( sizeof(point.noise) );
                  });

            }
            else  // else just skip side info section if we have not already received and parsed detected obj list
            {
              currentDataPos += readNoise(tlvLen);
            }
        }

        void readLogMagRange(const std::vector<uint8_t>& currentFrame,
                             std::vector<radar::RadarPointCartesian>& currentPoints,
                             uint32_t& currentDataPos,
                             uint32_t& tlvLen)
        {
            // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
            if (currentPoints.size() > 0)
            {
                std::for_each(currentPoints.begin(), currentPoints.end(), [currentFrame, &currentDataPos](radar::RadarPointCartesian &point)
                  {
                       //get snr (unit is 0.1 steps of dB)
                       std::memcpy( &point.magnitude, &currentFrame.data()[currentDataPos], sizeof(point.magnitude));
                       currentDataPos += ( sizeof(point.magnitude) );
                  });

            }
            else  // else just skip side info section if we have not already received and parsed detected obj list
            {
              currentDataPos += readNoise(tlvLen);
            }
        }


        void parseIncomingData(const std::vector<uint8_t>& currentFrame,
                               std::vector<radar::RadarPointCartesian>& currentPoints)
        {
            ParserState parserState = ParserState::READ_HEADER;

            MmwDemo_output_message_header_t mmwheader;
            uint32_t currentDataPos = 0;
            uint16_t numObjOut = 0;
            uint32_t tlvCount = 0;
            uint32_t tlvLen = 0;

            if(currentFrame.size() == 0)
            {
                printf("Empty buffer\n");
                parserState = ParserState::SWAP_BUFFERS;
            }

            while (parserState != ParserState::SWAP_BUFFERS)
            {
                if (parserState == ParserState::READ_HEADER)
                {
                    //printf("READ_HEADER: %u\n", static_cast<uint32_t>(currentFrame.size()));
                    uint8_t parseResult = parseHeader(currentFrame, currentDataPos, mmwheader);

                    //if packet lengths do not match, throw it away
                    if(parseResult && mmwheader.totalPacketLen == currentFrame.size() )
                    {
                       parserState = ParserState::CHECK_TLV_TYPE;
                       //printf("CHECK_TLV_TYPE");
                    }
                    else
                    {
                        parserState = ParserState::SWAP_BUFFERS;
                    }
                }

                if (parserState == ParserState::CHECK_TLV_TYPE)
                {
                    parserState = checkTlvType(currentFrame, currentDataPos, mmwheader,
                                             tlvCount, tlvLen);
                    //printf("CHECK_TLV_TYPE next: %d\n", static_cast<int>(parserState));
                }
                if (parserState == ParserState::READ_OBJ_STRUCT)
                {
                    uint8_t result = readObject(currentFrame, currentPoints, currentDataPos,
                                           mmwheader);
                    if (result)
                    {
                        parserState = ParserState::CHECK_TLV_TYPE;
                    }
                    else
                    {
                        parserState = ParserState::SWAP_BUFFERS;
                    }
                }

                if (parserState ==  ParserState::READ_SIDE_INFO)
                {
                    readSideInfo(currentFrame, currentPoints, currentDataPos, tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }

                if (parserState == ParserState::READ_LOG_MAG_RANGE)
                {
                    readLogMagRange(currentFrame, currentPoints, currentDataPos,tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }

                if (parserState == ParserState::READ_NOISE)
                {
                    currentDataPos += readNoise(tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }

                if (parserState == ParserState::READ_AZIMUTH)
                {
                    currentDataPos += readNoise(tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }

                if (parserState == ParserState::READ_DOPPLER)
                {
                    currentDataPos += readNoise(tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }

                if (parserState == ParserState::READ_STATS)
                {
                    currentDataPos += readNoise(tlvLen);
                    parserState = ParserState::CHECK_TLV_TYPE;
                }
            }
        }
    } /* namespace DataHelper */

} /* namespace iwr6843 */
