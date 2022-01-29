
//#include <DataUARTHandler.hpp>
#include <vector>
#include <regex>
#include <fstream>
#include <thread>
#include <chrono>
#include <thread>

#include <iwr6843aop.hpp>
#include <radar_data.hpp>
#include <serial/serial.h>
#include "TripleBuffer.h"
#include "iwr6843aop_mmwave.hpp"


namespace iwr6843
{
    const std::string iwr6843aopTag("10c4:ea70");
    const std::array<uint8_t, 8> magicWord = {2, 1, 4, 3, 6, 5, 8, 7};
    //const std::array<uint8_t, 8> magicWord = {1, 2, 3, 4, 5, 6, 7, 8};

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

    std::vector<std::string> uartports;

    std::atomic_uint8_t stopReadData(0);
    std::atomic_uint8_t stopParseData(0);

    std::unique_ptr<kria::TripleBuffer<std::vector<uint8_t>>> readBuffer =
        std::make_unique<kria::TripleBuffer<std::vector<uint8_t>>>();

    std::unique_ptr<kria::TripleBuffer<std::vector<radar::RadarPointCartesian>>> pointCloud =
            std::make_unique<kria::TripleBuffer<std::vector<radar::RadarPointCartesian>>>();

    std::thread thread_readData;
    std::thread thread_parseData;

    void find_iwr6843aop_ports (std::vector<std::string> &iwr6843_ports, std::string search_pattern)
    {
        std::vector<serial::PortInfo> devices_found = serial::list_ports ();

        std::vector<serial::PortInfo>::iterator iter = devices_found.begin ();

        while (iter != devices_found.end ())
        {
            serial::PortInfo device = *iter++;

            //printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );

            if (device.hardware_id.find (search_pattern) != std::string::npos)
            {
                printf ("(%s, %s, %s)\n", device.port.c_str (), device.description.c_str (),
                        device.hardware_id.c_str ());
                iwr6843_ports.emplace_back (device.port);
            }
        }
    }


    uint8_t sendSerialCommand(std::string port, std::string command, std::string& response)
    {
        /*Open Serial port and error check*/
        serial::Serial mySerialObject("", 115200, serial::Timeout::simpleTimeout(1000));
        mySerialObject.setPort(port);

        try
        {
                mySerialObject.open();
        }
        catch (std::exception &e1)
        {
            printf("iwr6843aop: Failed to open User serial port with error: %s\n", e1.what());
            printf("iwr6843aop: Waiting 20 seconds before trying again...\n");
            try
            {
                // Wait 20 seconds and try to open serial port again
                std::this_thread::sleep_for(std::chrono::seconds(20));
                mySerialObject.open();
            }
            catch (std::exception &e2)
            {
                printf("iwr6843aop: Failed second time to open User serial port, error: %s\n", e1.what());
                printf("iwr6843aop: Port could not be opened. Port is \"%s\" and baud rate is %d \n", port.c_str(), 115200);
                return 0;
            }
        }

        /*Read any previous pending response(s)*/
        while (mySerialObject.available() > 0)
        {
            mySerialObject.readline(response, 1024, ":/>");
            printf("iwr6843aop: Received (previous) response from sensor: '%s'\n", response.c_str());
            response = "";
        }

        /*Send out command received from the client*/
        printf("iwr6843aop: Sending command to sensor: '%s'\n", command.c_str());

        command.append("\n");

        int bytesSent = mySerialObject.write(command);

        /*Read output from mmwDemo*/
        mySerialObject.readline(response, 1024, ":/>");
        printf("iwr6843aop: Received response from sensor: '%s'\n", response.c_str());

        mySerialObject.close();


        return 1;

    }

    uint8_t applyRadarConfiguration(const std::string& configFile, const std::string& commandPort)
    {
        std::ifstream config (configFile);

        std::string configLine;

        std::string response;

        if (config.is_open ())
        {
            while (std::getline (config, configLine))
            {
                // Remove Windows carriage-return if present
                configLine.erase (std::remove (configLine.begin (), configLine.end (), '\r'), configLine.end ());
                // Ignore comment lines (first non-space char is '%') or blank lines
                if (!(std::regex_match (configLine, std::regex ("^\\s*%.*"))
                        || std::regex_match (configLine, std::regex ("^\\s*"))))
                {
                    if(sendSerialCommand(commandPort, configLine, response))
                    {
                        if (std::regex_search(response, std::regex("Done")))
                        {
                            //printf("iwr6843aop: Command successful (mmWave sensor responded with 'Done')\n");

                        }
                        else
                        {
                            printf("iwr6843aop: Command failed (mmWave sensor did not respond with 'Done')\n");
                            printf("iwr6843aop: Response: '%s'\n", response.c_str() );
                            return 0;
                        }
                    }
                    else
                    {
                        printf("iwr6843aop: Command failed (mmWave sensor did not respond with 'Done')\n");
                        printf("iwr6843aop: Response: '%s'\n", response.c_str() );
                        return 0;
                    }
                }
            }
        }
        return 1;
    }

    void readDataFromSensor(std::unique_ptr<kria::TripleBuffer<std::vector<uint8_t>>>& readBuffer,
                            std::atomic_uint8_t& stop, std::string dataPort, uint32_t baudRate)
    {

        printf("start reading data from sensor.\n");
        /*Open UART Port and error checking*/
        serial::Serial mySerialObject ("", baudRate, serial::Timeout::simpleTimeout (100));
        mySerialObject.setPort(dataPort);

        try
        {
            mySerialObject.open ();
        }

        catch (std::exception &e1)
        {
            printf ("iwr6843aop Read Thread: Failed to open Data serial dataPort with error: %s\n", e1.what ());
            printf ("iwr6843aop Read Thread: Waiting 20 seconds before trying again...\n");
            try
            {
                // Wait 20 seconds and try to open serial dataPort again
                std::this_thread::sleep_for(std::chrono::seconds(20));
                mySerialObject.open ();
            }

            catch (std::exception &e2)
            {
                printf ("iwr6843aop Read Thread: Failed second time to open Data serial dataPort, error: %s\n",
                           e1.what ());
                printf ("iwr6843aop Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d\n",
                           dataPort.c_str(), baudRate);
                stop = 1;
            }
        }

        if (mySerialObject.isOpen ())
            printf ("iwr6843aop Read Thread: Port is open\n");
        else
            printf ("iwr6843aop Read Thread: Port could not be opened\n");

        std::array<uint8_t, 8> last8Bytes = {0};

        /*Quick magicWord check to synchronize program with data Stream*/
        while((last8Bytes != magicWord) && stop == 0)
        {

            last8Bytes[0] = last8Bytes[1];
            last8Bytes[1] = last8Bytes[2];
            last8Bytes[2] = last8Bytes[3];
            last8Bytes[3] = last8Bytes[4];
            last8Bytes[4] = last8Bytes[5];
            last8Bytes[5] = last8Bytes[6];
            last8Bytes[6] = last8Bytes[7];
            mySerialObject.read(&last8Bytes[7], 1);
        }

        printf("iwr6843aop: Start reading...\n");
        std::vector<uint8_t>& currentFrame = readBuffer->currentWriteValue();

        auto runtime = std::chrono::microseconds{0}.count();

        auto start = std::chrono::high_resolution_clock::now();

        uint32_t frameCounter = 0;

        while(stop == 0)
        {
            /*Start reading UART data and writing to buffer while also checking for magicWord*/
            last8Bytes[0] = last8Bytes[1];
            last8Bytes[1] = last8Bytes[2];
            last8Bytes[2] = last8Bytes[3];
            last8Bytes[3] = last8Bytes[4];
            last8Bytes[4] = last8Bytes[5];
            last8Bytes[5] = last8Bytes[6];
            last8Bytes[6] = last8Bytes[7];
            mySerialObject.read(&last8Bytes[7], 1);

            currentFrame.emplace_back(last8Bytes[7]);  //push byte onto buffer

            /*If a magicWord is found wait for sorting to finish and switch buffers*/
            if( last8Bytes == magicWord)
            {
                //printf("read next frame.\n");
                readBuffer->swapWriteBuffer();
                currentFrame = readBuffer->currentWriteValue();
                auto end = std::chrono::high_resolution_clock::now();
                runtime += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                start = end;
                frameCounter++;
            }
        }

        mySerialObject.close();

        if (frameCounter != 0)
        {
            runtime /= frameCounter;
            float framerate = 1.0f / (runtime / 1000.0f / 1000.0f);
            printf ("iwr6843aop: Read thread frame rate for %d frames: %f fps.\n", frameCounter, framerate);
        }

    }


    uint8_t parseHeader(std::vector<uint8_t>& currentFrame,
                     uint32_t& currentDataPos,
                     MmwDemo_output_message_header_t& header)
    {
        //get version (4 bytes)
        memcpy( &header.version, &currentFrame.data()[currentDataPos], sizeof(header.version));
        currentDataPos += ( sizeof(header.version) );

        //get totalPacketLen (4 bytes)
        memcpy( &header.totalPacketLen, &currentFrame.data()[currentDataPos], sizeof(header.totalPacketLen));
        currentDataPos += ( sizeof(header.totalPacketLen) );

        //get platform (4 bytes)
        memcpy( &header.platform, &currentFrame.data()[currentDataPos], sizeof(header.platform));
        currentDataPos += ( sizeof(header.platform) );

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

        return 1;
    }

    void parseIncommingData(std::unique_ptr<kria::TripleBuffer<std::vector<uint8_t>>>& readBuffer,
                            std::unique_ptr<kria::TripleBuffer<std::vector<radar::RadarPointCartesian>>>& pointCloud,
                            std::atomic_uint8_t& stop)
    {

        printf("Start parse incomming data.\n");

        ParserState parserState = ParserState::READ_HEADER;

        MmwDemo_output_message_header_t mmwheader;
        uint32_t currentDataPos = 0;

        while(stop == 0)
        {
            std::vector<radar::RadarPointCartesian>& currentPoints = pointCloud->currentWriteValue();

            if (readBuffer->swapReadBuffer())
            {
                std::vector<uint8_t>& currentFrame = readBuffer->read();
                switch(parserState)
                {
                    case ParserState::READ_HEADER:
                    {
                        //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
                        if(currentFrame.size() < 12)
                        {
                           //printf("Error Frame size\n");
                           parserState = ParserState::SWAP_BUFFERS;
                           break;
                        }

                        uint8_t parseResult = parseHeader(currentFrame, currentDataPos, mmwheader);

                        //if packet lengths do not match, throw it away
                        if(parseResult && mmwheader.totalPacketLen == currentFrame.size() )
                        {
                           parserState = ParserState::CHECK_TLV_TYPE;
                        }
                        else
                        {
                            parserState = ParserState::SWAP_BUFFERS;
                        }
                    }
                        break;

                    case ParserState::READ_OBJ_STRUCT:
                        break;
                    case ParserState::READ_SIDE_INFO:
                        break;
                    case ParserState::READ_LOG_MAG_RANGE:
                        break;
                    case ParserState::READ_NOISE:
                        break;
                    case ParserState::READ_AZIMUTH:
                        break;
                    case ParserState::READ_DOPPLER:
                        break;
                    case ParserState::READ_STATS:
                        break;
                    case ParserState::CHECK_TLV_TYPE:
                        parserState = ParserState::SWAP_BUFFERS;
                        break;
                    case ParserState::SWAP_BUFFERS:
                        pointCloud->swapWriteBuffer();
                        currentDataPos = 0;
                        parserState = ParserState::READ_HEADER;
                        break;
                    default: break;
                }
            }
            //printf("header obj: %d\n", mmwheader.numDetectedObj);
        }
    }


	int Iwr6843aop::configure(std::string configFile)
	{

	    if (uartports.size() < 1)
	    {
	        if (!init())
	            return 0;
	    }

	    applyRadarConfiguration(configFile, uartports.at(0));

	    return 1;
	}

	int Iwr6843aop::init()
	{
	    uartports.clear();

        find_iwr6843aop_ports(uartports, iwr6843aopTag);

        if (uartports.size() < 1)
            return 0;

        return 1;

	}

	int Iwr6843aop::start()
	{

        thread_readData = std::thread(&readDataFromSensor, std::ref(readBuffer),
                                      std::ref(stopReadData),
                                      uartports.at(1), 921600);
        thread_parseData = std::thread(&parseIncommingData, std::ref(readBuffer),
                                             std::ref(pointCloud),
                                              std::ref(stopParseData));
        return 1;
	}

	void Iwr6843aop::stop()
	{
	    if (thread_readData.joinable())
	    {
	        printf("Request read thread to stop...\n");
	        stopReadData = 1;
	    }
	    if (thread_parseData.joinable())
	    {
	        printf("Request parsing thread to stop...\n");
	        stopParseData = 1;
	    }

	    thread_readData.join();
	    thread_parseData.join();

	    std::string response;
        sendSerialCommand(uartports.at(0), "sensorStop", response);

        if (!std::regex_search(response, std::regex("Done")))
        {
            printf("iwr6843aop: Command failed (mmWave sensor did not respond with 'Done')\n");
            printf("iwr6843aop: Response: '%s'\n", response.c_str() );
        }

	    printf("stopped.\n");
	}


	void Iwr6843aop::getpointCloud(std::vector<radar::RadarPointCartesian>& radarData)
	{

	}

}
