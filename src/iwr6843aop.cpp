
#include <iwr6843aop.h>

#include <serial/serial.h>
#include <vector>
#include <regex>
#include <fstream>
#include <thread>
#include <chrono>

namespace kria
{
    const std::string iwr6843aopTag("10c4:ea70");

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


	int Iwr6843aop::configure(std::string configFile)
	{
	    std::vector<std::string> iwr6843ports;

	    find_iwr6843aop_ports(iwr6843ports, iwr6843aopTag);

	    if (iwr6843ports.size() < 1)
	        return 0;

	    applyRadarConfiguration(configFile, iwr6843ports.at(0));

	    std::string response;
	    sendSerialCommand(iwr6843ports.at(0), "sensorStop", response);

	    if (!std::regex_search(response, std::regex("Done")))
        {
	        printf("iwr6843aop: Command failed (mmWave sensor did not respond with 'Done')\n");
	        printf("iwr6843aop: Response: '%s'\n", response.c_str() );
        }


	    return 1;
	}


}
