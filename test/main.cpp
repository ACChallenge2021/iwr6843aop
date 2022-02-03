
#include <iwr6843aop.hpp>
#include <iostream>
#include <thread>

int main(int argc, char* argv[])
{

    iwr6843::Iwr6843aop iwr6843aop;

    if (!iwr6843aop.init())
    {
        std::cout << "Init failed.\n";
        return 0;
    }

    iwr6843aop.configure("6843AOP_3d_withRange.cfg");


    iwr6843aop.start();

    uint32_t counter = 5000;

    std::vector<radar::RadarPointCartesian> radarData;

    while (counter != 0)
    {
       iwr6843aop.getpointCloud(radarData);
       std::cout << "Frame: " << counter << "\n";
       for (auto& point : radarData)
       {
           std::cout << "x: " << point.x
                   << " y: " << point.y
                   << " z: " << point.z
                   << " V:" << point.velocity
                   << " S:" << point.snr
                   << " N:" << point.noise
                   << " M:" << point.magnitude
                   << "\n";
       }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        counter--;
    }

    iwr6843aop.stop();

	return 0;
}
