
#include <DataHelper.hpp>
#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{

    using namespace iwr6843;
    std::vector<radar::RadarPointCartesian> radarData;

    std::vector<uint8_t> radarFrame;

    std::fstream input("radarFrame.bin", input.binary | input.in);
    if (input.is_open())
    {
        while (!input.eof())
        //for (int i = 0; i < 64; ++i)
        {
            char s;
            input.read(&s, 1);
            if (!input.eof())
            {
                radarFrame.emplace_back(s);
            }
        }
        input.close();
    }

    std::cout << "Size: " << radarFrame.size() << "\n";
    DataHelper::parseIncomingData(radarFrame, radarData);

    for (auto& point : radarData)
   {
       std::cout << "x: " << point.x
               << " y: " << point.y
               << " z: " << point.z
               << " V:" << point.velocity
               << "\n";
   }




	return 0;
}
