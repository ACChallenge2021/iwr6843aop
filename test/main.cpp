
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

    iwr6843aop.init();
    iwr6843aop.configure("6843AOP_3d.cfg");


    iwr6843aop.start();

    uint32_t counter = 30;

    while (counter != 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        counter--;
    }

    iwr6843aop.stop();

	return 0;
}
