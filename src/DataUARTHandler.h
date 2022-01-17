#ifndef SRC_DATAUARTHANDLER_H_
#define SRC_DATAUARTHANDLER_H_

#include <string>


namespace kria
{
    
    class DataUARTHandler
    {
        public:
            DataUARTHandler (std::string dataPort);

            /*User callable function to start the handler's internal threads*/
            void start();

        private:
            std::string port;

    };

} /* namespace kria */

#endif /* SRC_DATAUARTHANDLER_H_ */
