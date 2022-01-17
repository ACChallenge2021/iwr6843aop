#include <DataUARTHandler.h>
#include <TripleBuffer.h>
#include <vector>

namespace kria
{
    TripleBuffer<std::vector<uint8_t>> sensorData;
    
    DataUARTHandler::DataUARTHandler (std::string dataPort)
            : port(dataPort)
    {
        

    }

    void DataUARTHandler::start()
    {

    }

} /* namespace kria */
