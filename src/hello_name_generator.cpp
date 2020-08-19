#include "hello_name_generator.h"
#include "plog/Log.h"

namespace Magus
{
    std::string GenerateHelloName(const std::string &name)
    {
        LOG_DEBUG << "Name = " << name;
        return "Hello World and " + name;
    }
}
