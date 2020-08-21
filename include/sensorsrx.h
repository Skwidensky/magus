#include <librealsense2/rs.hpp>
#include "rxcpp/rx.hpp"

namespace Magus
{
    rxcpp::subscriber<rs2::frameset> driver(int t, rs2::pipeline pipe);
}