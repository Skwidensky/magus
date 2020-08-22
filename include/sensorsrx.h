#include <chrono>
#include <librealsense2/rs.hpp>
#include "rxcpp/rx.hpp"

namespace Magus
{
    rxcpp::observable<rs2::frameset> streamdriver(int t, rs2::pipeline pipe, rs2::colorizer color_map);
    class streamhandler
    {
    public:
        streamhandler();
        double handleD435Frameset(rs2::frameset);
        double handleT265Frameset(rs2::frameset);
    };
} // namespace Magus