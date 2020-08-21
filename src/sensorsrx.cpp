#include <sensorsrx.h>
#include <thread>
#include <memory>
#include <chrono>

namespace rx = rxcpp;

namespace Magus
{
    template <class Obs>
    class sensordriver
    {
    public:
        sensordriver(Obs o, rs2::pipeline pipe, int t) : o_(o), pipe_(pipe), t_(t) {}
        void listen()
        {
            for (;;)
            {
                if (!o_.is_subscribed())
                {
                    return;
                }
                o_.on_next(pipe_.wait_for_frames());
                if (!o_.is_subscribed())
                {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::seconds(t_));
            }
        }
        Obs o_;
        rs2::pipeline pipe_;
        int t_;
    };

    template <typename T>
    sensordriver<T> CreateDriver(const T &obs, rs2::pipeline pipe, int t)
    {
        return sensordriver<T>(obs, pipe, t);
    }

    rx::subscriber<rs2::frameset> driver(int t, rs2::pipeline pipe)
    {
        return rx::sources::create<rs2::frameset>([t, pipe](auto out) {
            std::thread th([t, pipe, out]() {
                auto a = CreateDriver(out, pipe, t);
                a.listen();
            });
            th.detach();
            return out.get_subscription();
        });
    }

    class complextask
    {
    public:
        complextask(int t) : t_(std::chrono::seconds(t)){};
        double calculate(rs2::frameset fs)
        {
            std::this_thread::sleep_for(t_);
            return fs.get_timestamp();
        }
        std::chrono::seconds t_;
    };

}