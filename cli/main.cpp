#include <iostream>
#include <sstream>
#include <string>
#include "version.h"
#include "cli_parser.hpp"
#include "sensors.h"
#include "processing.h"
#include "logger_init.h"

using namespace std;
using namespace stlplus;
using namespace Magus;

int main()
{
    cli_definitions cli_defs = {
        {
            "help",
            cli_kind_t::cli_switch_kind,
            cli_mode_t::cli_single_mode,
            "help",
        },
        {
            "log",
            cli_kind_t::cli_value_kind,
            cli_mode_t::cli_single_mode,
            "log",
        },
        {
            "",
            cli_kind_t::cli_value_kind,
            cli_mode_t::cli_single_mode,
            "name",
        },
    };

    message_handler messages(std::cerr);
    string log_file = "debug.log";
    InitLogger(log_file);
    initModels();
    initializeSensors();
    auto loop = rxcpp::observe_on_event_loop();
    Magus::streamhandler sh;

    /* Real-time asynchronous handling of data for the D435 and T265 cameras */
    observableD435()
        // slow down Observable
        .sample_with_time(std::chrono::duration<int, std::milli>(1000))
        .observe_on(loop)
        .map([&](rs2::frameset fs) { return sh.handleD435Frameset(fs); })
        .subscribe();

    // on command, collect a batch of frames to perform inference on

    // observableT265()
    //     // slow down Observable
    //     .sample_with_time(std::chrono::duration<int, std::milli>(1000))
    //     .observe_on(loop)
    //     .map([&](rs2::frameset fs) { return sh.handleT265Frameset(fs); })
    //     .subscribe();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
