#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "src/solutions/GA.h"
#include "src/solutions/GA_4sw_ring.h"
//#include "src/solutions/GA_line_2_2.h"
#include "src/solutions/GA_line_2_2.h"


//using std::vector;
namespace spd = spdlog;

int main(int argc, char *argv[]) {
//    if (argc < 5) {
//        log_info("Version %d.%d\n", Client_VERSION_MAJOR, Client_VERSION_MINOR);
//        log_info("Usage: %s work_mode.\n", argv[0]);
//        log_info("server address: %s.\n", argv[1]);
//    }
    try {
        // Console logger with color
        auto console = spd::stdout_color_mt("console");
//        console->set_pattern("%D %H:%M:%S %l %g %# %v");
        // Customize msg format for all messages
        spd::set_pattern("[%H:%M:%S][%^%l%$] [thread %t] %v");
        spd::set_level(spd::level::info); //Set global log level to info

        // Use global registry to retrieve loggers
        spd::get("console")->info(
                "loggers can be retrieved from a global registry using the spdlog::get(logger_name) function");

        // Create basic file logger (not rotated)
        auto my_logger = spd::basic_logger_mt("basic_logger", "schedule.txt");
        my_logger->info("Some log message");



        console->set_level(spd::level::debug); // Set specific logger's log level

        // Compile time log levels
        // define SPDLOG_DEBUG_ON or SPDLOG_TRACE_ON
        SPDLOG_TRACE(console, "Enabled only #ifdef SPDLOG_TRACE_ON..{} ,{}", 1, 3.23);
        SPDLOG_DEBUG(console, "Enabled only #ifdef SPDLOG_DEBUG_ON.. {} ,{}", 1, 3.23);

//        openGACal();
//        Small4SwRing::openGACal();
        GA_line_2_2::openGACal();
//        GA_line_2_1::openGACal();


        // Release and close all loggers
        spd::drop_all();
    } catch (const spd::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
