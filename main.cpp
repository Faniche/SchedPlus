#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "src/solutions/GA.h"
#include "src/solutions/GA_4sw_ring.h"
#include "src/solutions/GA_line_2_1.h"
#include "src/solutions/GA_line_2_2.h"
#include "lib/CLI11/include/CLI/CLI.hpp"

namespace spd = spdlog;

int main(int argc, char **argv) {
    CLI::App app{"Schedplus: based on GA to schedule time sensitive flows."};
    int option_topology = 0;
    std::string topology_description = "Topology index:\n\t1: line_1sw_2es\n\t2: line_2sw_2es\n\t3: ring_4sw";
    app.add_option("-t, --topology", option_topology, topology_description);
    std::string option_ned_file;
    app.add_option("-n, --ned", option_ned_file, "Net description file name");
    int option_flow_number = 2;
    app.add_option("-f, --flows", option_flow_number, "The number of flow");

    try {
        CLI11_PARSE(app, argc, argv);
    } catch (CLI::ParseError error) {
        app.exit(error);
    }
    try {
        auto console = spd::stdout_color_mt("console");
        spd::set_pattern("[%H:%M:%S][%^%l%$] [thread %t] %v");
        spd::set_level(spd::level::info); //Set global log level to info

        // Create basic file logger (not rotated)
        auto my_logger = spd::basic_logger_mt("basic_logger", "schedule.txt");
        my_logger->info("Some log message");

        console->set_level(spd::level::debug); // Set specific logger's log level

        // Compile time log levels
        // define SPDLOG_DEBUG_ON or SPDLOG_TRACE_ON
        SPDLOG_TRACE(console, "Enabled only #ifdef SPDLOG_TRACE_ON..{} ,{}", 1, 3.23);
        SPDLOG_DEBUG(console, "Enabled only #ifdef SPDLOG_DEBUG_ON.. {} ,{}", 1, 3.23);

        std::vector<Node *> nodes;
        std::vector<Node *> esList;
        std::vector<Node *> swList;
        std::map<node_idx , Node *> nodeMap;
        std::vector<DirectedLink> links;
        std::vector<Flow> flows;

        if (option_topology > 0) {
            spdlog::get("console")->info("Topology index: {}", option_topology);
            switch (option_topology) {
                case 1:
                    GA_line_2_1::openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
                case 2:
                    GA_line_2_2::openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
                case 3:
                    Small4SwRing::openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
                default:
                    openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
            }
        }
        MyFunctions myobject("/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small",
                             nodes, esList, swList, nodeMap, links, flows);
        std::string delete_file = "rm /home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small/*";
        system(delete_file.c_str());
        EA::Chronometer timer;
        timer.tic();

        GA_Type ga_obj;
        ga_obj.problem_mode = EA::GA_MODE::NSGA_III;
        ga_obj.multi_threading = true;
        ga_obj.dynamic_threading = true;
        ga_obj.verbose = true;
        ga_obj.population = 20;
        ga_obj.generation_max = 100;

        using std::bind;
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;

        ga_obj.calculate_MO_objectives = bind(&MyFunctions::calculate_MO_objectives, &myobject, _1);
        ga_obj.init_genes =  bind(&MyFunctions::init_genes, &myobject, _1, _2);
        ga_obj.eval_solution = bind(&MyFunctions::eval_solution, &myobject, _1, _2);
        ga_obj.mutate = bind(&MyFunctions::mutate, &myobject, _1, _2, _3);
        ga_obj.crossover = bind(&MyFunctions::crossover, &myobject, _1, _2, _3);
        ga_obj.MO_report_generation = bind(&MyFunctions::MO_report_generation, &myobject, _1, _2, _3);

        ga_obj.crossover_fraction = 0.9;
        ga_obj.mutation_rate = 0.2;
        ga_obj.solve();

        std::cout << "The problem is optimized in " << timer.toc() << " seconds." << std::endl;
        myobject.save_results(ga_obj, option_ned_file);

        spd::drop_all();
    } catch (const spd::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
