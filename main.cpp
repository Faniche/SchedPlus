#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_TRACE_ON
#define SPDLOG_DEBUG_ON

#include "lib/spdlog/include/spdlog/spdlog.h"
#include "lib/spdlog/include/spdlog/sinks/stdout_color_sinks.h"
#include "src/solutions/GA.h"
#include "src/solutions/GA_4sw_ring.h"
#include "src/solutions/GA_line_2_1.h"
#include "src/solutions/GA_line_2_2.h"
#include "src/solutions/GA_tree.h"
#include "lib/CLI11/include/CLI/CLI.hpp"
#include "src/solutions/NoWait.h"
#include "src/solutions/Wait.h"

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
    int option_generation_number = 100;
    app.add_option("-g, --generation", option_generation_number, "The number of generation");
    bool flag_debug = {false};
    app.add_flag("-d, --debug", flag_debug, "debug mode");
    bool flag_random = {false};
    app.add_flag("-r, --random", flag_random, "use random flow for test");
    try {
        CLI11_PARSE(app, argc, argv);
    } catch (CLI::ParseError error) {
        app.exit(error);
    }
    try {
//        auto console = spd::stdout_color_mt("console");
        auto sink = std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>();
        auto console = std::make_shared<spdlog::logger>("console", sink);
        console->set_level(spdlog::level::info);
        spd::set_default_logger(console);
        spd::set_pattern("[%H:%M:%S] [%^%l%$] %s:%# %v");


        // Compile time log levels
        // define SPDLOG_DEBUG_ON or SPDLOG_TRACE_ON
        SPDLOG_LOGGER_TRACE(console, "Enabled only #ifdef SPDLOG_TRACE_ON..{} ,{}", 1, 3.23);
        SPDLOG_LOGGER_DEBUG(console, "Enabled only #ifdef SPDLOG_DEBUG_ON.. {} ,{}", 1, 3.23);

        std::vector<Node *> nodes;
        std::vector<Node *> esList;
        std::vector<Node *> swList;
        std::map<node_idx , Node *> nodeMap;
        std::vector<DirectedLink> links;
        std::vector<Flow> flows;

        spdlog::get("console")->info("Topology index: {}", option_topology);
        if (flag_random) {
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
                case 4:
                    GA_tree::openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
                default:
                    openGACal(option_flow_number, nodes, esList, swList, nodeMap, links, flows);
                    break;
            }
            Util::saveFlows(flows);
        } else {
            switch (option_topology) {
                case 1:
                    GA_line_2_1::openGACal(nodes, esList, swList, nodeMap, links, flows);
                    break;
                case 2:
                    GA_line_2_2::openGACal(nodes, esList, swList, nodeMap, links, flows);
                    break;
                case 3:
                    Small4SwRing::openGACal(nodes, esList, swList, nodeMap, links, flows);
                    break;
                case 4:
                    GA_tree::openGACal(nodes, esList, swList, nodeMap, links, flows);
                    break;
                default:
                    openGACal(nodes, esList, swList, nodeMap, links, flows);
                    break;
            }
        }
        map<schedplus::PRIORITY_CODE_POINT, vector<uint32_t>> flowGroupPcp;
        for (int i = 0; i < flows.size(); ++i) {
            flowGroupPcp[flows[i].getPriorityCodePoint()].emplace_back(i);
        }
        map<uint64_t, vector<uint32_t>> same_period_flow;
        for (auto const &flow: flows) {
            if (flow.getPriorityCodePoint() == schedplus::P6)
                same_period_flow[flow.getPeriod()].push_back(flow.getId());
        }
        vector<uint64_t> del_keys;
        for (const auto& item: same_period_flow) {
            if (item.second.size() < 2) {
                del_keys.push_back(item.first);
                continue;
            }
            for (int i = 1; i < item.second.size(); ++i) {
                if (flows[item.second[i]].getFrameLength() != flows[item.second[0]].getFrameLength())
                    del_keys.push_back(item.first);
            }
        }
        for (const auto& item: del_keys) {
            same_period_flow.erase(item);
        }

//        NoWait myobject(nodes, esList, swList, nodeMap, links, flows, flowGroupPcp);
        Wait myobject(nodes, esList, swList, nodeMap, links, flows, flowGroupPcp, same_period_flow);
        std::string delete_file = "rm /home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small/wait/*";
        system(delete_file.c_str());
        EA::Chronometer timer;
        timer.tic();

        GA_Type ga_obj;
        ga_obj.problem_mode = EA::GA_MODE::NSGA_III;
//        ga_obj.problem_mode = EA::GA_MODE::SOGA;
        ga_obj.multi_threading = !flag_debug;
        ga_obj.dynamic_threading = !flag_debug;
        ga_obj.verbose = flag_debug;
        ga_obj.population = 100;
        ga_obj.generation_max = option_generation_number;

        using std::bind;
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;

//        ga_obj.calculate_MO_objectives = bind(&NoWait::calculate_MO_objectives, &myobject, _1);
//        ga_obj.init_genes =  bind(&NoWait::init_genes, &myobject, _1, _2);
//        ga_obj.eval_solution = bind(&NoWait::eval_solution, &myobject, _1, _2);
//        ga_obj.mutate = bind(&NoWait::mutate, &myobject, _1, _2, _3);
//        ga_obj.crossover = bind(&NoWait::crossover, &myobject, _1, _2, _3);
//        ga_obj.MO_report_generation = bind(&NoWait::MO_report_generation, &myobject, _1, _2, _3);

//        ga_obj.calculate_SO_total_fitness = bind(&NoWait::calculate_SO_total_fitness, &myobject, _1);
//        ga_obj.SO_report_generation = bind(&NoWait::SO_report_generation, &myobject, _1, _2, _3);

        ga_obj.calculate_MO_objectives = bind(&Wait::calculate_MO_objectives, &myobject, _1);
        ga_obj.init_genes =  bind(&Wait::init_genes, &myobject, _1, _2);
        ga_obj.eval_solution = bind(&Wait::eval_solution, &myobject, _1, _2);
        ga_obj.mutate = bind(&Wait::mutate, &myobject, _1, _2, _3);
        ga_obj.crossover = bind(&Wait::crossover, &myobject, _1, _2, _3);
        ga_obj.MO_report_generation = bind(&Wait::MO_report_generation, &myobject, _1, _2, _3);

//        ga_obj.get_shrink_scale = bind(&Wait::get_shrink_scale, &myobject, _1, _2);
        ga_obj.crossover_fraction = 0.7;
        ga_obj.mutation_rate = 0.4;

        ga_obj.solve();

        std::cout << "The problem is optimized in " << timer.toc() << " seconds." << std::endl;
        myobject.save_results(ga_obj, option_ned_file);

    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
        spdlog::shutdown();
        return 1;
    }
    spdlog::shutdown();
    return 0;
}
