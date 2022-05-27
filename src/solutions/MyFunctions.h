//
// Created by faniche on 22-4-20.
//

#ifndef SCHEDPLUS_MYFUNCTIONS_H
#define SCHEDPLUS_MYFUNCTIONS_H


#include <string>
#include <iostream>
#include <fstream>
#include <functional>
#include <utility>
#include <iomanip>
#include "GA_Solution.h"
#include "../Utils.h"
#include "../../lib/openGA.hpp"
#include "../../lib/pugixml.hpp"
#include "../event/Event.h"

class MyFunctions {
private:
    std::vector<Node *> nodes;
    std::vector<Node *> esList;
    std::vector<Node *> swList;
    std::map<node_idx, Node *> nodeMap;
    std::vector<std::reference_wrapper<DirectedLink>> links;
    std::vector<std::reference_wrapper<Flow>> flows;
    std::string output_location;
    uint64_t hyperPeriod;
    std::vector<schedplus::Event> events;
    std::map<uint32_t, std::vector<uint32_t>> link_flows;
    std::map<uint32_t, uint64_t> link_hyperperiod;

    static bool compIntervals(const std::pair<uint64_t, uint64_t> &p1, const std::pair<uint64_t, uint64_t> &p2) {
        return p1.first < p2.first;
    }

    void setLinkHyperperiod(const std::vector<uint64_t> &selected_route_idx){
        if (!link_hyperperiod.empty()) {
            link_hyperperiod.clear();
        }
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                if (link_hyperperiod.contains(link_id)) {
                    link_hyperperiod[link_id] = Util::lcm(link_hyperperiod[link_id], flow.get().getPeriod());
                } else {
                    link_hyperperiod[link_id] = flow.get().getPeriod();
                }
            }
        }
    }


    void setLinkFLow(){
        link_flows.clear();
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                link_flows[link.get().getId()].emplace_back(flow.get().getId());
            }
        }
    }

    void setTimeIntervals(const std::vector<uint64_t> &offsets,
                                 const std::vector<uint64_t> &selected_route_idx,
                                 std::map<uint32_t, std::vector<std::pair<uint64_t, uint64_t>>> &transmit_intervals) {
        //        link_id     mem
        std::map<uint32_t, uint64_t> mem_allocate;
        std::map<uint32_t, uint64_t> _link_hyperperiod;
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                if (_link_hyperperiod.contains(link_id)) {
                    _link_hyperperiod[link_id] = Util::lcm(_link_hyperperiod[link_id], flow.get().getPeriod());
                } else {
                    _link_hyperperiod[link_id] = flow.get().getPeriod();
                }
            }
        }
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId() - 1]].getLinks()) {
                uint32_t link_id = link.get().getId();
                uint64_t sendTimes = _link_hyperperiod[link_id] / flow.get().getPeriod();
                for (int i = 0; i < sendTimes; ++i) {
                    if (mem_allocate.contains(link_id)) {
                        mem_allocate[link_id] += sendTimes;
                    } else {
                        mem_allocate[link_id] = sendTimes;
                    }
                }
            }
        }
        for (auto &[link_id, gcl_entities]: transmit_intervals) {
            gcl_entities.reserve(mem_allocate[link_id]);
        }

        for (auto const &flow: flows) {
            /* Calculate GCL */
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t offset = offsets[flow_id];
            uint64_t accumulatedDelay = offset;
            uint64_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow_id]].getLinks()) {
                uint32_t link_id = link.get().getId();
                proc_delay = link.get().getSrcNode()->getDpr();
                trans_delay = flow.get().getFrameLength() * link.get().getSrcPort().getMacrotick();
                accumulatedDelay += proc_delay;
                uint64_t sendTimes = _link_hyperperiod[link_id] / flow.get().getPeriod();
                for (uint32_t i = 0; i < sendTimes; ++i) {
                    uint64_t start_time = accumulatedDelay + i * flow.get().getPeriod();
                    transmit_intervals[link_id].emplace_back(std::pair(start_time, trans_delay));
                }
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
            }
        }
        for (auto &[link_idx, intervals]: transmit_intervals) {
            std::sort(intervals.begin(), intervals.end(), compIntervals);
        }
    }

public:

    MyFunctions(std::string output_location,
                std::vector<Node *> nodes,
                std::vector<Node *> esList,
                std::vector<Node *> swList,
                std::map<node_idx, Node *> nodeMap,
                std::vector<DirectedLink> &_links,
                std::vector<Flow> &_flows) :
            output_location(std::move(output_location)),
            nodes(std::move(nodes)),
            esList(std::move(esList)),
            swList(std::move(swList)),
            nodeMap(std::move(nodeMap)) {

        links.assign(_links.begin(), _links.end());
        flows.assign(_flows.begin(), _flows.end());
        std::vector<std::reference_wrapper<Flow>> wrapper_flows(flows.begin(), flows.end());
        std::ostringstream oss;
        hyperPeriod = Util::getHyperPeriod(wrapper_flows, oss);
    }

    ~MyFunctions() = default;

    void init_genes(TTFlows &p, const std::function<double(void)> &rnd01) {
        p.offsets.assign(flows.size(), 0);
        p.selected_route_idx.assign(flows.size(), 0);
        /* Initialize the offset and route index of flow. */
        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            /* Transmit delay of frame, unit: ns */
            //      delay     =  frame_length   *    port_macrotick
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            uint64_t offset = (flow.get().getPeriod() - srcTransDelay) * rnd01();
            p.offsets[flow_id] = offset;
            if (flow.get().getRoutes().size() == 1) {
                p.selected_route_idx[flow_id] = 0;
            } else {
                uint64_t route_idx = (flow.get().getRoutes().size()) * rnd01();
                p.selected_route_idx[flow_id] = route_idx;
            }

        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        /* Check delivery guarantees. */
        uint64_t max_ddl = 0, max_e2e = 0;
        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t e2e = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getE2E();
            /* Check delivery guarantees */
            if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                uint64_t ddl = p.offsets[flow_id] + e2e;
                if (ddl > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    return false;
                /* the max ddl in current solution */
                if (ddl > max_ddl)
                    max_ddl = ddl;
            } else if (flow.get().getPriorityCodePoint() == schedplus::P5) {
                if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    return false;
                /* the max e2e in current solution */
                if (e2e > max_e2e)
                    max_e2e = e2e;
            }
        }

        /* Check gcl collision */
        //       link_id                          start     interval
        std::map<uint32_t, std::vector<std::pair<uint64_t, uint64_t>>> transmit_intervals;
        setTimeIntervals(p.offsets, p.selected_route_idx, transmit_intervals);
        for (auto &[link_idx, intervals]: transmit_intervals)
            for (int i = 0; i < intervals.size() - 1; ++i)
                if (intervals[i].first + intervals[i].second + schedplus::IFG_TIME > intervals[i + 1].first)
                    return false;


        /* Get variance of current solution */
        std::vector<uint64_t> link_gcl_size(transmit_intervals.size());
        for (auto &[link_idx, intervals]: transmit_intervals)
            link_gcl_size.emplace_back(intervals.size());
        double mean = std::accumulate(link_gcl_size.begin(), link_gcl_size.end(), 0.0) / link_gcl_size.size();
        std::vector<double> diff(link_gcl_size.size());
        std::transform(link_gcl_size.begin(), link_gcl_size.end(), diff.begin(), [mean](double x) {
            return x - mean;
        });
        c.variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / link_gcl_size.size();

        /* set ddl and e2e for solution */
        c.ddl = max_ddl;
        c.e2e = max_e2e;

        spdlog::set_level(spdlog::level::info);
        spdlog::get("console")->debug("*******Correct solution*******");
        spdlog::get("console")->debug("\tlink\troute\toffset");
        std::for_each(flows.begin(), flows.end(),
                      [&](std::reference_wrapper<Flow> flow) {
                          spdlog::get("console")->debug("\t{}\t{}\t{}", flow.get().getId() - 1,
                                                        p.selected_route_idx[flow.get().getId() - 1],
                                                        p.offsets[flow.get().getId() - 1]);
                      });
        spdlog::get("console")->debug("******************************");
        return true;
    }

    TTFlows mutate(
            const TTFlows &X_base,
            const std::function<double(void)> &rnd01,
            double shrink_scale) {
        TTFlows X_new;
        const double mu = 0.2 * shrink_scale; // mutation radius (adjustable)
        X_new = X_base;
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId() - 1;
            uint64_t offset = 0, route = 0;
            bool in_range;
            do {
                offset = X_base.offsets[flow_id] + mu * (rnd01() - rnd01());
                int srcTransDelay =
                        flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
                in_range = (offset >= 0) && (offset < (flow.get().getPeriod() - srcTransDelay));
                if (flow.get().getRoutes().size() > 1) {
                    route = X_base.selected_route_idx[flow_id] + mu * (rnd01() - rnd01());
                    in_range = in_range && route >= 0 && route < flow.get().getRoutes().size();
                } else {
                    route = 0;
                }
            } while (!in_range);
            X_new.offsets[flow_id] = offset;
            X_new.selected_route_idx[flow_id] = route;
        }
        return X_new;
    }

    TTFlows crossover(
            const TTFlows &X1,
            const TTFlows &X2,
            const std::function<double(void)> &rnd01) {
        TTFlows X_new;
        X_new = X1;
        double r_offset = rnd01();
        double r_route = rnd01();
        for (int i = 0; i < X1.offsets.size(); ++i) {
            uint64_t offset = r_offset * X1.offsets[i] + (1.0 - r_offset) * X2.offsets[i];
            X_new.offsets[i] = offset;
            if (flows[i].get().getRoutes().size() > 1) {
                uint64_t route = r_route * X1.selected_route_idx[i] + (1.0 - r_route) * X2.selected_route_idx[i];
                X_new.selected_route_idx[i] = route;
            } else {
                X_new.selected_route_idx[i] = 0;
            }
        }
        return X_new;
    }

    std::vector<double> calculate_MO_objectives(const GA_Type::thisChromosomeType &X) {
        return {
                X.middle_costs.variance,
                X.middle_costs.e2e,
                X.middle_costs.ddl
        };
    }

    void MO_report_generation(
            int generation_number,
            const EA::GenerationType<TTFlows, MyMiddleCost> &last_generation,
            const std::vector<unsigned int> &pareto_front) {
        (void) last_generation;

        std::cout << "Generation [" << generation_number << "], ";
        std::cout << "Pareto-Front {";
        for (unsigned int i = 0; i < pareto_front.size(); i++) {
            std::cout << (i > 0 ? "," : "");
            std::cout << pareto_front[i];
        }
        std::cout << "}" << std::endl;
    }

    void save_results(GA_Type &ga_obj, const std::string &ned_file) {
        std::vector<unsigned int> paretofront_indices = ga_obj.last_generation.fronts[0];
        for (unsigned int i: paretofront_indices) {
            /* set flow offset and route index */
            auto &X = ga_obj.last_generation.chromosomes[i];
            for (auto &flow: flows) {
                uint32_t flow_id = flow.get().getId() - 1;
                flow.get().setOffset(X.genes.offsets[flow_id]);
                flow.get().setSelectedRouteInx(X.genes.selected_route_idx[flow_id]);
            }
            setLinkHyperperiod(X.genes.selected_route_idx);
            setLinkFLow();
            saveGCL(X.genes.offsets, X.genes.selected_route_idx);

            std::string route_file = output_location;
            route_file.append("/" + std::to_string(i) + "SmallRouting.xml");
            save_route(route_file);

            std::string gcl_file = output_location;
            gcl_file.append("/" + std::to_string(i));
            saveGCL(gcl_file, X.genes.selected_route_idx);

            std::string route_file_name = std::to_string(i) + "SmallRouting.xml";
            std::string gcl_file_name = std::to_string(i) + "SmallGCL.xml";
            std::string ini_file = output_location;
            ini_file.append("/" + std::to_string(i) + "SmallTopology.ini");
            savIni(route_file_name, gcl_file_name, ini_file, ned_file, i);

//            std::string event_file = output_location;
//            event_file.append("/" + std::to_string(i) + "event.txt");
//            saveEvent(X.genes.offsets, X.genes.selected_route_idx, event_file);
        }
    }

    void save_route(const std::string &route_file_location) {
        spdlog::get("console")->info("Saving route: {}", route_file_location);



        std::map<node_idx, std::vector<uint32_t >> sw_links;
        for (auto const &sw: swList) {
            for (auto const &[link_id, flowids]: link_flows) {
                if (links[link_id].get().getSrcNode() == sw) {
                    sw_links[Node::nodeToIdx(nodeMap, sw)].emplace_back(link_id);
                }
            }
        }
        pugi::xml_document xdoc;
        pugi::xml_node xdec = xdoc.prepend_child(pugi::node_declaration);
        xdec.append_attribute("version").set_value("1.0");
        xdec.append_attribute("encoding").set_value("utf-8");
        pugi::xml_node xdbs = xdoc.append_child("filteringDatabases");

        /* iterate all switchs */
        for (auto const &[node_id, links_id]: sw_links) {
            pugi::xml_node xdb = xdbs.append_child("filteringDatabase");
            pugi::xml_attribute xsw_id = xdb.append_attribute("id");
            xsw_id.set_value(nodes[node_id]->getName().c_str());
            pugi::xml_node xstatic = xdb.append_child("static");
            pugi::xml_node xforward = xstatic.append_child("forward");
            /* iterate a switch's ports */
            size_t ports = ((Switch *) (nodes[node_id]))->getPortNum();
            for (int i = 0; i < ports; ++i) {
                for (auto &link_id: links_id) {
                    if (uuid_compare(links[link_id].get().getSrcPort().getId(),
                                     ((Switch *) (nodes[node_id]))->getPorts()[i].getId()) == 0) {
                        for (auto const &flow_id: link_flows[link_id]) {
                            pugi::xml_node xmulticast_addr = xforward.append_child("multicastAddress");
                            pugi::xml_attribute xmac = xmulticast_addr.append_attribute("macAddress");
                            std::string mac = "255-0-00-00-00-" + std::to_string(flow_id);
                            if (flow_id < 10) {
                                mac.insert(mac.length() - 1, "0");
                            }
                            xmac.set_value(mac.c_str());
                            pugi::xml_attribute xports = xmulticast_addr.append_attribute("ports");
                            xports.set_value(i);
                        }
                    }
                }
            }
        }
//        xdoc.print(std::cout);
        xdoc.save_file(route_file_location.c_str());
    }

    void saveGCL(const std::string &gcl_file_location, const std::vector<uint64_t> &selected_route_idx) {
        std::string switch_gcl_file = gcl_file_location + "SmallGCL.xml";
        saveSwPortSchedule(switch_gcl_file, selected_route_idx);
        saveEsSchedule(gcl_file_location, selected_route_idx);
    }

    void saveSwPortSchedule(const std::string &sched_file_location, const std::vector<uint64_t> &selected_route_idx) {
        spdlog::get("console")->info("Saving gcl: {}", sched_file_location);
        pugi::xml_document xdoc;
        pugi::xml_node xdec = xdoc.prepend_child(pugi::node_declaration);
        xdec.append_attribute("version").set_value("1.0");
        xdec.append_attribute("encoding").set_value("utf-8");
        pugi::xml_node xschedules = xdoc.append_child("schedules");
        pugi::xml_node xhyperperiod = xschedules.append_child("defaultcycle");
        std::string hyp_str = std::to_string(hyperPeriod) + "ns";
        xhyperperiod.append_child(pugi::node_pcdata).set_value(hyp_str.c_str());

        for (auto &sw: swList) {
            pugi::xml_node xswitch = xschedules.append_child("switch");
            pugi::xml_attribute xname = xswitch.append_attribute("name");
            xname.set_value(sw->getName().c_str());
            size_t ports = ((Switch *) sw)->getPortNum();
            for (int i = 0; i < ports; ++i) {
                for (auto const &link: links) {
                    if (link.get().getSrcNode() == sw) {
                        if (uuid_compare(link.get().getSrcPort().getId(),
                                         ((Switch *) sw)->getPorts()[i].getId()) == 0 &&
                            !link.get().getSrcPort().getGateControlList().empty()) {
                            uint32_t link_id = link.get().getId();
                            pugi::xml_node xport = xswitch.append_child("port");
                            pugi::xml_attribute xport_id = xport.append_attribute("id");
                            xport_id.set_value(i);
                            pugi::xml_node xschedule = xport.append_child("schedule");
                            pugi::xml_attribute xcycle_time = xschedule.append_attribute("cycleTime");
                            std::string cyc_time_str = std::to_string(link_hyperperiod[link_id]) + "ns";
                            xcycle_time.set_value(cyc_time_str.c_str());
                            uint32_t cur = 0;
                            for (auto const &gcl_entity: links[link_id].get().getSrcPort().getGateControlList()) {
                                uint32_t gap = gcl_entity.getStartTime() - cur;
                                GateControlEntry gateControlEntry;
                                pugi::xml_node xall_open_entry = xschedule.append_child("entry");
                                std::string wnd_len_str = std::to_string(gap) + "ns";
                                pugi::xml_node xall_open_len = xall_open_entry.append_child("length");
                                xall_open_len.append_child(pugi::node_pcdata).set_value(wnd_len_str.c_str());
                                pugi::xml_node xapp_open_bitvec = xall_open_entry.append_child("bitvector");
                                xapp_open_bitvec.append_child(pugi::node_pcdata).set_value(
                                        gateControlEntry.toBitVec().c_str());

                                pugi::xml_node xentry = xschedule.append_child("entry");
                                pugi::xml_node xlen = xentry.append_child("length");
                                wnd_len_str = std::to_string(gcl_entity.getTimeIntervalValue()) + "ns";
                                xlen.append_child(pugi::node_pcdata).set_value(wnd_len_str.c_str());
                                pugi::xml_node xbitvec = xentry.append_child("bitvector");
                                xbitvec.append_child(pugi::node_pcdata).set_value(gcl_entity.toBitVec().c_str());
                                cur = gcl_entity.getStartTime() + gcl_entity.getTimeIntervalValue();
                            }
                            if (cur < link_hyperperiod[link_id]) {
                                uint64_t gap = link_hyperperiod[link_id] - cur;
                                GateControlEntry gateControlEntry;
                                pugi::xml_node xentry = xschedule.append_child("entry");
                                pugi::xml_node xlen = xentry.append_child("length");
                                std::string wnd_len_str = std::to_string(gap) + "ns";
                                xlen.append_child(pugi::node_pcdata).set_value(wnd_len_str.c_str());
                                pugi::xml_node xbitvec = xentry.append_child("bitvector");
                                xbitvec.append_child(pugi::node_pcdata).set_value(gateControlEntry.toBitVec().c_str());
                            }
                        }
                    }
                }
            }
        }
        xdoc.save_file(sched_file_location.c_str());
    }

    void saveEsSchedule(const std::string &sched_file_location, const std::vector<uint64_t> &selected_route_idx) {
        std::map<node_idx, std::vector<std::reference_wrapper<Flow>>> flowGroup;
        /* Group the flow with src */
        for (auto &flow: flows) {
            node_idx key = Node::nodeToIdx(nodeMap, flow.get().getSrc());
            flowGroup[key].emplace_back(flow);
        }
        /* Sort the flow with PCP */
        spdlog::set_level(spdlog::level::info);
        for (auto &[src, _flows]: flowGroup) {
            std::sort(_flows.begin(), _flows.end(), Util::compareFlowWithOffset);
        }
        for (auto &[src, _flows]: flowGroup) {
            auto const &es = nodeMap[src];
            pugi::xml_document xesdoc;
            pugi::xml_node xesdec = xesdoc.prepend_child(pugi::node_declaration);
            xesdec.append_attribute("version").set_value("1.0");
            xesdec.append_attribute("encoding").set_value("utf-8");
            pugi::xml_node xes_schedules = xesdoc.append_child("schedules");
            pugi::xml_node xes_cycle = xes_schedules.append_child("defaultcycle");
            std::string cyc_str;
            for (auto const &link: links) {
                if (link.get().getSrcNode() == es) {
                    cyc_str = std::to_string(link_hyperperiod[link.get().getId()]) + "ns";
                    break;
                }
            }
            xes_cycle.append_child(pugi::node_pcdata).set_value(cyc_str.c_str());
            pugi::xml_node xhost = xes_schedules.append_child("host");
            pugi::xml_attribute xname = xhost.append_attribute("name");
            xname.set_value(es->getName().c_str());
            pugi::xml_node xcycle = xhost.append_child("cycle");
            xcycle.append_child(pugi::node_pcdata).set_value(cyc_str.c_str());
            for (auto const &flow: _flows) {
                pugi::xml_node xentry = xhost.append_child("entry");
                pugi::xml_node xstart = xentry.append_child("start");
                std::string start_str = std::to_string(flow.get().getOffset()) + "ns";
                xstart.append_child(pugi::node_pcdata).set_value(start_str.c_str());
                pugi::xml_node xqueue = xentry.append_child("queue");
                xqueue.append_child(pugi::node_pcdata).set_value(
                        std::to_string(flow.get().getPriorityCodePoint()).c_str());
                pugi::xml_node xdest = xentry.append_child("dest");
                std::string dest_str = "255:0:00:00:00:" + std::to_string(flow.get().getId());
                if (flow.get().getId() < 10) {
                    dest_str.insert(dest_str.length() - 1, "0");
                }
                xdest.append_child(pugi::node_pcdata).set_value(dest_str.c_str());
                pugi::xml_node xsize = xentry.append_child("size");
                std::string size_str = std::to_string(flow.get().getFrameLength() - schedplus::HEADER_LEN) + "B";
                xsize.append_child(pugi::node_pcdata).set_value(size_str.c_str());
                pugi::xml_node xflowId = xentry.append_child("flowId");
                xflowId.append_child(pugi::node_pcdata).set_value(std::to_string(flow.get().getId()).c_str());
                pugi::xml_node xflowPeriod = xentry.append_child("period");
                std::string period_str = std::to_string(flow.get().getPeriod()) + "ns";
                xflowPeriod.append_child(pugi::node_pcdata).set_value(period_str.c_str());
            }
            std::string es_traffic_gen = sched_file_location + es->getName() + ".xml";
            xesdoc.save_file(es_traffic_gen.c_str());
        }
    }

    void saveGCL(const std::vector<uint64_t> &offsets,
                 const std::vector<uint64_t> &selected_route_idx) {
        for (auto &flow: flows) {
            /* Calculate GCL */
            uint32_t flow_id = flow.get().getId() - 1;
            uint32_t offset = offsets[flow_id];
            uint32_t accumulatedDelay = offset;
            uint32_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
            flow.get().setSelectedRouteInx(selected_route_idx[flow_id]);
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow_id]].getLinks()) {
                link.get().clearGateControlEntry();
                uint32_t link_id = link.get().getId();
                proc_delay = link.get().getSrcNode()->getDpr();
                trans_delay = flow.get().getFrameLength() * link.get().getSrcPort().getMacrotick();
                accumulatedDelay += proc_delay;
                uint32_t sendTimes = link_hyperperiod[link_id] / flow.get().getPeriod();
                for (uint32_t i = 0; i < sendTimes; ++i) {
                    GateControlEntry gateControlEntry;
                    uint32_t start_time = accumulatedDelay + i * flow.get().getPeriod();
                    gateControlEntry.setStartTime(start_time);
                    gateControlEntry.setTimeIntervalValue(trans_delay);
                    for (int j = 0; j < 8; ++j) {
                        if (flow.get().getPriorityCodePoint() == j) {
                            gateControlEntry.setGateStatesValue(j, GATE_OPEN);
                        } else {
                            gateControlEntry.setGateStatesValue(j, GATE_CLOSE);
                        }
                    }
                    link.get().addGateControlEntry(gateControlEntry);
                }
                link.get().sortGCL();
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
            }
        }
    }

    void saveEvent(const std::vector<uint64_t> &offsets,
                   const std::vector<uint64_t> &selected_route_idx,
                   const std::string &event_file_location) {
        events.clear()  ;
        std::ostringstream oss;
        spdlog::get("console")->set_level(spdlog::level::info);
        for (auto const &es: esList) {
            for (auto const &flow: flows) {
                if (flow.get().getSrc() == es) {
                    auto &route = flow.get().getRoutes()[flow.get().getSelectedRouteInx()];
                    uint64_t repeat_times = hyperPeriod / link_hyperperiod[route.getLinks()[0].get().getId()];
                    for (int i = 0; i < repeat_times; ++i) {
                        uint64_t accumulatedDelay;
                        accumulatedDelay = flow.get().getOffset() + i * link_hyperperiod[route.getLinks()[0].get().getId()];
                        uint64_t prop_delay = 0, trans_delay = 0, proc_delay = 0;
                        int hop = route.getLinks().size();
                        for (int j = 0; j < hop; ++j) {
                            proc_delay = route.getLinks()[j].get().getSrcNode()->getDpr();
                            prop_delay = route.getLinks()[j].get().getLen() * route.getLinks()[j].get().getPropSpeed();
                            trans_delay = flow.get().getFrameLength() * route.getLinks()[j].get().getSrcPort().getMacrotick();
                            accumulatedDelay += proc_delay;
                            uint32_t sendTimes = link_hyperperiod[route.getLinks()[j].get().getId()] / flow.get().getPeriod();
                            for (int k = 0; k < sendTimes; ++k) {
                                uint64_t start_time = accumulatedDelay + k * flow.get().getPeriod();
                                uint64_t end_time = start_time + trans_delay;
                                schedplus::Event event(start_time, end_time, route.getLinks()[j].get().getSrcNode(),
                                                       &flow.get(), j);
                                if (j == 0) {
                                    event.setType(schedplus::TRANSMIT);
                                } else {
                                    event.setType(schedplus::FORWARD);
                                }
                                spdlog::get("console")->debug(event.toString(oss));
                                events.push_back(event);
                                if(j == hop - 1){
                                    event.setStart(end_time + prop_delay);
                                    event.setEnd(end_time + prop_delay + trans_delay);
                                    event.setType(schedplus::RECEIVE);
                                    event.setNode(flow.get().getDest());
                                    event.setHop(hop);
                                    events.push_back(event);
                                    spdlog::get("console")->debug(event.toString(oss));
                                }
                            }
                            accumulatedDelay += trans_delay;
                            accumulatedDelay += prop_delay;
                        }
                    }
                }
            }
        }
        schedplus::Event::sortEvent(events);
        std::ofstream output;
        output.open(event_file_location);
        output << std::right << std::setw(6) << "event"
               << std::left << std::setw(10) << "   start"
               << std::left << std::setw(10) << "    end"
               << std::left << std::setw(10) << "    type"
               << std::left << std::setw(9) << "     node"
               << std::right << std::setw(6) << " flow"
               << std::right << std::setw(5) << "hop" << std::endl;
        for (int i = 0; i < events.size(); ++i) {
            auto &event = events[i];
            output << std::right << std::setw(5) << i;
            output << std::right << std::setw(10) << std::to_string(event.getStart());
            output << std::right << std::setw(10) << std::to_string(event.getEnd());
            output << std::right << std::setw(10) << event.getType();
            output << std::right << std::setw(10) << event.getNode()->getName();
            output << std::right << std::setw(5) << std::to_string(event.getFlow()->getId());
            output << std::right << std::setw(5) << std::to_string(event.getHop()) << std::endl;
        }
        output.close();
    }

    void savIni(const std::string &route_file,
                const std::string &gcl_file,
                const std::string &ini_file,
                const std::string &ned_file,
                int solution_id) {
        std::ofstream output(ini_file);
        output << "[General]\n"
                  "network = " << ned_file << "\n"
                  "\n"
                  "record-eventlog = false \n"
                  "debug-on-errors = true\n"
                  "result-dir = result/" <<  ned_file << "\n"
                  "sim-time-limit =" << std::to_string(hyperPeriod / 1000000) <<  "ms\n"
                  "\n"
                  "# debug\n"
                  "**.displayAddresses = false\n"
                  "**.verbose = false" << std::endl;
        output << "# MAC Addresses" << std::endl;
        for (auto const &es: esList) {
            node_idx id = Node::nodeToIdx(nodeMap, es) + 1;
            output << "**." << es->getName()
                   << R"(.eth.address = "00-00-00-00-00-)" << std::setw(2) << std::setfill('0') << std::hex << id
                   << "\"" << std::endl;
        }
        output << R"(**.frequency = 1GHz)" << std::endl;
        output << "# Switches" << std::endl;

        output << R"(**.switch*.processingDelay.delay = 30000ns)" << std::endl;

        output << R"(**.filteringDatabase.database = xmldoc("xml/)" << route_file << R"(", "/filteringDatabases/"))"
               << std::endl;
        for (auto const &sw: swList) {
            size_t ports = ((Switch *) sw)->getPortNum();
            for (int i = 0; i < ports; ++i) {
                if (isPortInUse(((Switch *) sw)->getPorts()[i])) {
                    output << "**." << sw->getName() << ".eth[" << std::to_string(i)
                           << R"(].queue.gateController.initialSchedule = xmldoc("xml/)" << gcl_file;
                    output << R"(", "/schedules/switch[@name=')" << sw->getName() << R"(']/port[@id=')"
                           << std::to_string(i) << R"(']/schedule"))" << std::endl;
                }
            }
        }
        output << R"(**.gateController.enableHoldAndRelease = true)" << std::endl;
        for (int i = 0; i < 8; ++i) {
            output << R"(**.switch*.eth[*].queuing.tsAlgorithms[)" << std::to_string(i)
                   << R"(].typename = "StrictPriority")" << std::endl;
        }
//        output << R"(**.queues[*].bufferCapacity = 363360b)" << std::endl;
//        output << R"(**.sw*.eth[*].mac.enablePreemptingFrames = false)" << std::endl;

        for (auto &es: esList) {
            if (isPortInUse(((EndSystem *) es)->getPort())) {
                output << "**." << es->getName() << R"(.trafGenSchedApp.initialSchedule = xmldoc("xml/)"
                       << std::to_string(solution_id) << es->getName() << R"(.xml"))" << std::endl;
            }
        }
    }

    bool isPortInUse(const Port &port) {
        for (auto const &flow: flows) {
            for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                if (uuid_compare(link.get().getSrcPort().getId(), port.getId()) == 0)
                    return true;
            }
        }
        return false;
    }


};


#endif //SCHEDPLUS_MYFUNCTIONS_H
