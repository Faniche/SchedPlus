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
    vector<Node *> nodes;
    vector<Node *> esList;
    vector<Node *> swList;
    map<node_idx, Node *> nodeMap;
    vector<std::reference_wrapper<DirectedLink>> links;
    vector<std::reference_wrapper<Flow>> flows;
    std::string output_location;
    uint64_t hyperPeriod;
    vector<schedplus::Event> events;
    map<schedplus::PRIORITY_CODE_POINT, vector<uint32_t>> flowGroupPcp;

    static bool compIntervals(const std::pair<uint64_t, uint64_t> &p1, const std::pair<uint64_t, uint64_t> &p2) {
        return p1.first < p2.first;
    }

    void setLinkHyperperiod(const vector<uint64_t> &selected_route_idx, map<uint32_t, uint64_t> &link_hyperperiod){
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

    void setEachHopStartTimeP6(uint32_t flow_id,
                               uint64_t offset,
                               uint64_t route,
                               uint64_t snd_times,
                               map<uint32_t, map<uint64_t, map<uint8_t, uint64_t>>> &p6_traffic_offsets) {
        uint64_t accDelay = offset;
        auto const &flow = flows[flow_id].get();
        for (int i = 0; i < flow.getRoutes()[route].getLinks().size(); ++i) {
            auto const &link = flow.getRoutes()[route].getLinks()[i];
            accDelay += link.get().getSrcNode()->getDpr();
            p6_traffic_offsets[flow_id][snd_times][i] = accDelay;
            accDelay += flow.getFrameLength() * link.get().getSrcPort().getMacrotick();
            accDelay += link.get().getPropSpeed() * link.get().getLen();
        }
    }

    bool checkCollision (const map<uint32_t, vector<std::pair<uint32_t, uint8_t>>> &link_flows,
                         const schedplus::PRIORITY_CODE_POINT pcp,
                         std::ostringstream &oss,
                         map<uint32_t, map<uint64_t, map<uint8_t, uint64_t>>> &traffic_offsets) {
        spdlog::get("console")->set_level(spdlog::level::debug);
        for (auto const &[link_id, flows_id_hop]: link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                if (flow_i.getPriorityCodePoint() != pcp) continue;
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                uint64_t fi_mid = traffic_offsets[fi_id][0][hop_i] + fi_len / 2;
                for (int j = 0; j < i; ++j) {
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    if (flow_j.getPriorityCodePoint() != pcp) continue;
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    uint64_t fj_mid = traffic_offsets[fj_id][0][hop_j] + fj_len / 2;

                    uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
                    uint64_t hyp_cycle = Util::lcm(fi_period, fj_period);

                    if (fi_period < fj_period) {
                        for (int k = 0; k < hyp_cycle / fj_period; ++k) {
                            int d = (fj_mid + k * (fj_period - fi_period)) % fi_period - fi_mid;
                            if (std::abs(d) < dist) {
                                if (oss.str().empty()) oss << std::endl << "====start collision check===="<< std::endl;
                                oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                                    << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                                oss << "fi_period: " << std::left << std::setw(10) << flow_i.getPeriod()
                                    << "fj_period: " << std::left << std::setw(10) << flow_j.getPeriod() << std::endl;
                                oss << "fi_hop:    " << std::left << std::setw(10) << i
                                    << "fj_hop:    " << std::left << std::setw(10) << j << std::endl;
                                oss << "d:         " << std::left << std::setw(10) << d
                                    << "dist:      " << std::left << std::setw(10) << dist << std::endl;
                                oss << "=============end============="<< std::endl;
                                spdlog::get("console")->debug(oss.str());
                                oss.str("");
                                return false;
                            }

                        }
                    } else {
                        for (int k = 0; k < hyp_cycle / fi_period; ++k) {
                            int d = (fi_mid + k * (fi_period - fj_period)) % fj_period - fj_mid;
                            if (std::abs(d) < dist) {
                                if (oss.str().empty()) oss << std::endl << "====start=collision check===="<< std::endl;
                                oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                                    << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                                oss << "fi_period: " << std::left << std::setw(10) << flow_i.getPeriod()
                                    << "fj_period: " << std::left << std::setw(10) << flow_j.getPeriod() << std::endl;
                                oss << "fi_hop:    " << std::left << std::setw(10) << i
                                    << "fj_hop:    " << std::left << std::setw(10) << j << std::endl;
                                oss << "d:         " << std::left << std::setw(10) << d
                                    << "dist:      " << std::left << std::setw(10) << dist << std::endl;
                                oss << "=============end============="<< std::endl;
                                spdlog::get("console")->debug(oss.str());
                                oss.str("");
                                return false;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    bool checkP5E2E (const vector<uint64_t> &selected_route_idx, uint64_t &max_e2e) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P5]) {
            auto const &flow = flows[flow_id];
            uint64_t e2e = flow.get().getRoutes()[selected_route_idx[flow_id]].getE2E();
            if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                return false;
            /* the max e2e in current solution */
            if (e2e > max_e2e)
                max_e2e = e2e;
        }
        return true;
    }

    bool checkP6DDL (const vector<uint64_t> &selected_route_idx,
                     const vector<uint64_t> &offsets,
                     uint64_t &max_ddl) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P6]) {
            auto const &flow = flows[flow_id].get();
            uint64_t e2e = flow.getRoutes()[selected_route_idx[flow_id]].getE2E();
            uint64_t ddl = offsets[flow_id] + e2e;
            /* Check delivery guarantees */
            if (ddl > flow.getDeliveryGuarantees()[0].getLowerVal())
                return false;
            /* the max ddl in current solution */
            if (ddl > max_ddl)
                max_ddl = ddl;
        }
        return true;
    }
public:
    MyFunctions(std::string output_location,
                vector<Node *> nodes,
                vector<Node *> esList,
                vector<Node *> swList,
                map<node_idx, Node *> nodeMap,
                vector<DirectedLink> &_links,
                vector<Flow> &_flows,
                map<schedplus::PRIORITY_CODE_POINT, vector<uint32_t>> _flowGroupPcp) :
            output_location(std::move(output_location)),
            nodes(std::move(nodes)),
            esList(std::move(esList)),
            swList(std::move(swList)),
            nodeMap(std::move(nodeMap)),
            flowGroupPcp(std::move(_flowGroupPcp)){

        links.assign(_links.begin(), _links.end());
        flows.assign(_flows.begin(), _flows.end());
        vector<std::reference_wrapper<Flow>> wrapper_flows(flows.begin(), flows.end());
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
            uint64_t route_idx;
            if (flow.get().getRoutes().size() == 1)
                route_idx = 0;
            else
                route_idx = (flow.get().getRoutes().size()) * rnd01();
            p.selected_route_idx[flow_id] = route_idx;


            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            uint64_t up_bound = flow.get().getPeriod() - srcTransDelay;
            uint64_t offset = up_bound * rnd01();
//            uint64_t e2e = flow.get().getRoutes()[route_idx].getE2E();
//            if (flow.get().getPriorityCodePoint() == schedplus::P6) {
//                if (flow.get().getDeliveryGuarantees()[0].getLowerVal() <= e2e)
//                    offset = UINT64_MAX;
//                else {
//                    uint64_t up_bound = std::min((flow.get().getDeliveryGuarantees()[0].getLowerVal() - e2e), (flow.get().getPeriod() - srcTransDelay));
//                    offset = up_bound * rnd01();
//                }
//            } else if (flow.get().getPriorityCodePoint() == schedplus::P5) {
//                if (flow.get().getDeliveryGuarantees()[0].getLowerVal() <= e2e)
//                    offset = UINT64_MAX;
//                else {
//                    uint64_t up_bound = std::min((flow.get().getDeliveryGuarantees()[0].getLowerVal() - e2e), (flow.get().getPeriod() - srcTransDelay));
//                    offset = up_bound * rnd01();
//                }
//
//
//                /* Transmit delay of frame, unit: ns */
////                      delay     =  frame_length   *    port_macrotick
//                offset = (flow.get().getPeriod() - srcTransDelay) * rnd01();
//            }
            p.offsets[flow_id] = offset;
        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        /* Check Isochronous traffic ddl guarantee. */
        uint64_t max_ddl = 0, max_e2e = 0;

        if (!checkP6DDL(p.selected_route_idx, p.offsets, max_ddl))  return false;

        /* Set send offset of each hop */
        for (auto &flow_id: flowGroupPcp[schedplus::P6]) {
            setEachHopStartTimeP6(flow_id, p.offsets[flow_id], p.selected_route_idx[flow_id], 0, c.p6_traffic_offsets);
        }

        map<uint32_t, vector<std::pair<uint32_t, uint8_t>>> link_flows;
        for (auto const &flow_id: flowGroupPcp[schedplus::P6]) {
            auto const &flow = flows[flow_id].get();
            for (int i = 0; i < flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id =  flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }

        std::ostringstream oss("");

        if (!checkCollision(link_flows, schedplus::P6, oss, c.p6_traffic_offsets))
            return false;

        if (!checkP5E2E(p.selected_route_idx, max_e2e))
            return false;

        for (auto const &flow_id: flowGroupPcp[schedplus::P5]) {
            auto const &flow = flows[flow_id].get();
            for (int i = 0; i < flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id =  flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }
        setLinkHyperperiod(p.selected_route_idx, c.link_hyperperiod);
        spdlog::get("console")->set_level(spdlog::level::debug);
        for (auto &flow_id: flowGroupPcp[schedplus::P5]) {
            auto const &flow_i = flows[flow_id].get();
            auto const &route = flow_i.getRoutes()[p.selected_route_idx[flow_id]];
            uint64_t hyp = 0;
            for (auto const &link: route.getLinks()) {
                if (hyp == 0) hyp = c.link_hyperperiod[link.get().getId()];
                else hyp = Util::lcm(hyp, c.link_hyperperiod[link.get().getId()]);
            }
            uint64_t fi_send_times = hyp / flow_i.getPeriod();
            for (size_t i = 0; i < fi_send_times; ++i) {
                uint64_t offset = p.offsets[flow_id] + i * flow_i.getPeriod();
                c.p5_traffic_offsets[flow_id][i][0] = offset;
                oss << "flow: " << flow_id << ", cur: " << i << ", offset: " << offset << ", map: " << c.p5_traffic_offsets[flow_id][i][0] << std::endl;
            }
            /* repeat each hop of flow i */
            for (size_t i = 0; i < route.getLinks().size(); ++i) {
                auto const &link = route.getLinks()[i].get();
                uint64_t fi_len = flow_i.getFrameLength() * link.getSrcPort().getMacrotick();
                uint64_t dist = fi_len / 2;
                for (size_t j = 0; j < fi_send_times; ++j) {
                    c.p5_traffic_offsets[flow_id][j][i] = c.p5_traffic_offsets[flow_id][j][i] + link.getSrcNode()->getDpr();
                    /* End to end latency check */
                    c.p5_e2e[flow_id][j] = c.p5_traffic_offsets[flow_id][j][i] - (p.offsets[flow_id] + j * flow_i.getPeriod());
                    if (c.p5_e2e[flow_id][j] > flow_i.getDeliveryGuarantees()[0].getLowerVal()) {
                        spdlog::get("console")->debug("c.p5_traffic_offsets[{}][{}][{}] = {}", flow_id, j, i, c.p5_traffic_offsets[flow_id][j][i]);
                        spdlog::get("console")->debug("c.p5_e2e[{}][{}] = {}", flow_id, j, c.p5_e2e[flow_id][j]);
                        spdlog::get("console")->debug("delivery e2e: {}", flow_i.getDeliveryGuarantees()[0].getLowerVal());
                        spdlog::get("console")->debug("{} - {} = {}",
                                                      c.p5_traffic_offsets[flow_id][j][i],
                                                      p.offsets[flow_id] + j * flow_i.getPeriod(),
                                                      c.p5_e2e[flow_id][j]);
                        spdlog::get("console")->debug(oss.str());
                        oss.str("");
                        return false;
                    }
                    oss.str("");

                    if (c.p5_e2e[flow_id][j] > max_e2e)
                        max_e2e = c.p5_e2e[flow_id][j];
                    bool collison_checked = false;
                    if (collison_checked == false) {
                        for (auto const &_flow_hop: link_flows[link.getId()]) {
                            auto const &flow_j = flows[_flow_hop.first].get();
                            if (flow_j.getPriorityCodePoint() == schedplus::P5) continue;

                            uint64_t fj_start = c.p6_traffic_offsets[_flow_hop.first][0][_flow_hop.second];
                            uint64_t fj_len = flow_j.getFrameLength() * link.getSrcPort().getMacrotick();
                            uint64_t hij = Util::lcm(flow_i.getPeriod(), flow_j.getPeriod());

                            uint64_t fi_snd_times_hij = hij / flow_i.getPeriod();
                            uint64_t fi_idx_in_hij = j % fi_snd_times_hij;
                            uint64_t fi_mid = (c.p5_traffic_offsets[flow_id][j][i] + fi_len / 2 + fi_idx_in_hij * (flow_i.getPeriod() - flow_j.getPeriod())) % flow_j.getPeriod();
                            uint64_t fi_start = fi_mid - fi_len / 2;

                            int d = fi_mid - fj_start;
                            /* Queue cache constraint check */
                            if (std::abs(d) < dist)
                                return false;
                            if (d > 0 && fi_start < (fj_start + fj_len)) {
                                if (oss.str().empty()) oss << std::endl << "======start cache check======"<< std::endl;
                                uint64_t q_delay = fj_start + fj_len - fi_start;
                                collison_checked = true;
                                for (size_t k = j; k < fi_send_times;) {
                                    c.p5_traffic_offsets[flow_id][k][i] = c.p5_traffic_offsets[flow_id][k][i] + q_delay;
                                    k += fi_snd_times_hij;
                                }
                                oss << "route: " << route.toString() << std::endl;
                                oss << "totally send times: " << fi_send_times << " / " << j << std::endl;
                                oss << "cyc_id:     " << std::left << std::setw(10) << flow_i.getId()
                                    << "iso_id:     " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                                oss << "cyc_period: " << std::left << std::setw(10) << flow_i.getPeriod()
                                    << "iso_period: " << std::left << std::setw(10) << flow_j.getPeriod() << std::endl;
                                oss << "cyc_start:  " << std::left << std::setw(10) << fi_start
                                    << "iso_end:    " << std::left << std::setw(10) << fj_start + fj_len << "queue delay: " << q_delay << std::endl;
                                oss << "cyc_hop:    " << std::left << std::setw(10) << i
                                    << "iso_hop:    " << std::left << std::setw(10) << std::to_string(_flow_hop.second) << std::endl;
                            }
                        }
                    }
                    c.p5_traffic_offsets[flow_id][j][i + 1] = c.p5_traffic_offsets[flow_id][j][i] + fi_len + link.getPropSpeed() * link.getLen();
                }
            }
        }
        if (!oss.str().empty()) {
            oss << "=============end============="<< std::endl;
            spdlog::get("console")->debug(oss.str());
        }
        oss.str("");

        if (!checkCollision(link_flows, schedplus::P5, oss, c.p5_traffic_offsets))  return false;

        vector<uint64_t> link_gcl_size(link_flows.size());
        int i = 0;
        for (auto const &[link_id, flows_id_hop]: link_flows) {
            for (auto const &[flow_id, hop]: flows_id_hop) {
                link_gcl_size[i] = link_gcl_size[i] + c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
            }
            ++i;
        }

        /* Get variance of current solution */
        double mean = std::accumulate(link_gcl_size.begin(), link_gcl_size.end(), 0.0) / link_gcl_size.size();
        vector<double> diff(link_gcl_size.size());
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
        for (int i = 0; i < flows.size(); ++i) {
            auto &flow = flows[i];
            uint64_t offset = 0, route = 0;
            bool in_range;
            do {
                offset = X_base.offsets[i] + mu * (rnd01() - rnd01());
                int srcTransDelay =
                        flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
                in_range = (offset >= 0) && (offset < (flow.get().getPeriod() - srcTransDelay));
                if (flow.get().getRoutes().size() > 1) {
                    route = X_base.selected_route_idx[i] + mu * (rnd01() - rnd01());
                    in_range = in_range && route >= 0 && route < flow.get().getRoutes().size();
                } else {
                    route = 0;
                }
            } while (!in_range);
            X_new.offsets[i] = offset;
            X_new.selected_route_idx[i] = route;
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

    vector<double> calculate_MO_objectives(const GA_Type::thisChromosomeType &X) {
        return {
                X.middle_costs.variance,
                X.middle_costs.e2e,
                X.middle_costs.ddl
        };
    }

    void MO_report_generation(
            int generation_number,
            const EA::GenerationType<TTFlows, MyMiddleCost> &last_generation,
            const vector<unsigned int> &pareto_front) {
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
        vector<unsigned int> paretofront_indices = ga_obj.last_generation.fronts[0];
        for (unsigned int i: paretofront_indices) {
            /* set flow offset and route index */
            auto &X = ga_obj.last_generation.chromosomes[i];
            map<uint32_t, vector<uint32_t>> link_flows;
            for (auto &flow: flows) {
                uint32_t flow_id = flow.get().getId() - 1;
                flow.get().setOffset(X.genes.offsets[flow_id]);
                flow.get().setSelectedRouteInx(X.genes.selected_route_idx[flow_id]);
                for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                    link_flows[link.get().getId()].emplace_back(flow.get().getId());
                }
            }
            map<uint32_t, uint64_t> link_hyperperiod;
            setLinkHyperperiod(X.genes.selected_route_idx, link_hyperperiod);
            saveGCL(X.genes.offsets, X.genes.selected_route_idx);

            std::string route_file = output_location;
            route_file.append("/" + std::to_string(i) + "SmallRouting.xml");
            save_route(route_file, link_flows);

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

    void save_route(const std::string &route_file_location,
                    map<uint32_t, vector<uint32_t>> &link_flows) {
        spdlog::get("console")->info("Saving route: {}", route_file_location);
        map<node_idx, vector<uint32_t >> sw_links;
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

    void saveGCL(const std::string &gcl_file_location, const vector<uint64_t> &selected_route_idx) {
        std::string switch_gcl_file = gcl_file_location + "SmallGCL.xml";
        saveSwPortSchedule(switch_gcl_file, selected_route_idx);
        saveEsSchedule(gcl_file_location, selected_route_idx);
    }

    void saveSwPortSchedule(const std::string &sched_file_location, const vector<uint64_t> &selected_route_idx) {
        spdlog::get("console")->info("Saving gcl: {}", sched_file_location);
        pugi::xml_document xdoc;
        pugi::xml_node xdec = xdoc.prepend_child(pugi::node_declaration);
        xdec.append_attribute("version").set_value("1.0");
        xdec.append_attribute("encoding").set_value("utf-8");
        pugi::xml_node xschedules = xdoc.append_child("schedules");
        pugi::xml_node xhyperperiod = xschedules.append_child("defaultcycle");
        std::string hyp_str = std::to_string(hyperPeriod) + "ns";
        xhyperperiod.append_child(pugi::node_pcdata).set_value(hyp_str.c_str());

        map<uint32_t, uint64_t> link_hyperperiod;
        setLinkHyperperiod(selected_route_idx, link_hyperperiod);

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

    void saveEsSchedule(const std::string &sched_file_location, const vector<uint64_t> &selected_route_idx) {
        map<node_idx, vector<std::reference_wrapper<Flow>>> flowGroup;
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

        map<uint32_t, uint64_t> link_hyperperiod;
        setLinkHyperperiod(selected_route_idx, link_hyperperiod);

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

    void saveGCL(const vector<uint64_t> &offsets,
                 const vector<uint64_t> &selected_route_idx) {
        map<uint32_t, uint64_t> link_hyperperiod;
        setLinkHyperperiod(selected_route_idx, link_hyperperiod);
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
                link.get().mergeGCL();
                accumulatedDelay += trans_delay;
                prop_delay = link.get().getLen() * link.get().getPropSpeed();
                accumulatedDelay += prop_delay;
            }
        }
    }

    void saveEvent(const vector<uint64_t> &offsets,
                   const vector<uint64_t> &selected_route_idx,
                   const std::string &event_file_location) {
        events.clear()  ;
        std::ostringstream oss;
        spdlog::get("console")->set_level(spdlog::level::info);
        map<uint32_t, uint64_t> link_hyperperiod;
        setLinkHyperperiod(selected_route_idx, link_hyperperiod);
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
