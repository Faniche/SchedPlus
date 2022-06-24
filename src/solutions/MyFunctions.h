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
#include "../event/Event.h"
#include "SaveSolution.h"

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
            for (auto &link: flow.get().getRoutes()[selected_route_idx[flow.get().getId()]].getLinks()) {
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
                               map<uint32_t, map<uint8_t, uint64_t>> &p6_traffic_offsets) {
        uint64_t accDelay = offset;
        auto const &flow = flows[flow_id].get();
        for (int i = 0; i < flow.getRoutes()[route].getLinks().size(); ++i) {
            auto const &link = flow.getRoutes()[route].getLinks()[i].get();
            accDelay += link.getSrcNode()->getDpr();
            p6_traffic_offsets[flow_id][i] = accDelay;
            accDelay += flow.getFrameLength() * link.getSrcPort().getMacrotick();
            accDelay += link.getPropSpeed() * link.getLen();
        }
    }

    static bool checkCollisionHelp (const uint64_t fi_period, const uint64_t fi_mid, const uint64_t fi_len,
                             const uint64_t fj_period, const uint64_t fj_mid, const uint64_t fj_len) {
        uint64_t hyp_cycle = Util::lcm(fi_period, fj_period);
        uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
        if (fi_period < fj_period) {
            for (int k = 0; k < hyp_cycle / fj_period; ++k) {
                int d = (fj_mid + k * (fj_period - fi_period)) % fi_period - fi_mid;
                if (std::abs(d) < dist) {
                    SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        } else {
            for (int k = 0; k < hyp_cycle / fi_period; ++k) {
                int d = (fi_mid + k * (fi_period - fj_period)) % fj_period - fj_mid;
                if (std::abs(d) < dist) {
                    SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        }
        return true;
    }

    bool checkP5Collision (const map<uint32_t, vector<std::pair<uint32_t, uint8_t>>> &link_flows,
                         std::ostringstream &oss,
                         map<uint32_t, map<uint64_t, map<uint8_t, uint64_t>>> &traffic_offsets) {
        oss.str("");
        for (auto const &[link_id, flows_id_hop]: link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                if (flow_i.getPriorityCodePoint() != schedplus::P5) continue;
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                uint64_t fi_mid = traffic_offsets[fi_id][0][hop_i] + fi_len / 2;
                for (int j = i + 1; j < flows_id_hop.size(); ++j) {
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    if (flow_j.getPriorityCodePoint() != schedplus::P5) continue;
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    uint64_t fj_mid = traffic_offsets[fj_id][0][hop_j] + fj_len / 2;

                    if (!checkCollisionHelp(fi_period, fi_mid, fi_len, fj_period, fj_mid, fj_len)) {
                        oss << std::endl << "====collision check failed===="<< std::endl;
                        oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                            << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                        oss << "fi_period: " << std::left << std::setw(10) << fi_period
                            << "fj_period: " << std::left << std::setw(10) << fj_period << std::endl;
                        oss << "fi_hop:    " << std::left << std::setw(10) << i
                            << "fj_hop:    " << std::left << std::setw(10) << j << std::endl;
                        oss << "=============================="<< std::endl;
                        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), oss.str());
                        oss.str("");
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool checkP6Collision (const TTFlows &p, MyMiddleCost &c) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P6]) {
            auto const &flow = flows[flow_id].get();
            auto const &route = flows[flow_id].get().getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            for (int i = 0; i < route.size(); ++i) {
                uint32_t link_id = route[i].get().getId();
                uint64_t start = (c.p6_traffic_offsets[flow_id][i] % flow.getPeriod()) % c.link_min_period[link_id];
                uint64_t end = ((c.p6_traffic_offsets[flow_id][i] + flow.getFrameLength() * route[i].get().getSrcPort().getMacrotick()) % flow.getPeriod()) % c.link_min_period[link_id];
                c.link_ring[link_id].emplace_back(std::pair(start, end));
            }
        }

        for (auto &item: c.link_ring)
            std::sort(item.second.begin(), item.second.end(), compIntervals);

        for (auto &[link_id, intervals]: c.link_ring) {
            if ((intervals[intervals.size() - 1].second + schedplus::IFG_TIME > c.link_min_period[link_id])
                    && ((intervals[intervals.size() - 1].second + schedplus::IFG_TIME) % c.link_min_period[link_id] > intervals[0].first)) {
                SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "P6 collision check failed");
                return false;
            }
            for (int i = 0; i < intervals.size() - 1; ++i) {
                if (intervals[i].second + schedplus::IFG_TIME > intervals[i + 1].first) {
                    SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "P6 collision check failed");
                    return false;
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

    bool checkQueueCache (const TTFlows &p, MyMiddleCost &c, uint64_t &max_e2e, std::ostringstream &oss) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P5]) {
            auto const &flow = flows[flow_id].get();
            auto const &route = flows[flow_id].get().getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            for (size_t i = 0; i < fi_send_times; ++i)
                c.p5_traffic_offsets[flow_id][i][0] = p.offsets[flow_id] + i * flow_i.getPeriod();
            for (int i = 0; i < route.size(); ++i) {
                uint32_t link_id = route[i].get().getId();
                uint64_t start = (c.p5_traffic_offsets[flow_id][i] % flow.getPeriod()) % c.link_min_period[link_id];
                uint64_t end = ((c.p6_traffic_offsets[flow_id][i] + flow.getFrameLength() * route[i].get().getSrcPort().getMacrotick()) % flow.getPeriod()) % c.link_min_period[link_id];
                c.link_ring[link_id].emplace_back(std::pair(start, end));
            }
        }
        for (auto &flow_id: flowGroupPcp[schedplus::P5]) {
            auto const &flow_i = flows[flow_id].get();
            vector<uint32_t> flow_ids{flow_id};
            flow_ids.insert(flow_ids.end(), flowGroupPcp[schedplus::P6].begin(), flowGroupPcp[schedplus::P6].end());
            map<uint32_t, uint64_t> link_p6_hyperperiod;
            for (auto const &_flow_id: flow_ids) {
                auto const &flow = flows[_flow_id].get();
                auto const &route = flow.getRoutes()[p.selected_route_idx[_flow_id]];
                SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].route: {}", _flow_id, route.toString());
                for (auto const &link: route.getLinks()) {
                    uint32_t link_id = link.get().getId();
                    if (link_p6_hyperperiod.contains(link_id)) {
                        link_p6_hyperperiod[link_id] = Util::lcm(link_p6_hyperperiod[link_id], flow.getPeriod());
                    } else {
                        link_p6_hyperperiod[link_id] = flow.getPeriod();
                    }
                    SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].link[{}].hyp = {}", _flow_id, link_id, link_p6_hyperperiod[link_id]);
                }
            }
            for (auto const &item: link_p6_hyperperiod) {
                SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].link[{}].{}.hyp = {}", flow_id, item.first, links[item.first].get().getSrcNode()->getName(), item.second);
            }
            auto const &route = flow_i.getRoutes()[p.selected_route_idx[flow_id]];
            uint64_t hyp = 0;
            for (auto const &link: route.getLinks()) {
                if (hyp == 0)
                    hyp = link_p6_hyperperiod[link.get().getId()];
                else
                    hyp = Util::lcm(link_p6_hyperperiod[link.get().getId()], hyp);
            }
            uint64_t fi_send_times = hyp / flow_i.getPeriod();
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "flow[{}].route: {}", flow_id, route.toString());
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "flow[{}].period = {}, flow[{}] send {} times in {}", flow_id, flow_i.getPeriod(), flow_id, fi_send_times, hyp);
            for (size_t i = 0; i < fi_send_times; ++i)
                c.p5_traffic_offsets[flow_id][i][0] = p.offsets[flow_id] + i * flow_i.getPeriod();
            /* repeat each hop of flow i */
            for (size_t i = 0; i < route.getLinks().size(); ++i) {
                auto const &link = route.getLinks()[i].get();
                SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "flow[{}].hop[{}].link[{}].hyp = {}, transmit {} times, repeat {} times in hyp",
                                    flow_id, i, link.getId(), link_p6_hyperperiod[link.getId()], link_p6_hyperperiod[link.getId()] / flow_i.getPeriod(),
                                    hyp / link_p6_hyperperiod[link.getId()]);

                uint64_t fi_len = flow_i.getFrameLength() * link.getSrcPort().getMacrotick();
                uint64_t dist = fi_len / 2;
                for (size_t j = 0; j < fi_send_times; ++j) {
                    c.p5_traffic_offsets[flow_id][j][i] = c.p5_traffic_offsets[flow_id][j][i] + link.getSrcNode()->getDpr();
                    /* End to end latency check */
                    c.p5_e2e[flow_id][j] = c.p5_traffic_offsets[flow_id][j][i] - (p.offsets[flow_id] + j * flow_i.getPeriod());
                    if (c.p5_e2e[flow_id][j] > flow_i.getDeliveryGuarantees()[0].getLowerVal()) {
                        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "===e2e check failed===");
                        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "flow[{}].hop[{}].send[{}].e2e = {}", flow_id, i, j, c.p5_e2e[flow_id][j]);
                        return false;
                    }

                    if (c.p5_e2e[flow_id][j] > max_e2e)
                        max_e2e = c.p5_e2e[flow_id][j];

//                    bool collison_checked = false;
//                    if (collison_checked == false) {
                    for (auto const &_flow_hop: c.link_flows[link.getId()]) {
                        auto const &flow_j = flows[_flow_hop.first].get();
                        if (flow_j.getPriorityCodePoint() == schedplus::P5) continue;

                        uint64_t fj_start = c.p6_traffic_offsets[_flow_hop.first][_flow_hop.second];
                        uint64_t fj_len = flow_j.getFrameLength() * link.getSrcPort().getMacrotick();
                        uint64_t hij = Util::lcm(flow_i.getPeriod(), flow_j.getPeriod());

                        uint64_t fi_snd_times_hij = hij / flow_i.getPeriod();
                        uint64_t fi_idx_in_hij = j % fi_snd_times_hij;
                        uint64_t fi_mid = (c.p5_traffic_offsets[flow_id][j][i] + fi_len / 2 + fi_idx_in_hij * (flow_i.getPeriod() - flow_j.getPeriod())) % flow_j.getPeriod();
                        uint64_t fi_start = fi_mid - fi_len / 2;

                        int d = fi_mid - fj_start;
                        /* Queue cache constraint check */
                        if (std::abs(d) < dist) {
                            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "NO COLLISION CHECK FAILED, flow[{}] and flow[{}]", flow_id, flow_j.getId());
                            return false;
                        }
                        if (d > 0 && fi_start < (fj_start + fj_len)) {
                            if (oss.str().empty()) oss << std::endl << "======start cache check======"<< std::endl;
                            uint64_t q_delay = fj_start + fj_len - fi_start;
//                                collison_checked = true;
                            size_t count = 0;
                            for (size_t k = j; k < fi_send_times;) {
                                c.p5_traffic_offsets[flow_id][k][i] = c.p5_traffic_offsets[flow_id][k][i] + q_delay;
                                count++;
                                SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "cur: c.p5_traffic_offsets[{}][{}][{}] = {}", flow_id, k, i, c.p5_traffic_offsets[flow_id][k][i]);
                                k += fi_snd_times_hij;
                            }
                            int cache_times_in_super_hyp = count * c.link_hyperperiod[link.getId()] / hyp;
                            c.link_gcl_size[link.getId()] = c.link_gcl_size[link.getId()] - cache_times_in_super_hyp;
                            oss << "flow[" << flow_id << "].route: " << route.toString() << std::endl;
                            oss << "flow[" << flow_id << "] will cache " << count << " times with flow[" << flow_j.getId() << "]" << std::endl;
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
//                    }
                    c.p5_traffic_offsets[flow_id][j][i + 1] = c.p5_traffic_offsets[flow_id][j][i] + fi_len + link.getPropSpeed() * link.getLen();
                }
            }
        }
        if (!oss.str().empty()) {
            oss << "=============end============="<< std::endl;
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), oss.str());
        }
    }

    bool checkP6DDL (const TTFlows &p,
                     uint64_t &max_ddl) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P6]) {
            auto const &flow = flows[flow_id].get();
            uint64_t e2e = flow.getRoutes()[p.selected_route_idx[flow_id]].getE2E();
            uint64_t ddl = p.offsets[flow_id] + e2e;
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
            uint32_t flow_id = flow.get().getId();
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
            p.offsets[flow_id] = offset;
        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        spdlog::set_level(spdlog::level::debug);
        /* Check Isochronous traffic ddl guarantee. */
        uint64_t max_ddl = 0, max_e2e = 0;

        if (!checkP6DDL(p, max_ddl))  return false;
        if (!checkP5E2E(p.selected_route_idx, max_e2e)) return false;

        /* Set send offset of each hop */
        for (auto &flow_id: flowGroupPcp[schedplus::P6]) {
            setEachHopStartTimeP6(flow_id, p.offsets[flow_id], p.selected_route_idx[flow_id],c.p6_traffic_offsets);
        }

        setLinkHyperperiod(p.selected_route_idx, c.link_hyperperiod);
        for (int i = 0; i < p.offsets.size(); ++i) {
            auto const &route = flows[i].get().getRoutes()[p.selected_route_idx[i]].getLinks();
            for (auto const &link: route) {
                c.link_flowid[link.get().getId()].emplace_back(i);
            }
        }

        for (auto const &[link_id, flows_id]: c.link_flowid) {
            for (auto const &flow_id: flows_id) {
                if (!c.link_min_period.contains(link_id) || c.link_min_period[link_id] > flows[flow_id].get().getPeriod())
                    c.link_min_period[link_id] = flows[flow_id].get().getPeriod();
            }
        }

        if (!checkP6Collision(p, c))
            return false;

        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            for (int i = 0; i < flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id =  flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                c.link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }

        std::ostringstream oss("");

        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            for (auto const &[flow_id, hop]: flows_id_hop) {
                if (c.link_gcl_size.contains(link_id)) {
                    c.link_gcl_size[link_id] = c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
                }
                c.link_gcl_size[link_id] = c.link_gcl_size[link_id] + c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
            }
        }

        if (!checkQueueCache(p, c, max_e2e, oss))  return false;

        if (!checkP5Collision(c.link_flows, oss, c.p5_traffic_offsets))  return false;

        /* Get variance of current solution */
        vector<uint64_t> tmp(c.link_gcl_size.size());
        for (auto const &item: c.link_gcl_size) {
            tmp.emplace_back(item.second);
        }
        double mean = std::accumulate(tmp.begin(), tmp.end(), 0.0) / tmp.size();
        vector<double> diff(tmp.size());
        std::transform(tmp.begin(), tmp.end(), diff.begin(), [mean](double x) {
            return x - mean;
        });
        c.variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / tmp.size();

        /* set ddl and e2e for solution */
        c.ddl = max_ddl;
        c.e2e = max_e2e;
        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "*******Correct solution*******");
        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "\tlink\troute\toffset");
        std::for_each(flows.begin(), flows.end(),
                      [&](std::reference_wrapper<Flow> flow) {
                          SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "\t{}\t{}\t{}", flow.get().getId(),
                                                        p.selected_route_idx[flow.get().getId()],
                                                        p.offsets[flow.get().getId()]);
                      });
        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "******************************");
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
        std::string result = output_location;
        result.append("/solution_report.txt");
        std::ofstream out_file(result);
        for (unsigned int i: paretofront_indices) {
            /* set flow offset and route index */
            auto &X = ga_obj.last_generation.chromosomes[i];
            map<uint32_t, vector<uint32_t>> link_flows;
            for (auto &flow: flows) {
                uint32_t flow_id = flow.get().getId();
                flow.get().setOffset(X.genes.offsets[flow_id]);
                flow.get().setSelectedRouteInx(X.genes.selected_route_idx[flow_id]);
                for (auto &link: flow.get().getRoutes()[flow.get().getSelectedRouteInx()].getLinks()) {
                    link_flows[link.get().getId()].emplace_back(flow.get().getId());
                }
            }
            SaveSolution saveSolution(nodes, esList, swList, nodeMap, links, flows, output_location, hyperPeriod, events, X.middle_costs.link_hyperperiod);
            saveSolution.saveGCL(X.genes.offsets, X.genes.selected_route_idx);

            std::string route_file = output_location;
            route_file.append("/" + std::to_string(i) + "SmallRouting.xml");
            saveSolution.save_route(route_file, link_flows);

            std::string gcl_file = output_location;
            gcl_file.append("/" + std::to_string(i));
            saveSolution.saveGCL(gcl_file, X.genes.selected_route_idx);

            std::string route_file_name = std::to_string(i) + "SmallRouting.xml";
            std::string gcl_file_name = std::to_string(i) + "SmallGCL.xml";
            std::string ini_file = output_location;
            ini_file.append("/" + std::to_string(i) + "SmallTopology.ini");
            saveSolution.saveIni(route_file_name, gcl_file_name, ini_file, ned_file, i);

            std::string event_file = output_location;
            event_file.append("/" + std::to_string(i) + "event.txt");
            saveSolution.saveEvent(X.genes.offsets, X.genes.selected_route_idx, event_file);

            out_file
                    << i << "\t"
                    << X.middle_costs.e2e << "\t"
                    << X.middle_costs.ddl << "\t"
                    << X.middle_costs.variance << "\t" << std::endl;

        }
    }

};


#endif //SCHEDPLUS_MYFUNCTIONS_H
