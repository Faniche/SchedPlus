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
#include "../../lib/openGA/openGA.hpp"
#include "../event/Event.h"
#include "../include/env.h"
#include "../../lib/pugixml/pugixml.hpp"

class MyFunctions {
private:
    vector<Node *> nodes;
    vector<Node *> esList;
    vector<Node *> swList;
    map<node_idx, Node *> nodeMap;
    vector<std::reference_wrapper<DirectedLink>> links;
    vector<std::reference_wrapper<Flow>> flows;
    uint64_t hyperPeriod;
    vector<schedplus::Event> events;
    map<schedplus::PRIORITY_CODE_POINT, vector<uint32_t>> flowGroupPcp;

    void setLinkHyperperiod(const vector<uint64_t> &selected_route_idx, map<uint32_t, uint64_t> &link_hyperperiod) {
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

    void setEachHopStartTimeP6(const TTFlows &p, uint32_t flow_id,
                               map<uint32_t, map<uint8_t, uint64_t>> &p6_traffic_offsets) {
        auto const &flow = flows[flow_id].get();
        auto const &route_links = flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks();
        uint64_t accDelay = p.offsets[flow_id];
        for (int i = 0; i < route_links.size(); ++i) {
            auto const &link = route_links[i].get();
            accDelay += link.getSrcNode()->getDpr();
            p6_traffic_offsets[flow_id][i] = accDelay;
            accDelay += flow.getFrameLength() * link.getSrcPort().getMacrotick();
            accDelay += link.getPropSpeed() * link.getLen();
        }
        p6_traffic_offsets[flow_id][route_links.size()] = accDelay;
    }

    void setEachHopStartTimeP5(const TTFlows &p, MyMiddleCost &c, uint32_t flow_id) {
        if (c.p5_traffic_offsets.contains(flow_id)) {
            c.p5_traffic_offsets.erase(flow_id);
        }
        auto const &flow = flows[flow_id].get();
        auto const &route_links = flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks();
        uint64_t accDelay = p.offsets[flow_id];
        for (int i = 0; i < route_links.size(); ++i) {
            auto const &link = route_links[i].get();
            accDelay += link.getSrcNode()->getDpr();
            c.p5_traffic_offsets[flow_id][i][0] = accDelay;
            accDelay += flow.getFrameLength() * link.getSrcPort().getMacrotick();
            accDelay += link.getPropSpeed() * link.getLen();
        }
        c.p5_traffic_offsets[flow_id][route_links.size()][0] = accDelay;
        for (int i = 0; i < route_links.size(); ++i) {
            auto const &link = route_links[i].get();
            uint64_t transmit_times = c.link_hyperperiod[link.getId()] / flow.getPeriod();
            for (int j = 0; j < transmit_times; ++j) {
                c.p5_traffic_offsets[flow_id][i][j] = c.p5_traffic_offsets[flow_id][i][0] + j * flow.getPeriod();
                if (i + 1 == route_links.size()) {
                    c.p5_traffic_offsets[flow_id][i + 1][j] = c.p5_traffic_offsets[flow_id][i][0] + j * flow.getPeriod();
                }
            }
        }
    }

    static bool checkCollisionHelp(const uint64_t fi_period, const uint64_t fi_mid, const uint64_t fi_len,
                                   const uint64_t fj_period, const uint64_t fj_mid, const uint64_t fj_len) {
        uint64_t hpij = Util::lcm(fi_period, fj_period);
        uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
        if (fi_period < fj_period) {
            for (int k = 0; k < hpij / fj_period; ++k) {
                int d = (fj_mid + k * (fj_period - fi_period)) % fi_period - fi_mid;
                if (std::abs(d) <= dist) {
                    SPDLOG_LOGGER_TRACE(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        } else {
            for (int k = 0; k < hpij / fi_period; ++k) {
                int d = (fi_mid + k * (fi_period - fj_period)) % fj_period - fj_mid;
                if (std::abs(d) <= dist) {
                    SPDLOG_LOGGER_TRACE(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        }
        return true;
    }

    bool checkP5Collision(MyMiddleCost &c, std::ostringstream &oss) {
        oss.str("");
        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                if (flow_i.getPriorityCodePoint() != schedplus::P5) continue;
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                for (int j = i + 1; j < flows_id_hop.size(); ++j) {
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    if (flow_j.getPriorityCodePoint() != schedplus::P5) continue;
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    for (int k = 0; k < c.link_hyperperiod[link_id] / fi_period; ++k) {
                        uint64_t fi_mid = c.p5_traffic_offsets[fi_id][hop_i][k] +  fi_len / 2;
                        for (int l = 0; l < c.link_hyperperiod[link_id] / fj_period; ++l) {
                            uint64_t fj_mid = c.p5_traffic_offsets[fj_id][hop_j][l] +  fj_len / 2;
                            uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
                            int d = fi_mid - fj_mid;
                            if (std::abs(d) <= dist) {
                                oss << std::endl << "dist = " << dist << ", d = " << d << std::endl;
                                oss << "===P5 collision check failed===" << std::endl;
                                oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                                    << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                                oss << "fi_period: " << std::left << std::setw(10) << fi_period
                                    << "fj_period: " << std::left << std::setw(10) << fj_period << std::endl;
                                oss << "fi_hop:    " << std::left << std::setw(10) << std::to_string(hop_i)
                                    << "fj_hop:    " << std::left << std::setw(10) << std::to_string(hop_j) << std::endl;
                                oss << "===================+===========" << std::endl;
                                SPDLOG_LOGGER_DEBUG(spdlog::get("console"), oss.str());
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

    bool checkP6Collision(std::ostringstream &oss,
                          MyMiddleCost &c) {
        oss.str("");
        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                uint64_t fi_mid = 0;
                if (flow_i.getPriorityCodePoint() == schedplus::P5)
                    fi_mid = c.p5_traffic_offsets[fi_id][hop_i][0] + fi_len / 2;
                else
                    fi_mid = c.p6_traffic_offsets[fi_id][hop_i] + fi_len / 2;
                for (int j = i + 1; j < flows_id_hop.size(); ++j) {
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    uint64_t fj_mid = 0;
                    if (flow_j.getPriorityCodePoint() == schedplus::P5)
                        fj_mid = c.p5_traffic_offsets[fj_id][hop_j][0] + fj_len / 2;
                    else
                        fj_mid = c.p6_traffic_offsets[fj_id][hop_j] + fj_len / 2;

                    if (!checkCollisionHelp(fi_period, fi_mid, fi_len, fj_period, fj_mid, fj_len)) {
                        oss << std::endl << "===P6 collision check failed===" << std::endl;
                        oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                            << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                        oss << "fi_period: " << std::left << std::setw(10) << fi_period
                            << "fj_period: " << std::left << std::setw(10) << fj_period << std::endl;
                        oss << "fi_hop:    " << std::left << std::setw(10) << std::to_string(hop_i)
                            << "fj_hop:    " << std::left << std::setw(10) << std::to_string(hop_j) << std::endl;
                        oss << "===============================" << std::endl;
                        SPDLOG_LOGGER_TRACE(spdlog::get("console"), oss.str());
                        oss.str("");
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool checkP5E2E(const TTFlows &p, MyMiddleCost &c, uint64_t &max_e2e) {
        for (auto const &flow_id: flowGroupPcp[schedplus::P5]) {
            bool is_cached_flow = true;
            for (int i = 0; i < c.cached_flows.size(); ++i) {
                if (c.cached_flows[i] == flow_id) {
                    is_cached_flow = false;
                    break;
                }
            }
            if (!is_cached_flow) continue;
            auto const &flow = flows[flow_id];
            uint64_t e2e = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getE2E();
            if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                return false;
            /* the max e2e in current solution */
            if (e2e > max_e2e)
                max_e2e = e2e;
        }
        return true;
    }

    bool checkP5E2E(MyMiddleCost &c, uint64_t &max_e2e) {
        for (auto const &flow_id: c.cached_flows) {
            auto const &flow = flows[flow_id];
            uint64_t send_times = c.p5_traffic_offsets[flow_id][0].size();
            uint64_t final_hop = c.p5_traffic_offsets[flow_id].size() - 1;
            for (uint64_t i = 0; i < send_times; ++i) {
                uint64_t e2e = c.p5_traffic_offsets[flow_id][final_hop][i] - c.p5_traffic_offsets[flow_id][0][i];
                if (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal())
                    return false;
                /* the max e2e in current solution */
                if (e2e > max_e2e)
                    max_e2e = e2e;
                c.p5_e2e[flow_id][i] = e2e;
            }
        }
        return true;
    }

    bool checkQueueCache(const TTFlows &p, MyMiddleCost &c, std::ostringstream &oss) {
        spdlog::get("console")->set_level(spdlog::level::debug);
        for (auto cur = c.cached_flows.begin(); cur != c.cached_flows.end(); ++cur) {
            uint32_t flow_id = *cur;
            auto const &flow_i = flows[flow_id].get();
            auto const &route = flow_i.getRoutes()[p.selected_route_idx[flow_id]];
            uint64_t src_hyp = c.link_hyperperiod[route.getLinks()[0].get().getId()];
            uint64_t fi_send_times = src_hyp / flow_i.getPeriod();
            SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].route: {}", flow_id, route.toString());
            SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].period = {}, flow[{}] send {} times in {} us",
                                flow_id, flow_i.getPeriod(), flow_id, fi_send_times, src_hyp);
            /* Set send time at first hop */
            for (size_t i = 0; i < fi_send_times; ++i)
                c.p5_traffic_offsets[flow_id][0][i] = p.offsets[flow_id] + i * flow_i.getPeriod();
            /* repeat each hop of flow i */
            bool could_cache = true;
            for (size_t i = 0; i < route.getLinks().size() && could_cache; ++i) {
                auto const &link = route.getLinks()[i].get();
                uint64_t fi_len = flow_i.getFrameLength() * link.getSrcPort().getMacrotick();
                uint64_t dist = fi_len / 2;
                for (size_t j = 0; j < fi_send_times && could_cache; ++j) {
                    c.p5_traffic_offsets[flow_id][i][j] = c.p5_traffic_offsets[flow_id][i][j] + link.getSrcNode()->getDpr();
                    uint32_t collision_flow_id = UINT32_MAX;
                    for (auto const &_flow_hop: c.link_flows[link.getId()]) {
                        auto const &flow_j = flows[_flow_hop.first].get();
                        if (flow_j.getPriorityCodePoint() != schedplus::P6) continue;
                        uint64_t fj_start = c.p6_traffic_offsets[_flow_hop.first][_flow_hop.second];
                        uint64_t fj_len = flow_j.getFrameLength() * link.getSrcPort().getMacrotick();
                        uint64_t hij = Util::lcm(flow_i.getPeriod(), flow_j.getPeriod());
                        uint64_t fi_snd_times_hij = hij / flow_i.getPeriod();
                        uint64_t fi_idx_in_hij = j % fi_snd_times_hij;
                        uint64_t fi_mid = (c.p5_traffic_offsets[flow_id][i][j] + fi_len / 2 +
                                           fi_idx_in_hij * (flow_i.getPeriod() - flow_j.getPeriod())) %
                                          flow_j.getPeriod();

                        uint64_t fi_start = fi_mid - fi_len / 2;

                        int d = fi_mid - fj_start;
                        /* Queue cache constraint check */
                        if (std::abs(d) <= dist) {
                            SPDLOG_LOGGER_TRACE(spdlog::get("console"),
                                                "NO COLLISION CHECK FAILED, flow[{}] and flow[{}]", flow_id,
                                                flow_j.getId());
                            return false;
                        }
                        if (d > 0 && fi_start < (fj_start + fj_len + schedplus::IFG_TIME)) {
                            if (collision_flow_id != UINT32_MAX && collision_flow_id != flow_j.getId()) {
                                return false;
//                                c.cached_flows.erase(--cur);
//                                c.p5_traffic_offsets.erase(flow_id);
//                                setEachHopStartTimeP5(p, c, flow_id);
//                                could_cache = false;
//                                break;
                            }
                            if (collision_flow_id != UINT32_MAX && collision_flow_id == flow_j.getId())
                                continue;
                            collision_flow_id = flow_j.getId();

                            if (oss.str().empty()) oss << std::endl << "======start cache check======" << std::endl;
                            uint64_t q_delay = fj_start + fj_len + schedplus::IFG_TIME - fi_start;
                            size_t count = 0;
                            for (size_t k = j; k < fi_send_times;) {
                                c.p5_traffic_offsets[flow_id][i][k] = c.p5_traffic_offsets[flow_id][i][k] + q_delay;
                                count++;
                                SPDLOG_LOGGER_TRACE(spdlog::get("console"),
                                                    "cur: c.p5_traffic_offsets[{}][{}][{}] = {}", flow_id, i, k,
                                                    c.p5_traffic_offsets[flow_id][i][k]);
                                k += fi_snd_times_hij;
                            }
                            if (links[link.getId()].get().getSrcNode()->getNodeType() == SWITCH) {
                                int cache_times_in_super_hyp = count * c.link_hyperperiod[link.getId()] / src_hyp;
                                c.link_gcl_size[link.getId()] =
                                        c.link_gcl_size[link.getId()] - cache_times_in_super_hyp;
                                if (c.link_gcl_merge_count.contains(link.getId()))
                                    c.link_gcl_merge_count[link.getId()] = c.link_gcl_merge_count[link.getId()] + count;
                                else
                                    c.link_gcl_merge_count[link.getId()] = count;
                            }
                            oss << "flow[" << flow_id << "].route: " << route.toString() << std::endl;
                            oss << "flow[" << flow_id << "] will cache " << count << " time with flow[" << flow_j.getId() << "]"
                                << std::endl;
                        }
                    }
                    if (could_cache) {
                        c.p5_traffic_offsets[flow_id][i + 1][j] =
                                c.p5_traffic_offsets[flow_id][i][j] + fi_len + link.getPropSpeed() * link.getLen();
                    }
                }
            }
        }

        for (auto &flow_id: c.cached_flows) {
            auto const &flow_i = flows[flow_id].get();
            auto const &route = flow_i.getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            uint64_t src_hyp = c.link_hyperperiod[route[0].get().getId()];
            uint64_t src_send_times = src_hyp / flow_i.getPeriod();
            for (size_t i = 1; i < route.size(); ++i) {
                size_t cur_send_times = c.link_hyperperiod[route[i].get().getId()] / flow_i.getPeriod();
                for (size_t j = src_send_times; j < cur_send_times; ++j) {
                    c.p5_traffic_offsets[flow_id][i][j] = c.p5_traffic_offsets[flow_id][i][j % src_send_times] + src_hyp * (j / src_send_times);
                }
            }
        }
        if (!oss.str().empty()) {
            oss << "=============end=============" << std::endl;
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), oss.str());
        }

        return true;
    }

    bool checkP6DDL(const TTFlows &p,
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
    MyFunctions(vector<Node *> nodes,
                vector<Node *> esList,
                vector<Node *> swList,
                map<node_idx, Node *> nodeMap,
                vector<DirectedLink> &_links,
                vector<Flow> &_flows,
                map<schedplus::PRIORITY_CODE_POINT, vector<uint32_t>> _flowGroupPcp) :
            nodes(std::move(nodes)),
            esList(std::move(esList)),
            swList(std::move(swList)),
            nodeMap(std::move(nodeMap)),
            flowGroupPcp(std::move(_flowGroupPcp)) {

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
            uint64_t e2e = 0;
            if (flow.get().getRoutes().size() == 1) route_idx = 0;
            else {
                if (flow.get().getPriorityCodePoint() == schedplus::P5 ||
                flow.get().getPriorityCodePoint() == schedplus::P6) {
                    do {
                        route_idx = (flow.get().getRoutes().size()) * rnd01();
                        e2e = flow.get().getRoutes()[route_idx].getE2E();
                    } while (e2e > flow.get().getDeliveryGuarantees()[0].getLowerVal());
                }
            }

            p.selected_route_idx[flow_id] = route_idx;
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            e2e = flow.get().getRoutes()[route_idx].getE2E();
            uint64_t up_bound = flow.get().getPeriod() - srcTransDelay;
            if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                uint64_t tmp = flow.get().getDeliveryGuarantees()[0].getLowerVal() - e2e;
                up_bound = std::min(up_bound, tmp);
            }
            uint64_t offset = up_bound * rnd01();
            if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                uint64_t ddl = 0;
                do {
                    offset = up_bound * rnd01();
                    ddl = e2e + offset;
                } while (ddl > flow.get().getDeliveryGuarantees()[0].getLowerVal());
            }
            p.offsets[flow_id] = offset;
        }
    }

    bool eval_solution(const TTFlows &p, MyMiddleCost &c) {
        /* Check Isochronous traffic ddl guarantee. */
        uint64_t max_ddl = 0, max_e2e = 0;

        if (!checkP6DDL(p, max_ddl))
            return false;

        /* Set send offset of each hop */
        for (auto &flow_id: flowGroupPcp[schedplus::P6]) {
            setEachHopStartTimeP6(p, flow_id, c.p6_traffic_offsets);
        }

        setLinkHyperperiod(p.selected_route_idx, c.link_hyperperiod);
        for (auto const &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            bool couldCache = true;
            if (flow.get().getPriorityCodePoint() == schedplus::P5) {
                auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow_id]];
                uint64_t src_hyp = c.link_hyperperiod[route.getLinks()[0].get().getId()];
                for (int i = 1; i < route.getLinks().size(); ++i) {
                    auto const &link = route.getLinks()[i];
                    if (!(c.link_hyperperiod[link.get().getId()] >= src_hyp)
                            && (c.link_hyperperiod[link.get().getId()] % src_hyp == 0)) {
                        couldCache = false;
                        break;
                    }
                }
            }
            if (flow.get().getPriorityCodePoint() == schedplus::P5) {
                if (couldCache) {
                    c.cached_flows.emplace_back(flow_id);
                } else
                    setEachHopStartTimeP5(p, c, flow_id);
                continue;
            }

            for (int i = 0; i < flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                c.link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }

        std::ostringstream oss("");

        if (!checkP6Collision(oss, c))
            return false;

        if (!checkP5E2E(p, c, max_e2e))
            return false;

        for (auto const &flow_id: c.cached_flows) {
            auto const &flow = flows[flow_id].get();
            for (int i = 0; i < flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id = flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                c.link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }

        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            if (links[link_id].get().getSrcNode()->getNodeType() == SWITCH)
            for (auto const &[flow_id, hop]: flows_id_hop) {
                if (!c.link_gcl_size.contains(link_id)) {
                    c.link_gcl_size[link_id] = c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
                } else
                    c.link_gcl_size[link_id] =
                        c.link_gcl_size[link_id] + c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
            }
        }

        if (!checkQueueCache(p, c, oss))
            return false;

        if (!checkP5E2E(c, max_e2e))
            return false;

        if (!checkP5Collision(c, oss))
            return false;

        /* Get variance of current solution */
        vector<uint64_t> tmp;
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
        c.ddl = (double)max_ddl;
        c.e2e = (double)max_e2e;
        c.total_transmit = (double)*std::max_element(p.offsets.begin(), p.offsets.end())
                                - (double)*std::min_element(p.offsets.begin(),p.offsets.end());
//        spdlog::set_level(spdlog::level::info);
//        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "*******Correct solution*******");
//        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "\tlink\troute\toffset");
//        std::for_each(flows.begin(), flows.end(),
//                      [&](std::reference_wrapper<Flow> flow) {
//                          SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "\t{}\t{}\t{}", flow.get().getId(),
//                                              p.selected_route_idx[flow.get().getId()],
//                                              p.offsets[flow.get().getId()]);
//                      });
//        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "******************************");
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
//                X.middle_costs.e2e,
//                X.middle_costs.ddl,
                X.middle_costs.total_transmit
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
        std::string result = OUT_LOCATION_NOWAIT;
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
            saveGCL(X.genes, X.middle_costs);

            std::string route_file = OUT_LOCATION_NOWAIT;
            route_file.append("/" + std::to_string(i) + "_SmallRouting.xml");
            save_route(X.genes, X.middle_costs, route_file);

            std::string gcl_file = OUT_LOCATION_NOWAIT;
            gcl_file.append("/" + std::to_string(i));
            saveGCL(gcl_file, X.middle_costs);

            std::string route_file_name = std::to_string(i) + "_SmallRouting.xml";
            std::string gcl_file_name = std::to_string(i) + "_SmallGCL.xml";
            std::string ini_file = OUT_LOCATION_NOWAIT;
            ini_file.append("/" + std::to_string(i) + "_SmallTopology.ini");
            saveIni(route_file_name, gcl_file_name, ini_file, ned_file, i);

            std::string event_file = OUT_LOCATION_NOWAIT;
            event_file.append("/" + std::to_string(i) + "_event.txt");
            saveEvent(X.genes, X.middle_costs, event_file);

            out_file
                    << std::setw(3) << i
//                    << std::setw(20) << X.middle_costs.e2e
//                    << std::setw(20) << X.middle_costs.ddl
                    << std::setw(20) << X.middle_costs.variance
                    << std::setw(20) << (uint64_t)X.middle_costs.total_transmit << std::endl;
            for (auto const&[link_id, gcl_merge_count]: X.middle_costs.link_gcl_merge_count) {
                out_file << "link[" << link_id << "] merge " << gcl_merge_count << " times" << std::endl;
            }
        }
    }

    void saveEvent(const TTFlows &p, MyMiddleCost &c, const std::string& event_file) {
        events.clear();
        spdlog::get("console")->set_level(spdlog::level::info);

        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            for (int i = 0; i < route.size(); ++i) {
                auto &link = route[i].get();
                uint64_t time_interval = link.getSrcPort().getMacrotick() * flow.get().getFrameLength();
                if (flow.get().getPriorityCodePoint() == schedplus::P5) {
                    for (auto const &[j, offset]: c.p5_traffic_offsets[flow_id][i]) {
                        schedplus::Event event(offset, offset + time_interval, link.getSrcNode(), &flow.get(), i);
                        if (i == 0)
                            event.setType(schedplus::TRANSMIT);
                        else
                            event.setType(schedplus::FORWARD);
                        events.push_back(event);
                    }
                    if (i + 1 == route.size()) {
                        for (auto const &[j, offset]: c.p5_traffic_offsets[flow_id][i + 1]) {
                            schedplus::Event event(offset, offset + time_interval, link.getDestNode(), &flow.get(), i);
                            event.setType(schedplus::RECEIVE);
                            events.push_back(event);
                        }
                    }
                } else if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                    size_t send_times = c.link_hyperperiod[link.getId()] / flow.get().getPeriod();
                    for (int j = 0; j < send_times; ++j) {
                        uint64_t offset = c.p6_traffic_offsets[flow_id][i] + j * flow.get().getPeriod();
                        schedplus::Event event1(offset, offset + time_interval, link.getSrcNode(), &flow.get(), i);
                        if (i == 0)
                            event1.setType(schedplus::TRANSMIT);
                        else
                            event1.setType(schedplus::FORWARD);
                        if (i + 1 == route.size()) {
                            uint64_t rcv_time = c.p6_traffic_offsets[flow_id][i + 1] + j * flow.get().getPeriod();
                            schedplus::Event event2(rcv_time,
                                                    rcv_time + time_interval,
                                                    link.getDestNode(), &flow.get(), i);
                            event2.setType(schedplus::RECEIVE);
                            events.push_back(event2);
                        }

                        events.push_back(event1);
                    }


                }
            }
        }

        schedplus::Event::sortEvent(events);
        std::ofstream output;
        output.open(event_file);
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

    void saveGCL(const TTFlows &p, MyMiddleCost &c) {
        for (auto link: links) {
            if (link.get().getSrcPort().getGateControlList().empty()) continue;
            link.get().clearGateControlEntry();
        }
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            for (int i = 1; i < route.size(); ++i) {
                auto &link = route[i].get();
                uint64_t time_interval = link.getSrcPort().getMacrotick() * flow.get().getFrameLength();
                if (flow.get().getPriorityCodePoint() == schedplus::P5) {
                    for (auto const &[j, offset]: c.p5_traffic_offsets[flow_id][i]) {
                        GateControlEntry gateControlEntry;
                        gateControlEntry.setStartTime(offset);
                        gateControlEntry.setTimeIntervalValue(time_interval);
                        for (int k = 0; k < 8; ++k) {
                            if (flow.get().getPriorityCodePoint() == k)
                                gateControlEntry.setGateStatesValue(k, GATE_OPEN);
                            else
                                gateControlEntry.setGateStatesValue(k, GATE_CLOSE);
                        }
                        link.addGateControlEntry(gateControlEntry);
                    }
                } else if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                    uint64_t offset = c.p6_traffic_offsets[flow_id][i];
                    size_t send_times = c.link_hyperperiod[link.getId()] / flow.get().getPeriod();
                    for (int j = 0; j < send_times; ++j) {
                        GateControlEntry gateControlEntry;
                        gateControlEntry.setStartTime(offset + j * flow.get().getPeriod());
                        gateControlEntry.setTimeIntervalValue(time_interval);
                        for (int k = 0; k < 8; ++k) {
                            if (flow.get().getPriorityCodePoint() == k)
                                gateControlEntry.setGateStatesValue(k, GATE_OPEN);
                            else
                                gateControlEntry.setGateStatesValue(k, GATE_CLOSE);
                        }
                        link.addGateControlEntry(gateControlEntry);
                    }
                }
            }
        }

        for (auto link: links) {
            if (link.get().getSrcPort().getGateControlList().empty()) continue;
            link.get().sortGCL();
            link.get().mergeGCL();
        }
    }

    void save_route(const TTFlows &p, MyMiddleCost &c,
                                  const std::string &route_file_location) {
        spdlog::get("console")->info("Saving route: {}", route_file_location);
        map<node_idx, vector<uint32_t >> sw_links;
        for (auto const &sw: swList) {
            for (auto const &[link_id, flowids]: c.link_flows) {
                if (links[link_id].get().getSrcNode() == sw) {
                    sw_links[sw->getId()].emplace_back(link_id);
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
                        for (auto const &[flow_id, hop]: c.link_flows[link_id]) {
                            uint32_t _flow_id = flow_id + 1;
                            pugi::xml_node xmulticast_addr = xforward.append_child("multicastAddress");
                            pugi::xml_attribute xmac = xmulticast_addr.append_attribute("macAddress");
                            std::string mac = "255-0-00-00-00-" + std::to_string(_flow_id);
                            if (_flow_id < 10) {
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
        xdoc.save_file(route_file_location.c_str());
    }

    void saveGCL(const std::string &gcl_file_location, MyMiddleCost &c) {
        std::string switch_gcl_file = gcl_file_location + "_SmallGCL.xml";
        saveSwPortSchedule(switch_gcl_file, c);
        saveEsSchedule(gcl_file_location, c);
    }

    void saveSwPortSchedule(const std::string &sched_file_location, MyMiddleCost &c) {
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
                            std::string cyc_time_str = std::to_string(c.link_hyperperiod[link_id]) + "ns";
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
                            if (cur < c.link_hyperperiod[link_id]) {
                                uint64_t gap = c.link_hyperperiod[link_id] - cur;
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

    void saveEsSchedule(const std::string &sched_file_location, MyMiddleCost &c) {
        map<node_idx, vector<std::reference_wrapper<Flow>>> flowGroup;
        /* Group the flow with src */
        for (auto &flow: flows) {
            node_idx key = flow.get().getSrc()->getId();
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
                    cyc_str = std::to_string(c.link_hyperperiod[link.get().getId()]) + "ns";
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
                std::string dest_str = "255:0:00:00:00:" + std::to_string(flow.get().getId() + 1);
                if (flow.get().getId() + 1 < 10) {
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
            std::string es_traffic_gen = sched_file_location + "_" + es->getName() + ".xml";
            xesdoc.save_file(es_traffic_gen.c_str());
        }
    }



    void saveIni(const std::string &route_file,
                               const std::string &gcl_file,
                               const std::string &ini_file,
                               const std::string &ned_file,
                               uint64_t solution_id) const {
        std::ofstream output(ini_file);
        output << "[General]\n"
                  "network = " << ned_file << "\n"
                                              "\n"
                                              "record-eventlog = false \n"
                                              "debug-on-errors = true\n"
                                              "result-dir = result/" << ned_file << "\n"
                                                                                    "sim-time-limit ="
               << std::to_string(hyperPeriod / 1000000) << "ms\n"
                                                           "\n"
                                                           "# debug\n"
                                                           "**.displayAddresses = false\n"
                                                           "**.verbose = false" << std::endl;
        output << "# MAC Addresses" << std::endl;
        for (auto const &es: esList) {
            node_idx id = es->getId() + 1;
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
                if (Util::isPortInUse(((Switch *) sw)->getPorts()[i], flows)) {
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
            if (Util::isPortInUse(((EndSystem *) es)->getPort(), flows)) {
                output << "**." << es->getName() << R"(.trafGenSchedApp.initialSchedule = xmldoc("xml/)"
                       << std::to_string(solution_id) << "_" << es->getName() << R"(.xml"))" << std::endl;
            }
        }
    }
};


#endif //SCHEDPLUS_MYFUNCTIONS_H
