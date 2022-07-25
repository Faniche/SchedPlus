//
// Created by faniche on 22-7-10.
//

#ifndef SCHEDPLUS_WAIT_H
#define SCHEDPLUS_WAIT_H

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
#include "SaveSolution.h"

class Wait {
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

    void setEachHopStartTime(const TTFlows &p, MyMiddleCost &c) {
        for (int i = 0; i < p.offsets.size(); ++i) {
            auto &flow = flows[i].get();
            auto &route_links = flow.getRoutes()[p.selected_route_idx[i]].getLinks();
            uint64_t accDelay = p.offsets[i];
            for (int j = 0; j < route_links.size(); ++j) {
                auto const &link = route_links[j].get();
                accDelay += link.getSrcNode()->getDpr();
                c.traffic_offsets[i][j] = accDelay;
                accDelay += flow.getFrameLength() * link.getSrcPort().getMacrotick();
                accDelay += link.getPropSpeed() * link.getLen();
            }
            c.traffic_offsets[i][route_links.size()] = accDelay;
            if (flow.getPriorityCodePoint() == schedplus::P5)
                c.ddl_or_e2e[i] = accDelay - p.offsets[i];
            else if (flow.getPriorityCodePoint() == schedplus::P6)
                c.ddl_or_e2e[i] = accDelay;
        }
    }


    static bool checkCollisionHelp(const uint64_t fi_period, const uint64_t fi_mid, const uint64_t fi_len,
                                   const uint64_t fj_period, const uint64_t fj_mid, const uint64_t fj_len) {
        uint64_t hpij = Util::lcm(fi_period, fj_period);
        uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
        if (fi_period < fj_period) {
            uint64_t _fi_mid = fi_mid % fi_period;
            for (int k = 0; k < hpij / fj_period; ++k) {
                int d = (fj_mid + k * fj_period) % fi_period - _fi_mid;
                if (std::abs(d) <= dist) {
                    SPDLOG_LOGGER_TRACE(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        } else {
            uint64_t _fj_mid = fj_mid % fj_period;
            for (int k = 0; k < hpij / fi_period; ++k) {
                int d = (fi_mid + k * fi_period) % fj_period - _fj_mid;
                if (std::abs(d) <= dist) {
                    SPDLOG_LOGGER_TRACE(spdlog::get("console"), "dist = {}, d = {}", dist, d);
                    return false;
                }
            }
        }
        return true;
    }

    bool checkCollision(std::ostringstream &oss, MyMiddleCost &c) {
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
                fi_mid = c.traffic_offsets[fi_id][hop_i] + fi_len / 2;
                for (int j = i + 1; j < flows_id_hop.size(); ++j) {
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    uint64_t fj_mid = 0;
                    fj_mid = c.traffic_offsets[fj_id][hop_j] + fj_len / 2;
                    if (!checkCollisionHelp(fi_period, fi_mid, fi_len, fj_period, fj_mid, fj_len)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool checkDDLorE2E(MyMiddleCost &c, uint64_t &max_ddl, uint64_t &max_e2e) {
        for (int i = 0; i < c.ddl_or_e2e.size(); ++i) {
            auto const &flow = flows[i].get();
            if (c.ddl_or_e2e[i] >= flow.getDeliveryGuarantees()[0].getLowerVal())
                return false;
            if (flow.getPriorityCodePoint() == schedplus::P5) {
                if (max_e2e < c.ddl_or_e2e[i])
                    max_e2e = c.ddl_or_e2e[i];
            } else if (flow.getPriorityCodePoint() == schedplus::P6)
                if (max_ddl < c.ddl_or_e2e[i])
                    max_ddl = c.ddl_or_e2e[i];
        }
        return true;
    }

    bool checkP5E2E(const TTFlows &p, MyMiddleCost &c) {
        for (auto const &flow_id: c.cached_flows) {
            auto const &flow = flows[flow_id].get();
            uint64_t dg_low_val = flow.getDeliveryGuarantees()[0].getLowerVal();
            auto const &route = flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            size_t route_size = route.size();
            size_t fi_src_snd_times = c.link_hyperperiod[route[0].get().getId()] / flow.getPeriod();
            for (int i = 0; i < fi_src_snd_times; ++i) {
                uint64_t final = c.p5_traffic_offsets[flow_id][route.size()][i];
                uint64_t first = c.p5_traffic_offsets[flow_id][0][i];
                uint64_t e2e = final - first;
                if (e2e >= dg_low_val)
                    return false;
                if (c.e2e < e2e)
                    c.e2e = e2e;
            }
        }
        return true;
    }

    bool checkQueueCache(const TTFlows &p, MyMiddleCost &c, std::ostringstream &oss) {
        for (auto const &flowId: c.cached_flows) {
            auto const &flow_i = flows[flowId].get();
            auto const &route = flow_i.getRoutes()[p.selected_route_idx[flowId]];
            uint64_t srcHyp = c.link_hyperperiod[route.getLinks()[0].get().getId()];
            uint64_t srcSendTimes = srcHyp / flow_i.getPeriod();
            SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].route: {}", flowId, route.toString());
            SPDLOG_LOGGER_TRACE(spdlog::get("console"), "flow[{}].period = {}, flow[{}] send {} times in {} us",
                                flowId, flow_i.getPeriod(), flowId, srcSendTimes, srcHyp);
            /* Set send time at first hop */
            for (size_t i = 0; i < srcSendTimes; ++i)
                c.p5_traffic_offsets[flowId][0][i] = p.offsets[flowId] + i * flow_i.getPeriod();
            /* repeat each hop of flow i */
            for (size_t i = 0; i < route.getLinks().size(); ++i) {
                auto const &link = route.getLinks()[i].get();
                size_t count = 0;
                uint64_t fi_len = flow_i.getFrameLength() * link.getSrcPort().getMacrotick();
                uint64_t dist = fi_len / 2;
                size_t fi_cur_snd_times = c.link_hyperperiod[link.getId()] / flow_i.getPeriod();
                for (size_t j = 0; j < fi_cur_snd_times; ++j) {
                    if (j >= srcSendTimes) {
                        c.p5_traffic_offsets[flowId][i][j] =
                                c.p5_traffic_offsets[flowId][i][j % srcSendTimes] + j / srcSendTimes * srcHyp;
                    } else {
                        c.p5_traffic_offsets[flowId][i][j] += link.getSrcNode()->getDpr();
                    }
                    uint64_t fi_start = c.p5_traffic_offsets[flowId][i][j];
                    uint64_t fi_mid = fi_start + fi_len / 2;
                    for (auto const &[fj_id, hop_j]: c.link_flows[link.getId()]) {
                        auto const &flow_j = flows[fj_id].get();
                        /* check queue cache with isochronous flows */
                        if (flow_j.getPriorityCodePoint() != schedplus::P6) continue;
                        uint64_t fj_start = c.traffic_offsets[fj_id][hop_j] % flow_j.getPeriod();
                        int d = fi_mid % flow_j.getPeriod() - fj_start;
                        if (std::abs(d) <= dist) {
                            SPDLOG_LOGGER_TRACE(spdlog::get("console"),
                                                "NO COLLISION CHECK FAILED, flow[{}] and flow[{}]", flowId,
                                                flow_j.getId());
                            return false;
                        }
                        uint64_t fj_len = flow_j.getFrameLength() * link.getSrcPort().getMacrotick();
                        if (d > 0 && fi_start % flow_j.getPeriod() < (fj_start + fj_len + schedplus::IFG_TIME)) {
                            if (j > srcSendTimes)
                                return false;
                            if (flow_i.getPeriod() % flow_j.getPeriod() != 0)
                                return false;
                            uint64_t q_delay = fj_start + fj_len + schedplus::IFG_TIME - fi_start % flow_j.getPeriod();
                            c.total_cache += q_delay;
                            c.p5_traffic_offsets[flowId][i][j] += q_delay;
                            count++;
                        }
                    }
                    if (j < srcSendTimes) {
                        c.p5_traffic_offsets[flowId][i + 1][j] =
                                c.p5_traffic_offsets[flowId][i][j] + fi_len + link.getPropSpeed() * link.getLen();
                    }
                }
                count *= fi_cur_snd_times / srcSendTimes;
                if (links[link.getId()].get().getSrcNode()->getNodeType() == SWITCH) {
                    c.link_gcl_size[link.getId()] =
                            c.link_gcl_size[link.getId()] - count;
                    if (c.link_gcl_merge_count.contains(link.getId()))
                        c.link_gcl_merge_count[link.getId()] += count;
                    else
                        c.link_gcl_merge_count[link.getId()] = count;
                }
            }
        }
        return true;
    }

    bool checkP5CollisionCachedWithUncached(MyMiddleCost &c, std::ostringstream &oss) {
        oss.str("");
        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                /* select uncached flow_i in P5 */
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                if (flow_i.getPriorityCodePoint() != schedplus::P5) continue;
                bool isUnCachedFlow = std::any_of(c.uncached_flows.begin(), c.uncached_flows.end(),
                                                  [&fi_id](uint32_t flowId) {
                                                      return fi_id == flowId;
                                                  });
                if (!isUnCachedFlow) continue;
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                uint64_t fi_mid = (c.traffic_offsets[fi_id][hop_i] + fi_len / 2) % fi_period;
                for (int j = 0; j < flows_id_hop.size(); ++j) {
                    /* select cached flow_i in P5 */
                    uint32_t fj_id = flows_id_hop[j].first;
                    auto const &flow_j = flows[fj_id].get();
                    if (flow_j.getPriorityCodePoint() != schedplus::P5) continue;
                    bool isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                                    [&fj_id](uint32_t flowId) {
                                                        return fj_id == flowId;
                                                    });
                    if (!isCachedFlow) continue;
                    uint64_t fj_period = flow_j.getPeriod();
                    uint64_t fj_len = flow_j.getFrameLength() * mt;
                    uint8_t hop_j = flows_id_hop[j].second;
                    uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
                    for (int k = 0; k < c.p5_traffic_offsets[fj_id][hop_j].size(); ++k) {
                        uint64_t fj_mid = (c.p5_traffic_offsets[fj_id][hop_j][k] + fj_len / 2) % fi_period;
                        int d = fi_mid - fj_mid;
                        if (std::abs(d) <= dist) {
                            oss << std::endl << "dist = " << dist << ", d = " << d << std::endl;
                            oss << "====P5 mix flows collision check failed====" << std::endl;
                            oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                                << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                            oss << "fi_mid:    " << std::left << std::setw(10) << fi_mid
                                << "fj_mid:    " << std::left << std::setw(10) << fj_mid << std::endl;
                            oss << "fi_period: " << std::left << std::setw(10) << fi_period
                                << "fj_period: " << std::left << std::setw(10) << fj_period << std::endl;
                            oss << "fi_hop:    " << std::left << std::setw(10) << std::to_string(hop_i)
                                << "fj_hop:    " << std::left << std::setw(10) << std::to_string(hop_j) << std::endl;
                            oss << "===========================================" << std::endl;
                            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), oss.str());
                            oss.str("");
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    bool checkP5CollisionCached(MyMiddleCost &c, std::ostringstream &oss) {
        oss.str("");
        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            uint8_t mt = links[link_id].get().getSrcPort().getMacrotick();
            for (int i = 0; i < flows_id_hop.size(); ++i) {
                /* select uncached flow_i in P5 */
                uint32_t fi_id = flows_id_hop[i].first;
                auto const &flow_i = flows[fi_id].get();
                if (flow_i.getPriorityCodePoint() != schedplus::P5) continue;
                bool isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                                [&fi_id](uint32_t flowId) {
                                                    return fi_id == flowId;
                                                });
                if (!isCachedFlow) continue;
                uint64_t fi_period = flow_i.getPeriod();
                uint64_t fi_len = flow_i.getFrameLength() * mt;
                uint8_t hop_i = flows_id_hop[i].second;
                for (int j = 0; j < c.p5_traffic_offsets[fi_id][hop_i].size(); ++j) {
                    uint64_t fi_mid = (c.p5_traffic_offsets[fi_id][hop_i][j] + fi_len / 2) % fi_period;
                    for (int k = i + 1; k < flows_id_hop.size(); ++k) {
                        uint32_t fj_id = flows_id_hop[k].first;
                        auto const &flow_j = flows[fj_id].get();
                        if (flow_j.getPriorityCodePoint() != schedplus::P5) continue;
                        isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                                   [&fj_id](uint32_t flowId) {
                                                       return fj_id == flowId;
                                                   });
                        if (!isCachedFlow) continue;
                        uint64_t fj_period = flow_j.getPeriod();
                        uint64_t fj_len = flow_j.getFrameLength() * mt;
                        uint8_t hop_j = flows_id_hop[k].second;
                        uint64_t dist = (fi_len + fj_len) / 2 + schedplus::IFG_TIME;
                        for (int l = 0; l < c.p5_traffic_offsets[fj_id][hop_j].size(); ++l) {
                            uint64_t fj_mid = (c.p5_traffic_offsets[fj_id][hop_j][l] + fj_len / 2) % fi_period;
                            int d = fi_mid - fj_mid;
                            if (std::abs(d) <= dist) {
                                oss << std::endl << "dist = " << dist << ", d = " << d << std::endl;
                                oss << "===P5 cached flows collision check failed===" << std::endl;
                                oss << "flow_i:    " << std::left << std::setw(10) << flow_i.getId()
                                    << "flow_j:    " << std::left << std::setw(10) << flow_j.getId() << std::endl;
                                oss << "fi_mid:    " << std::left << std::setw(10) << fi_mid
                                    << "fj_mid:    " << std::left << std::setw(10) << fj_mid << std::endl;
                                oss << "fi_period: " << std::left << std::setw(10) << fi_period
                                    << "fj_period: " << std::left << std::setw(10) << fj_period << std::endl;
                                oss << "fi_hop:    " << std::left << std::setw(10) << std::to_string(hop_i)
                                    << "fj_hop:    " << std::left << std::setw(10) << std::to_string(hop_j)
                                    << std::endl;
                                oss << "============================================" << std::endl;
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

public:
    Wait(vector<Node *> nodes,
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

    ~Wait() = default;

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

        /* Set send offset of each hop */
        setEachHopStartTime(p, c);

        if (!checkDDLorE2E(c, max_ddl, max_e2e))
            return false;

        c.uncached_flows.insert(c.uncached_flows.end(), flowGroupPcp[schedplus::P6].begin(),
                                flowGroupPcp[schedplus::P6].end());
        setLinkHyperperiod(p.selected_route_idx, c.link_hyperperiod);
        for (auto const &flow_id: flowGroupPcp[schedplus::P5]) {
            bool couldCache = true;
            auto const &flow = flows[flow_id];
            auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow_id]];
            uint64_t src_hyp = c.link_hyperperiod[route.getLinks()[0].get().getId()];
            for (int i = 1; i < route.getLinks().size(); ++i) {
                auto const &link = route.getLinks()[i];
                if (!(c.link_hyperperiod[link.get().getId()] >= src_hyp)
                    && (c.link_hyperperiod[link.get().getId()] % src_hyp != 0)) {
                    couldCache = false;
                    break;
                }
            }
            if (couldCache)
                c.cached_flows.push_back(flow_id);
            else
                c.uncached_flows.push_back(flow_id);
        }

        std::ostringstream oss("");
        for (auto const &flow_id: c.uncached_flows) {
            auto const &flow = flows[flow_id];
            for (int i = 0; i < flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                c.link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }
        if (!checkCollision(oss, c))
            return false;

        for (auto const &flow_id: c.cached_flows) {
            auto const &flow = flows[flow_id].get();
            for (int i = 0; i < flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks().size(); ++i) {
                uint32_t link_id = flow.getRoutes()[p.selected_route_idx[flow_id]].getLinks()[i].get().getId();
                c.link_flows[link_id].emplace_back(std::pair(flow_id, i));
            }
        }

        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            if (links[link_id].get().getSrcNode()->getNodeType() == SWITCH) {
                for (auto const &[flow_id, hop]: flows_id_hop) {
                    if (!c.link_gcl_size.contains(link_id)) {
                        c.link_gcl_size[link_id] = c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
                    } else {
                        c.link_gcl_size[link_id] += c.link_hyperperiod[link_id] / flows[flow_id].get().getPeriod();
                    }
                }
            }
        }
        c.total_cache = 0;
        if (!checkQueueCache(p, c, oss))
            return false;

        if (!checkP5CollisionCachedWithUncached(c, oss))
            return false;

        if (!checkP5CollisionCached(c, oss))
            return false;


        if (!checkP5E2E(p, c))
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
        c.ddl = (double) max_ddl;
        c.e2e = (double) max_e2e;
        c.total_transmit = (double) *std::max_element(p.offsets.begin(), p.offsets.end())
                           - (double) *std::min_element(p.offsets.begin(), p.offsets.end());
        return true;
    }

    double get_shrink_scale (int n_generation, const std::function<double(void)> &rnd01) {
        double scale = (n_generation <= 5 ? 1.0 : 1.0 / sqrt(n_generation - 5 + 1));
        if (rnd01() < 0.4)
            scale *= scale;
        else if (rnd01() < 0.1)
            scale = 1.0;
        return scale;
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
            bool in_range = true;
            int srcTransDelay =
                    flow.get().getFrameLength() * ((EndSystem *) flow.get().getSrc())->getPort().getMacrotick();
            int64_t up_bound = flow.get().getPeriod() - srcTransDelay;
            int64_t route;
            if (flow.get().getRoutes().size() > 1) {
                do {
                    route = X_base.selected_route_idx[i] + mu * (rnd01() - rnd01());
                    in_range = route >= 0 && route < flow.get().getRoutes().size();
                } while (!in_range);
            } else      route = 0;
            X_new.selected_route_idx[i] = (uint64_t) route;
            if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                int64_t tmp = flow.get().getDeliveryGuarantees()[0].getLowerVal() - flow.get().getRoutes()[route].getE2E();
                up_bound = std::min(up_bound, tmp);
            }
            do {
                double offset = X_base.offsets[i] / pow(10, 9);
                double tmp = mu * (rnd01() - rnd01());
                offset = offset + tmp;
                offset *= pow(10, 9);
                in_range = in_range && (offset >= 0) && (offset < up_bound);
                if (in_range) {
                    X_new.offsets[i] = (uint64_t) offset;
                }
            } while (!in_range);
        }
        return X_new;
    }

    TTFlows crossover(
            const TTFlows &X1,
            const TTFlows &X2,
            const std::function<double(void)> &rnd01) {
        TTFlows X_new;
        X_new = X1;
//        double r_offset_1 = rnd01();
//        double r_offset_2 = rnd01();
//        int offset_p1, offset_p2;
//        while (r_offset_1 == r_offset_2) {
//            r_offset_2 = rnd01();
//        }
//        if (r_offset_1 > r_offset_2) {
//            offset_p1 = X1.offsets.size() * r_offset_2;
//            offset_p2 = X1.offsets.size() * r_offset_1;
//        } else  {
//            offset_p1 = X1.offsets.size() * r_offset_1;
//            offset_p2 = X1.offsets.size() * r_offset_2;
//        }
//        double r_route_1 = rnd01();
//        double r_route_2 = rnd01();
//        int route_p1, route_p2;
//        while (r_route_1 == r_route_1) {
//            r_route_1 = rnd01();
//        }
//        if (r_route_1 > r_route_2) {
//            route_p1 = X1.offsets.size() * r_route_2;
//            route_p2 = X1.offsets.size() * r_route_1;
//        } else  {
//            route_p1 = X1.offsets.size() * r_route_1;
//            route_p2 = X1.offsets.size() * r_route_2;
//        }

//        for (int i = 0; i < X1.offsets.size(); ++i) {
//            if (i >= offset_p1 && i < offset_p2)
//                X_new.offsets[i] = X1.offsets[i];
//            else
//                X_new.offsets[i] = X2.offsets[i];
//            if (i >= route_p1 && i < route_p2)
//                X_new.selected_route_idx[i] = X1.selected_route_idx[i];
//            else
//                X_new.selected_route_idx[i] = X2.selected_route_idx[i];
//        }
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
                X.middle_costs.total_cache,
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
        std::string result = RESULT_REPORT;
        result.append("/solution_report_wait_schedule.txt");
        std::ofstream out_file(result);
        out_file << std::setw(3) << "id"
                 //                    << std::setw(20) << X.middle_costs.e2e
                 << std::setw(20) << "variance"
                 << std::setw(20) << "total_cache"
                 << std::setw(20) << "total_transmit" << std::endl;
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
            spdlog::get("console")->info("==============Solution {}==============", i);
            saveGCL(X.genes, X.middle_costs);
            spdlog::get("console")->info("=======================================");
            std::string route_file = OUT_LOCATION_WAIT;
            route_file.append("/" + std::to_string(i) + "_SmallRouting.xml");
            save_route(X.genes, X.middle_costs, route_file);

            std::string gcl_file = OUT_LOCATION_WAIT;
            gcl_file.append("/" + std::to_string(i));
            saveGCL(gcl_file, X.middle_costs);

            std::string route_file_name = std::to_string(i) + "_SmallRouting.xml";
            std::string gcl_file_name = std::to_string(i) + "_SmallGCL.xml";
            std::string ini_file = OUT_LOCATION_WAIT;
            ini_file.append("/" + std::to_string(i) + "_SmallTopology.ini");
            saveIni(route_file_name, gcl_file_name, ini_file, ned_file, i);

            std::string event_file = OUT_LOCATION_WAIT;
            event_file.append("/" + std::to_string(i) + "_event.txt");
            saveEvent(X.genes, X.middle_costs, event_file);

            out_file << std::setw(3) << i
                     //                    << std::setw(20) << X.middle_costs.e2e
                     << std::setw(20) << std::setprecision(8) << X.middle_costs.variance
                     << std::setw(20) << (uint64_t) X.middle_costs.total_cache
                     << std::setw(20) << (uint64_t) X.middle_costs.total_transmit << std::endl;
            for (auto const &[link_id, gcl_merge_count]: X.middle_costs.link_gcl_merge_count) {
                if (gcl_merge_count > 0) {
                    out_file << "link[" << link_id << "] merge " << gcl_merge_count << " times" << std::endl;
                }
            }
        }
    }

    void saveEvent(const TTFlows &p, MyMiddleCost &c, const std::string &event_file) {
        events.clear();
//        spdlog::get("console")->set_level(spdlog::level::info);
        for (auto &flow: flows) {
            uint32_t flow_id = flow.get().getId();
            auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow_id]].getLinks();
            bool isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                            [&flow_id](uint32_t flowId) {
                                                return flow_id == flowId;
                                            });
            for (int i = 0; i < route.size(); ++i) {
                auto &link = route[i].get();
                uint64_t time_interval = link.getSrcPort().getMacrotick() * flow.get().getFrameLength();
                if (isCachedFlow) {
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
                } else {
                    size_t send_times = c.link_hyperperiod[link.getId()] / flow.get().getPeriod();
                    for (int j = 0; j < send_times; ++j) {
                        uint64_t offset = c.traffic_offsets[flow_id][i] + j * flow.get().getPeriod();
                        schedplus::Event event1(offset, offset + time_interval, link.getSrcNode(), &flow.get(), i);
                        if (i == 0)
                            event1.setType(schedplus::TRANSMIT);
                        else
                            event1.setType(schedplus::FORWARD);
                        if (i + 1 == route.size()) {
                            uint64_t rcv_time = c.traffic_offsets[flow_id][i + 1] + j * flow.get().getPeriod();
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
            output << std::right << std::setw(5) << std::to_string(event.getFlow()->getId() + 1);
            output << std::right << std::setw(5) << std::to_string(event.getHop()) << std::endl;
        }
        output.close();

        std::ostringstream oss("");
        oss << std::setw(5) << "link"
            << std::setw(10) << "hyp"
            << std::setw(50) << "cached"
            << std::setw(50) << "uncached" << std::endl;

        std::ostringstream oss_cached("");
        std::ostringstream oss_uncached("");
        for (auto const &[link_id, flows_id_hop]: c.link_flows) {
            oss << std::setw(5) << link_id
                << std::setw(10) << c.link_hyperperiod[link_id];
            oss_cached << "{";
            oss_uncached << "{";
            for (auto const &[flow_id, hop]: flows_id_hop) {
                uint32_t fid = flow_id;
                bool isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                                [&fid](uint32_t flowId) {
                                                    return fid == flowId;
                                                });
                if (isCachedFlow) {
                    oss_cached << flow_id + 1 << ", ";
                } else {
                    oss_uncached << flow_id + 1 << ", ";
                }
                oss_cached.seekp(-2, std::ios_base::end);
                oss_uncached.seekp(-2, std::ios_base::end);
                oss_cached << "}";
                oss_uncached << "}";
            }
            oss << std::setw(50) << oss_cached.str()
                << std::setw(50) << oss_uncached.str() << std::endl;

            oss_cached.str("");
            oss_uncached.str("");
        }
        oss.str("");
        oss << std::endl << std::left << std::setw(5) << "flow"
            << std::left << std::setw(4) << "hop" << "route" << std::endl;
        for (auto const &flow: flows) {
            auto const &route = flow.get().getRoutes()[p.selected_route_idx[flow.get().getId()]].getLinks();
            oss << std::left << std::setw(5) << flow.get().getId() + 1;
            oss << std::left << std::setw(4) << route.size() - 1;
            for (auto const &link: route) {
                oss << link.get().getSrcNode()->getName() << ".link[" << link.get().getId() << "]("
                    << c.link_hyperperiod[link.get().getId()] << ") -> ";
            }
            oss.seekp(-3, std::ios_base::end);
            oss << std::endl;
        }
        oss.seekp(-1, std::ios_base::end);
        spdlog::get("console")->info("link_hyp: {}", oss.str());

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
                bool isCachedFlow = std::any_of(c.cached_flows.begin(), c.cached_flows.end(),
                                                [&flow_id](uint32_t flowId) {
                                                    return flow_id == flowId;
                                                });
                if (isCachedFlow) {
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
                } else {
                    uint64_t offset = c.traffic_offsets[flow_id][i];
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

        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "link gcl");
        for (auto link: links) {
            if (link.get().getSrcPort().getGateControlList().empty()) continue;
            link.get().sortGCL();
            link.get().mergeGCL();
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "link[{}].gcl.size = {}", link.get().getId(),
                                link.get().getSrcPort().getGateControlList().size());
        }
        SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "middle cost");
        for (auto const &[link_id, gcl_size]: c.link_gcl_size) {
            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), "link[{}].gcl.size = {}", link_id, gcl_size);
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
                            std::string mac = "255-0-00-00-00-" + std::to_string(flow_id + 1);
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
//        spdlog::set_level(spdlog::level::info);
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
                xflowId.append_child(pugi::node_pcdata).set_value(std::to_string(flow.get().getId() + 1).c_str());
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


#endif //SCHEDPLUS_WAIT_H
