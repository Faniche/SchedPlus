//
// Created by faniche on 22-6-22.
//


#include "SaveSolution.h"

#include <utility>

SaveSolution::SaveSolution(uint64_t hyperPeriod, const vector<schedplus::Event> &events) :hyperPeriod(hyperPeriod), events(events) {

}

void SaveSolution::save_route(const TTFlows &p, MyMiddleCost &c,
                              const std::string &route_file_location,
                              vector<Node *> &nodes,
                              vector<Node *> &swList,
                              vector<std::reference_wrapper<DirectedLink>> &links) {
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
    xdoc.save_file(route_file_location.c_str());
}

void SaveSolution::saveGCL(const std::string &gcl_file_location,
                           vector<std::reference_wrapper<Flow>> flows,
                           map<node_idx, Node *> nodeMap,
                           vector<Node *> &swList,
                           vector<std::reference_wrapper<DirectedLink>> &links) {
    std::string switch_gcl_file = gcl_file_location + "SmallGCL.xml";
    saveSwPortSchedule(switch_gcl_file, swList, links);
    saveEsSchedule(gcl_file_location, std::move(flows), nodeMap, links);
}

void SaveSolution::saveSwPortSchedule(const std::string &sched_file_location,
                                      vector<Node *> &swList,
                                      vector<std::reference_wrapper<DirectedLink>> &links) {
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

void SaveSolution::saveEsSchedule(const std::string &sched_file_location,
                                  const vector<std::reference_wrapper<Flow>>& flows,
                                  map<node_idx, Node *> nodeMap,
                                  vector<std::reference_wrapper<DirectedLink>> &links) {
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

void SaveSolution::saveGCL(const TTFlows &p, MyMiddleCost &c,
                           const vector<std::reference_wrapper<Flow>>& flows,
                           vector<std::reference_wrapper<DirectedLink>> &links) {
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

void SaveSolution::saveEvent(const TTFlows &p, MyMiddleCost &c,
                             const vector<std::reference_wrapper<Flow>>& flows,
                             const std::string& event_file) {
    events.clear();
    std::ostringstream oss;
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
                for (auto const &[j, offset]: c.p5_traffic_offsets[flow_id][i + 1]) {
                    schedplus::Event event(offset, offset + time_interval, link.getDestNode(), &flow.get(), i);
                    event.setType(schedplus::RECEIVE);
                    events.push_back(event);
                }
            } else if (flow.get().getPriorityCodePoint() == schedplus::P6) {
                size_t send_times = c.link_hyperperiod[link.getId()] / flow.get().getPeriod();
                for (int j = 0; j < send_times; ++j) {
                    uint64_t offset = c.p6_traffic_offsets[flow_id][i] + j * flow.get().getPeriod();
                    schedplus::Event event(offset, offset + time_interval, link.getSrcNode(), &flow.get(), i);
                    if (i == 0)
                        event.setType(schedplus::TRANSMIT);
                    else if (i == route.size())
                        event.setType(schedplus::RECEIVE);
                    else
                        event.setType(schedplus::FORWARD);
                    events.push_back(event);
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

void SaveSolution::saveIni(const std::string &route_file,
                           const std::string &gcl_file,
                           const std::string &ini_file,
                           const std::string &ned_file,
                           vector<Node *> &swList,
                           vector<Node *> &esList,
                           const vector<std::reference_wrapper<Flow>>& flows,
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
        node_idx id = es->getId();
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
                   << std::to_string(solution_id) << es->getName() << R"(.xml"))" << std::endl;
        }
    }
}


