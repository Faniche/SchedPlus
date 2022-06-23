//
// Created by faniche on 22-6-22.
//


#include "SaveSolution.h"

SaveSolution::SaveSolution(const vector<Node *> &nodes, const vector<Node *> &esList, const vector<Node *> &swList,
                           const map<node_idx, Node *> &nodeMap,
                           const vector<std::reference_wrapper<DirectedLink>> &links,
                           const vector<std::reference_wrapper<Flow>> &flows, const std::string &outputLocation,
                           uint64_t hyperPeriod, const vector<schedplus::Event> &events,
                           const map<uint32_t, uint64_t> &linkHyperperiod) : nodes(nodes), esList(esList),
                                                                             swList(swList), nodeMap(nodeMap),
                                                                             links(links), flows(flows),
                                                                             output_location(outputLocation),
                                                                             hyperPeriod(hyperPeriod), events(events),
                                                                             link_hyperperiod(linkHyperperiod) {}

void SaveSolution::save_route(const std::string &route_file_location,
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

void SaveSolution::saveGCL(const std::string &gcl_file_location, const vector<uint64_t> &selected_route_idx) {
    std::string switch_gcl_file = gcl_file_location + "SmallGCL.xml";
    saveSwPortSchedule(switch_gcl_file, selected_route_idx);
    saveEsSchedule(gcl_file_location, selected_route_idx);
}

void SaveSolution::saveSwPortSchedule(const std::string &sched_file_location,
                                      const vector<uint64_t> &selected_route_idx) {
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

void SaveSolution::saveEsSchedule(const std::string &sched_file_location, const vector<uint64_t> &selected_route_idx) {
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

void SaveSolution::saveGCL(const vector<uint64_t> &offsets,
                           const vector<uint64_t> &selected_route_idx) {
    for (auto &flow: flows) {
        /* Calculate GCL */
        uint32_t flow_id = flow.get().getId();
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

void SaveSolution::saveEvent(const vector<uint64_t> &offsets,
                             const vector<uint64_t> &selected_route_idx,
                             const std::string &event_file_location) {
    events.clear();
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
                        trans_delay =
                                flow.get().getFrameLength() * route.getLinks()[j].get().getSrcPort().getMacrotick();
                        accumulatedDelay += proc_delay;
                        uint32_t sendTimes =
                                link_hyperperiod[route.getLinks()[j].get().getId()] / flow.get().getPeriod();
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
                            SPDLOG_LOGGER_DEBUG(spdlog::get("console"), event.toString(oss));
                            events.push_back(event);
                            if (j == hop - 1) {
                                event.setStart(end_time + prop_delay);
                                event.setEnd(end_time + prop_delay + trans_delay);
                                event.setType(schedplus::RECEIVE);
                                event.setNode(flow.get().getDest());
                                event.setHop(hop);
                                events.push_back(event);
                                SPDLOG_LOGGER_DEBUG(spdlog::get("console"), event.toString(oss));
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

void SaveSolution::saveIni(const std::string &route_file,
                           const std::string &gcl_file,
                           const std::string &ini_file,
                           const std::string &ned_file,
                           uint64_t solution_id) {
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