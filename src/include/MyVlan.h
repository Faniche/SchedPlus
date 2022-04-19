//
// Created by faniche on 2022/3/15.
//

#ifndef SCHEDPLUS_MYVLAN_H
#define SCHEDPLUS_MYVLAN_H


/**
 * @brief Definition of Priority Code Point for vlan frame https://en.wikipedia.org/wiki/IEEE_P802.1p
 *
 */
typedef int PRIORITY_CODE_POINT;
enum {
    P0,     // 0 (lowest)       Background
    P1,     // 1 (default)      Best effort
    P2,     // 2                Excellent effort
    P3,     // 3                Critical applications
    P4,     // 4                Video, < 100 ms latency and jitter
    P5,     // 5                Voice, < 10 ms latency and jitter
    P6,     // 6                Internetwork control
    P7      // 7 (highest)      Network control
};

#endif //SCHEDPLUS_MYVLAN_H
