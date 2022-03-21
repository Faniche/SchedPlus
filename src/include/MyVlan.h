//
// Created by faniche on 2022/3/15.
//

#ifndef SCHEDPLUS_MYVLAN_H
#define SCHEDPLUS_MYVLAN_H


/* Definition of Priority Code Point for vlan frame
 * https://en.wikipedia.org/wiki/IEEE_P802.1p
 **/
typedef int PRIORITY_CODE_POINT;
enum {
    BK,     // 0 (lowest)       Background
    BE,     // 1 (default)      Best effort
    EE,     // 2                Excellent effort
    CA,     // 3                Critical applications
    VI,     // 4                Video, < 100 ms latency and jitter
    VO,     // 5                Voice, < 10 ms latency and jitter
    IC,     // 6                Internetwork control
    NC      // 7 (highest)      Network control
};

#endif //SCHEDPLUS_MYVLAN_H
