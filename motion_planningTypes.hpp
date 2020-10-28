#ifndef motion_planning_TYPES_HPP
#define motion_planning_TYPES_HPP

#include <vector>

namespace motion_planning {
    struct ArmProfile
    {
        std::vector<std::vector<double> > position;
    };
}

#endif

