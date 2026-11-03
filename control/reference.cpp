#include "control/reference.hpp"

namespace T1::wbc
{

    Reference sample_reference(const Trajectory& traj, double /*t*/)
    {
        // TODO(M1)
        return traj.front().ref;
    }

}  // namespace T1::wbc
