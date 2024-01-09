#include "huron/multibody/joint_common.h"

namespace huron {
namespace multibody {

std::ostream& operator<<(std::ostream &os, const JointDescription &jd) {
    return (os << "ID: " << jd.id()
               << "\nName: " << jd.name()
               << "\nParent Frame ID: " << jd.parent_frame_id()
               << "\nChild Frame ID: " << jd.child_frame_id()
               << "\nNum Positions: " << jd.num_positions()
               << "\nNum Velocities: " << jd.num_velocities()
               << "\nType: " << (size_t) jd.type()
               << "\nMin Position: " << jd.min_position()
               << "\nMax Position: " << jd.max_position()
               << "\nMin Velocity: " << jd.min_velocity()
               << "\nMax Velocity: " << jd.max_velocity()
               << "\nMin Acceleration: " << jd.min_acceleration()
               << "\nMax Acceleration: " << jd.max_acceleration()
               << "\nMin Torque: " << jd.min_torque()
               << "\nMax Torque: " << jd.max_torque()
               << "\nFriction: " << jd.friction()
               << "\nDamping: " << jd.damping()
               << std::endl);
}

}  // namespace multibody
}  // namespace huron
