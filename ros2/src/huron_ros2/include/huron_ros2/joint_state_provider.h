#pragma once

#include "huron/control_interfaces/state_provider.h"
#include "huron_node.h"

namespace huron {
namespace ros2 {

class JointStateProvider : public huron::StateProvider {
 public:
  JointStateProvider(size_t id_q, size_t nq, size_t id_v, size_t nv,
                     std::shared_ptr<HuronNode> node);

  void RequestStateUpdate() override;
  void GetNewState(Eigen::Ref<Eigen::MatrixXd> new_state) const override;

 private:
  std::shared_ptr<HuronNode> node_;
  size_t nq_;
  size_t nv_;
  size_t id_q_;
  size_t id_v_;
};

}  // namespace ros2
}  // namespace huron
