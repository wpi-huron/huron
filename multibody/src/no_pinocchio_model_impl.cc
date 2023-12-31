#include "huron/multibody/pinocchio_model.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

template <bool has_pinocchio>
PinocchioModel<has_pinocchio>::PinocchioModel(const std::string& path_to_file)
    : ModelBase() {
  throw NotImplementedException("Pinocchio not available!");
}

}  // namespace internal
}  // namespace multibody
}  // namespace huron
