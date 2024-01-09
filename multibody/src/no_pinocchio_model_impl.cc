#include "huron/multibody/pinocchio_model_impl.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

struct PinocchioModelImpl::Impl {
  int dummy;
};

PinocchioModelImpl::PinocchioModelImpl()
    : ModelImplInterface() {
  throw NotImplementedException("Pinocchio not available!");
}
PinocchioModelImpl::~PinocchioModelImpl() = default;

}  // namespace internal
}  // namespace multibody
}  // namespace huron
