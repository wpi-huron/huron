#pragma once

#include "model_impl_types.h"
#include "model_impl_interface.h"
#include "pinocchio_model_impl.h"

namespace huron {
namespace multibody {
namespace internal {

class ModelImplFactory final {
  friend class multibody::Model;
 public:
  ModelImplFactory() = delete;
  ModelImplFactory(const ModelImplFactory&) = delete;
  ModelImplFactory& operator=(const ModelImplFactory&) = delete;
  ~ModelImplFactory() = default;
 private:
  static std::unique_ptr<internal::ModelImplInterface>
  Create(ModelImplType type) {
    switch (type) {
      case ModelImplType::kPinocchio:
        return std::make_unique<internal::PinocchioModelImpl>();
      default:
        throw std::runtime_error("ModelImplType not implemented.");
    }
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace huron
