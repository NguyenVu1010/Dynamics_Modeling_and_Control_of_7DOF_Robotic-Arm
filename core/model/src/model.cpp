#include "cobot/model/model.hpp"

#include <cstddef>
#include <string>
#include <utility>

#include "cobot/common/dof.hpp"

namespace cobot::model {

template <int N>
Result<Model<N>> make_model(const UrdfChain& chain, const Vec3& gravity_b) {
  constexpr std::size_t kN = static_cast<std::size_t>(N);
  if (chain.joints.size() != kN) {
    return Result<Model<N>>::error("expected " + std::to_string(N) + " revolute joints, got " +
                                   std::to_string(chain.joints.size()));
  }
  if (chain.links.size() != kN) {
    return Result<Model<N>>::error("expected " + std::to_string(N) + " links, got " +
                                   std::to_string(chain.links.size()));
  }
  Model<N> model;
  for (std::size_t k = 0; k < kN; ++k) {
    model.joints[k] = chain.joints[k];
    model.links[k] = chain.links[k];
  }
  model.T_last_tip = chain.T_last_tip;
  model.gravity_b = gravity_b;
  return Result<Model<N>>::ok(std::move(model));
}

#define COBOT_MODEL_INSTANTIATE(N) \
  template Result<Model<N>> make_model<N>(const UrdfChain& chain, const Vec3& gravity_b);
COBOT_INSTANTIATE_DOF(COBOT_MODEL_INSTANTIATE)
#undef COBOT_MODEL_INSTANTIATE

}  // namespace cobot::model
