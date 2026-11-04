#include "cobot/studio/profile_catalog.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <system_error>

namespace cobot::studio {

std::vector<BasePair> discover_base_pairs(const fs::path& config_dir) {
  static const std::array<const char*, 6> pinned{
      "ideal", "realistic", "pessimistic", "premium_fr3", "mid_kr1205", "budget_xarm7"};
  const auto rank = [](const std::string& name) {
    return static_cast<std::size_t>(std::distance(
        pinned.begin(), std::find(pinned.begin(), pinned.end(), name)));  // 6 = not pinned
  };
  std::vector<BasePair> out;
  std::error_code ec;
  for (fs::directory_iterator it(config_dir / "imperfections", ec), end; !ec && it != end;
       it.increment(ec)) {
    const fs::path& p = it->path();
    if (p.extension() != ".yaml" || !it->is_regular_file(ec)) {
      continue;
    }
    const std::string name = p.stem().string();
    const fs::path own = config_dir / (name + ".yaml");
    out.push_back(
        BasePair{name, fs::is_regular_file(own, ec) ? own : config_dir / "default.yaml", p});
  }
  std::sort(out.begin(), out.end(), [&rank](const BasePair& a, const BasePair& b) {
    const std::size_t ra = rank(a.name);
    const std::size_t rb = rank(b.name);
    return ra != rb ? ra < rb : a.name < b.name;
  });
  return out;
}

}  // namespace cobot::studio
