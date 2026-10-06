#pragma once

#include <cmath>

namespace uav_controller {

// Allow small clock synchronization errors, but still reject missing, stale
// and substantially future-dated data. Do not replace source stamps with now.
inline bool freshTimestamp(double stamp, double now, double timeout,
                           double future_tolerance) {
  const double age = now - stamp;
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0.0 &&
         age >= -future_tolerance && age <= timeout;
}

}  // namespace uav_controller
