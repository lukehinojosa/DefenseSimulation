#ifndef SIM_GUIDANCE_HPP
#define SIM_GUIDANCE_HPP

#include <limits>

#include "sim/Vector3.hpp"

namespace sim {
namespace guidance {

/// Default navigation constant N. Effective ProNav range is typically 3–5.
constexpr double kDefaultNavConstant = 4.0;

/**
 * @brief Line-of-sight (LOS) rotation rate vector, Omega.
 *
 * Given the relative position R (target minus interceptor) and relative
 * velocity V_r (target minus interceptor), the LOS angular velocity is
 *
 *     Omega = (R x V_r) / (R . R).
 *
 * On a collision course the bearing to the target is constant, so Omega is
 * the zero vector — the signal ProNav drives to zero.
 */
inline Vector3 lineOfSightRate(const Vector3& relPos, const Vector3& relVel,
                               double epsilon = 1e-9) {
    const double r2 = relPos.magnitudeSquared();
    if (r2 < epsilon) {
        return Vector3{};
    }
    return relPos.cross(relVel) / r2;
}

/**
 * @brief Closing speed (range rate), positive when the range is decreasing.
 *
 * V_c = -(R . V_r) / |R|.
 */
inline double closingSpeed(const Vector3& relPos, const Vector3& relVel,
                           double epsilon = 1e-9) {
    const double r = relPos.magnitude();
    if (r < epsilon) {
        return 0.0;
    }
    return -relPos.dot(relVel) / r;
}

/**
 * @brief First-order time-to-go estimate: range divided by closing speed.
 * @return +infinity when the pair is not closing (V_c <= 0).
 */
inline double timeToGo(const Vector3& relPos, const Vector3& relVel,
                       double epsilon = 1e-9) {
    const double vc = closingSpeed(relPos, relVel, epsilon);
    if (vc <= epsilon) {
        return std::numeric_limits<double>::infinity();
    }
    return relPos.magnitude() / vc;
}

/**
 * @brief True Proportional Navigation commanded acceleration.
 *
 * Implements the program's guidance law
 *
 *     a_c = N * (V_r x Omega),   Omega = (R x V_r) / (R . R),
 *
 * where R = target_pos - interceptor_pos and V_r = target_vel -
 * interceptor_vel. Expanding with the vector triple product,
 *
 *     a_c = (N / |R|^2) * [ R |V_r|^2 - V_r (R . V_r) ],
 *
 * whose LOS component pulls the interceptor toward the target while the
 * lateral component nulls the LOS rate. The command vanishes when Omega is
 * zero (already on a collision course), which is the defining ProNav
 * property. N in [3, 5] gives well-damped convergence.
 */
inline Vector3 proNavAcceleration(const Vector3& interceptorPos,
                                  const Vector3& interceptorVel,
                                  const Vector3& targetPos,
                                  const Vector3& targetVel,
                                  double navConstant = kDefaultNavConstant) {
    const Vector3 relPos = targetPos - interceptorPos; // LOS, interceptor->target
    const Vector3 relVel = targetVel - interceptorVel; // relative velocity
    const Vector3 omega  = lineOfSightRate(relPos, relVel);
    return relVel.cross(omega) * navConstant;
}

} // namespace guidance
} // namespace sim

#endif // SIM_GUIDANCE_HPP
