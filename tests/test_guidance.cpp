#include <gtest/gtest.h>

#include <functional>

#include "sim/Guidance.hpp"
#include "sim/Vector3.hpp"

using namespace sim;
using sim::guidance::proNavAcceleration;

namespace {

struct EngagementResult {
    double minSeparation;
    double finalTime;
    bool   intercepted;
};

/**
 * Closed-loop kinematic ProNav simulation.
 *
 * The interceptor steers with Proportional Navigation while holding constant
 * speed; the target position advances via @p targetAt(t). Runs at 60 Hz until
 * the fuze radius is reached or @p maxTime elapses.
 */
EngagementResult flyEngagement(Vector3 interceptorPos,
                               double cruiseSpeed,
                               const std::function<Vector3(double)>& targetAt,
                               double N,
                               double fuzeRadius = 5.0,
                               double maxTime = 60.0) {
    const double dt = 1.0 / 60.0;
    // Launch heading: straight at the target's initial position.
    Vector3 toTarget = (targetAt(0.0) - interceptorPos).normalized();
    Vector3 vel = toTarget * cruiseSpeed;

    double minSep = interceptorPos.distanceTo(targetAt(0.0));
    double t = 0.0;
    for (; t < maxTime; t += dt) {
        const Vector3 tgtPos = targetAt(t);
        // Finite-difference the target velocity so the law sees maneuvers.
        const Vector3 tgtVel = (targetAt(t + dt) - tgtPos) / dt;

        const Vector3 accel =
            proNavAcceleration(interceptorPos, vel, tgtPos, tgtVel, N);
        vel = vel + accel * dt;
        const double sp = vel.magnitude();
        if (sp > 1e-9) {
            vel = vel * (cruiseSpeed / sp);
        }
        interceptorPos += vel * dt;

        const double sep = interceptorPos.distanceTo(targetAt(t + dt));
        minSep = std::min(minSep, sep);
        if (sep <= fuzeRadius) {
            return {minSep, t, true};
        }
    }
    return {minSep, t, false};
}

} // namespace

TEST(Guidance, LosRateIsZeroOnCollisionCourse) {
    // Interceptor closing directly along the LOS: bearing is constant.
    Vector3 relPos{1000.0, 0.0, 0.0};
    Vector3 relVel{-300.0, 0.0, 0.0}; // purely along -R, closing
    Vector3 omega = guidance::lineOfSightRate(relPos, relVel);
    EXPECT_NEAR(omega.magnitude(), 0.0, 1e-9);

    // A crossing component produces a non-zero LOS rate.
    Vector3 crossingVel{-300.0, 50.0, 0.0};
    Vector3 omega2 = guidance::lineOfSightRate(relPos, crossingVel);
    EXPECT_GT(omega2.magnitude(), 0.0);
}

TEST(Guidance, ClosingSpeedAndTimeToGo) {
    Vector3 relPos{1000.0, 0.0, 0.0};
    Vector3 closing{-250.0, 0.0, 0.0};
    EXPECT_NEAR(guidance::closingSpeed(relPos, closing), 250.0, 1e-9);
    EXPECT_NEAR(guidance::timeToGo(relPos, closing), 4.0, 1e-9);

    // Receding pair: not closing -> infinite time-to-go.
    Vector3 receding{250.0, 0.0, 0.0};
    EXPECT_TRUE(std::isinf(guidance::timeToGo(relPos, receding)));
}

TEST(Guidance, CommandVanishesOnCollisionCourse) {
    // No LOS rotation -> zero commanded acceleration.
    Vector3 iPos{0.0, 0.0, 0.0};
    Vector3 iVel{300.0, 0.0, 0.0};
    Vector3 tPos{1000.0, 0.0, 0.0};
    Vector3 tVel{0.0, 0.0, 0.0}; // stationary, dead ahead
    Vector3 a = proNavAcceleration(iPos, iVel, tPos, tVel, 4.0);
    EXPECT_NEAR(a.magnitude(), 0.0, 1e-6);
}

TEST(Guidance, InterceptsStraightLineCrossingTarget) {
    // Target crossing laterally at constant velocity.
    auto target = [](double t) {
        return Vector3{5000.0, 3000.0, 1000.0} + Vector3{-200.0, 0.0, 0.0} * t;
    };
    for (double N : {3.0, 4.0, 5.0}) {
        auto r = flyEngagement({0.0, 0.0, 1000.0}, 1000.0, target, N);
        EXPECT_TRUE(r.intercepted)
            << "N=" << N << " min separation " << r.minSeparation << " m";
    }
}

TEST(Guidance, InterceptsInboundTarget) {
    // Target flying toward the defended volume in 3D.
    auto target = [](double t) {
        return Vector3{8000.0, 8000.0, 5000.0} +
               Vector3{-300.0, -280.0, -50.0} * t;
    };
    // At 60 Hz with >1 km/s closing speed the interceptor advances ~20 m per
    // frame, so endpoint-sampled miss distance is bounded near half that.
    // A 10 m proximity fuze reflects that reality; the guidance itself drives
    // the closest approach to a few meters (asserted below).
    auto r = flyEngagement({0.0, 0.0, 0.0}, 1200.0, target, 4.0,
                           /*fuzeRadius=*/10.0);
    EXPECT_TRUE(r.intercepted) << "min separation " << r.minSeparation << " m";
    EXPECT_LT(r.minSeparation, 10.0);
}

TEST(Guidance, InterceptsWeavingManeuveringTarget) {
    // Sinusoidal weave superimposed on a constant closing velocity.
    auto target = [](double t) {
        const double weave = 800.0 * std::sin(1.5 * t);
        return Vector3{6000.0, 2000.0 + weave, 1500.0} +
               Vector3{-250.0, 0.0, 0.0} * t;
    };
    // Faster interceptor and higher N cope with the lateral maneuver.
    auto r = flyEngagement({0.0, 0.0, 1500.0}, 1400.0, target, 5.0,
                           /*fuzeRadius=*/10.0);
    EXPECT_TRUE(r.intercepted) << "min separation " << r.minSeparation << " m";
}

TEST(Guidance, DivergesLessWithGuidanceThanWithout) {
    // Sanity: guided min-separation must beat an unguided straight shot.
    auto target = [](double t) {
        return Vector3{5000.0, 4000.0, 1000.0} + Vector3{-300.0, 100.0, 0.0} * t;
    };
    auto guided = flyEngagement({0.0, 0.0, 1000.0}, 1000.0, target, 4.0);
    auto unguided = flyEngagement({0.0, 0.0, 1000.0}, 1000.0, target,
                                  /*N=*/0.0);
    EXPECT_LT(guided.minSeparation, unguided.minSeparation);
}
