#ifndef SIM_VECTOR3_HPP
#define SIM_VECTOR3_HPP

#include <cmath>
#include <ostream>

namespace sim {

/**
 * @brief Lightweight 3D vector in a right-handed Cartesian frame.
 *
 * All coordinates are expressed in meters within the simulation's world
 * frame (X = East, Y = North, Z = Up). The type is a trivially copyable
 * value type with constexpr-friendly arithmetic so it can live in
 * fixed-size telemetry structs and hot update loops without overhead.
 */
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3() = default;
    constexpr Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // --- Component-wise arithmetic -----------------------------------------
    constexpr Vector3 operator+(const Vector3& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }
    constexpr Vector3 operator-(const Vector3& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }
    constexpr Vector3 operator-() const { return {-x, -y, -z}; }

    constexpr Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vector3& operator+=(const Vector3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z; return *this;
    }
    Vector3& operator-=(const Vector3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this;
    }
    Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    // --- Geometric products ------------------------------------------------
    constexpr double dot(const Vector3& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    constexpr Vector3 cross(const Vector3& rhs) const {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    // --- Magnitude ---------------------------------------------------------
    constexpr double magnitudeSquared() const { return dot(*this); }
    double magnitude() const { return std::sqrt(magnitudeSquared()); }

    /**
     * @brief Returns a unit-length copy of this vector.
     *
     * A near-zero vector (magnitude below @p epsilon) has no defined
     * direction and is returned unchanged as the zero vector, which avoids
     * producing NaNs in downstream guidance math.
     */
    Vector3 normalized(double epsilon = 1e-12) const {
        const double mag = magnitude();
        if (mag < epsilon) {
            return Vector3{};
        }
        return *this / mag;
    }

    double distanceTo(const Vector3& other) const {
        return (*this - other).magnitude();
    }
    constexpr double distanceSquaredTo(const Vector3& other) const {
        return (*this - other).magnitudeSquared();
    }
};

// Scalar-on-left multiplication: 2.0 * v
constexpr Vector3 operator*(double s, const Vector3& v) { return v * s; }

inline std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
    return os;
}

} // namespace sim

#endif // SIM_VECTOR3_HPP
