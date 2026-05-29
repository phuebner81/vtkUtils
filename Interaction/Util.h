#pragma once

#include <string>
#include <type_traits>
#include <cstdint>
#include <cmath>

#ifndef M_PI
#   define M_PI 3.14159265358979323846
#endif

namespace Util {

// ───────────── StringConverter ─────────────
struct StringConverter {
    template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    static std::string toStdstring(T v) { return std::to_string(v); }

    template <typename T>
    static std::string toString(T v) { return toStdstring(v); }
};

// ───────────── Angle base ─────────────
class Angle {
protected:
    float rad_{ 0.f };

public:
    constexpr Angle() = default;
    constexpr explicit Angle(float r) : rad_(r) {}

    static Angle fromDegrees(float deg) { return Angle(deg * float(M_PI) / 180.f); }
    static Angle fromRadians(float rad) { return Angle(rad); }

    float valueRadians() const { return rad_; }
    void setRadians(float r) { rad_ = r; }
};

// ───────────── Forward for Degree/Radian ─────────────
class Radian;
class Degree;

// ───────────── Degree ─────────────
class Degree : public Angle {
public:
    constexpr Degree() = default;
    constexpr explicit Degree(float d) : Angle(d * float(M_PI) / 180.f) {}
    explicit Degree(const Radian& r);  // implemented after Radian

    operator float() const { return rad_ * 180.f / float(M_PI); }
    operator Angle() const { return *this; }
};

// ───────────── Radian ─────────────
class Radian : public Angle {
public:
    explicit Radian(std::int64_t r) : Angle(static_cast<float>(r)) {}
    explicit Radian(float r = 0.f) : Angle(r) {}
    explicit Radian(double r) : Angle(static_cast<float>(r)) {}
    Radian(const Degree& d);  // implemented below

    Radian& operator=(float f) { rad_ = f; return *this; }
    Radian& operator=(double f) { rad_ = static_cast<float>(f); return *this; }
    Radian& operator=(const Radian& r) { rad_ = r.rad_; return *this; }
    Radian& operator=(const Degree& d) { rad_ = d.valueRadians(); return *this; }
};

// ───────────── Implement cross ctors ─────────────
inline Degree::Degree(const Radian& r) : Angle(r.valueRadians()) {}
inline Radian::Radian(const Degree& d) : Angle(d.valueRadians()) {}

} // namespace Util

