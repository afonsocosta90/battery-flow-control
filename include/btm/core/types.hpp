#pragma once

#include <cstdint>
#include <type_traits>
#include <ostream>

namespace btm::core {

// Forward declarations for strong types
struct Temperature;
struct MassFlowRate;
struct Duration;
struct Current;
struct Power;
struct Energy;

// -----------------------------------------------------------------------------
// Temperature (kelvin internally, but we work in °C for human values)
// -----------------------------------------------------------------------------
struct Temperature {
    constexpr Temperature() : value(0.0) {}               // default to 0 for arrays/containers
    explicit constexpr Temperature(double v) : value(v) {}

    double value{0.0}; // °C (by convention in this project)

    constexpr Temperature& operator+=(Temperature rhs) { value += rhs.value; return *this; }
    constexpr Temperature& operator-=(Temperature rhs) { value -= rhs.value; return *this; }
    constexpr Temperature& operator*=(double s)        { value *= s; return *this; }
    constexpr Temperature& operator/=(double s)        { value /= s; return *this; }

    friend constexpr Temperature operator+(Temperature a, Temperature b) { return Temperature{a.value + b.value}; }
    friend constexpr Temperature operator-(Temperature a, Temperature b) { return Temperature{a.value - b.value}; }
    friend constexpr Temperature operator*(Temperature a, double s)      { return Temperature{a.value * s}; }
    friend constexpr Temperature operator*(double s, Temperature a)      { return Temperature{s * a.value}; }
    friend constexpr Temperature operator/(Temperature a, double s)      { return Temperature{a.value / s}; }

    friend constexpr bool operator==(Temperature a, Temperature b) { return a.value == b.value; }
    friend constexpr bool operator!=(Temperature a, Temperature b) { return !(a == b); }
    friend constexpr bool operator< (Temperature a, Temperature b) { return a.value <  b.value; }
    friend constexpr bool operator<=(Temperature a, Temperature b) { return a.value <= b.value; }
    friend constexpr bool operator> (Temperature a, Temperature b) { return a.value >  b.value; }
    friend constexpr bool operator>=(Temperature a, Temperature b) { return a.value >= b.value; }

    friend std::ostream& operator<<(std::ostream& os, Temperature t) {
        return os << t.value << " °C";
    }
};

// -----------------------------------------------------------------------------
// MassFlowRate (kg/s)
// -----------------------------------------------------------------------------
struct MassFlowRate {
    constexpr MassFlowRate() : value(0.0) {}
    explicit constexpr MassFlowRate(double v) : value(v) {}

    double value{0.0};

    constexpr MassFlowRate& operator+=(MassFlowRate rhs) { value += rhs.value; return *this; }
    constexpr MassFlowRate& operator-=(MassFlowRate rhs) { value -= rhs.value; return *this; }
    constexpr MassFlowRate& operator*=(double s)         { value *= s; return *this; }
    constexpr MassFlowRate& operator/=(double s)         { value /= s; return *this; }

    friend constexpr MassFlowRate operator+(MassFlowRate a, MassFlowRate b) { return MassFlowRate{a.value + b.value}; }
    friend constexpr MassFlowRate operator-(MassFlowRate a, MassFlowRate b) { return MassFlowRate{a.value - b.value}; }
    friend constexpr MassFlowRate operator*(MassFlowRate a, double s)       { return MassFlowRate{a.value * s}; }
    friend constexpr MassFlowRate operator*(double s, MassFlowRate a)       { return MassFlowRate{s * a.value}; }
    friend constexpr MassFlowRate operator/(MassFlowRate a, double s)       { return MassFlowRate{a.value / s}; }

    friend constexpr bool operator==(MassFlowRate a, MassFlowRate b) { return a.value == b.value; }
    friend constexpr bool operator!=(MassFlowRate a, MassFlowRate b) { return !(a == b); }
    friend constexpr bool operator< (MassFlowRate a, MassFlowRate b) { return a.value <  b.value; }

    friend std::ostream& operator<<(std::ostream& os, MassFlowRate m) {
        return os << m.value << " kg/s";
    }
};

// -----------------------------------------------------------------------------
// Duration (seconds)
// -----------------------------------------------------------------------------
struct Duration {
    constexpr Duration() : value(0.0) {}
    explicit constexpr Duration(double v) : value(v) {}

    double value{0.0};

    constexpr Duration& operator+=(Duration rhs) { value += rhs.value; return *this; }
    constexpr Duration& operator-=(Duration rhs) { value -= rhs.value; return *this; }
    constexpr Duration& operator*=(double s)     { value *= s; return *this; }
    constexpr Duration& operator/=(double s)     { value /= s; return *this; }

    friend constexpr Duration operator+(Duration a, Duration b) { return Duration{a.value + b.value}; }
    friend constexpr Duration operator-(Duration a, Duration b) { return Duration{a.value - b.value}; }
    friend constexpr Duration operator*(Duration a, double s)   { return Duration{a.value * s}; }
    friend constexpr Duration operator*(double s, Duration a)   { return Duration{s * a.value}; }
    friend constexpr Duration operator/(Duration a, double s)   { return Duration{a.value / s}; }

    friend constexpr bool operator==(Duration a, Duration b) { return a.value == b.value; }
    friend constexpr bool operator<(Duration a, Duration b)  { return a.value < b.value; }

    friend std::ostream& operator<<(std::ostream& os, Duration d) {
        return os << d.value << " s";
    }
};

// -----------------------------------------------------------------------------
// Current (amperes)
// -----------------------------------------------------------------------------
struct Current {
    constexpr Current() : value(0.0) {}
    explicit constexpr Current(double v) : value(v) {}

    double value{0.0};

    friend constexpr Current operator*(Current a, double s) { return Current{a.value * s}; }
    friend constexpr Current operator*(double s, Current a) { return Current{s * a.value}; }
    friend constexpr Current operator/(Current a, double s) { return Current{a.value / s}; }

    friend constexpr bool operator==(Current a, Current b) { return a.value == b.value; }
    friend constexpr bool operator<(Current a, Current b)  { return a.value < b.value; }

    friend std::ostream& operator<<(std::ostream& os, Current i) {
        return os << i.value << " A";
    }
};

// -----------------------------------------------------------------------------
// Power (watts)
// -----------------------------------------------------------------------------
struct Power {
    constexpr Power() : value(0.0) {}
    explicit constexpr Power(double v) : value(v) {}

    double value{0.0};

    friend constexpr Power operator+(Power a, Power b) { return Power{a.value + b.value}; }
    friend constexpr Power operator-(Power a, Power b) { return Power{a.value - b.value}; }
    friend constexpr Power operator*(Power a, double s) { return Power{a.value * s}; }
    friend constexpr Power operator*(double s, Power a) { return Power{s * a.value}; }
    friend constexpr Power operator/(Power a, double s) { return Power{a.value / s}; }

    friend constexpr bool operator==(Power a, Power b) { return a.value == b.value; }

    friend std::ostream& operator<<(std::ostream& os, Power p) {
        return os << p.value << " W";
    }
};

// -----------------------------------------------------------------------------
// Compile-time unit safety (static_asserts)
// These will fail to compile if someone tries to add incompatible quantities.
// -----------------------------------------------------------------------------
static_assert(!std::is_convertible_v<Temperature, MassFlowRate>, "Temperature must not convert to MassFlowRate");
static_assert(!std::is_convertible_v<MassFlowRate, Duration>,     "MassFlowRate must not convert to Duration");
static_assert(!std::is_convertible_v<Current, Temperature>,       "Current must not convert to Temperature");

// Prevent accidental mixing in arithmetic at compile time (selected examples)
template <typename T, typename U>
concept SamePhysicalType = std::is_same_v<T, U>;

} // namespace btm::core
