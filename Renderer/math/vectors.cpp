#include "vectors.h"

// #################################################################################################
// Vec2 Implementation
// #################################################################################################

Vec2::Vec2() : x(0), y(0) {}
Vec2::Vec2(float a) : x(a), y(a) {}
Vec2::Vec2(float _x, float _y) : x(_x), y(_y) {}
Vec2::Vec2(const Vec2i& v) : x(static_cast<float>(v.x)), y(static_cast<float>(v.y)) {}
Vec2::Vec2(const Vec2u& v) : x(static_cast<float>(v.x)), y(static_cast<float>(v.y)) {}
Vec2::Vec2(const Vec3& v) : x(v.x), y(v.y) {}

Vec2 Vec2::operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
Vec2 Vec2::operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
Vec2 Vec2::operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
Vec2 Vec2::operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }

Vec2 Vec2::operator*(const Vec2& v) const { return Vec2(x * v.x, y * v.y); }
Vec2 Vec2::operator/(const Vec2& v) const { return Vec2(x / v.x, y / v.y); }
Vec2 Vec2::operator*=(const Vec2& v) { x *= v.x; y *= v.y; return *this; }
Vec2 Vec2::operator/=(const Vec2& v) { x /= v.x; y /= v.y; return *this; }

Vec2 Vec2::operator+(const float& v) const { return Vec2(x + v, y + v); }
Vec2 Vec2::operator-(const float& v) const { return Vec2(x - v, y - v); }
Vec2 Vec2::operator+=(const float& v) { x += v; y += v; return *this; }
Vec2 Vec2::operator-=(const float& v) { x -= v; y -= v; return *this; }

Vec2 Vec2::operator*(const float& v) const { return Vec2(x * v, y * v); }
Vec2 Vec2::operator/(const float& v) const { return Vec2(x / v, y / v); }
Vec2 Vec2::operator*=(const float& v) { x *= v; y *= v; return *this; }
Vec2 Vec2::operator/=(const float& v) { x /= v; y /= v; return *this; }

Vec2 Vec2::operator-() const { return Vec2(-x, -y); }

bool Vec2::operator<=(const Vec2& other) const { return x <= other.x && y <= other.y; }
bool Vec2::operator>=(const Vec2& other) const { return x >= other.x && y >= other.y; }
bool Vec2::operator<(const Vec2& other) const { return x < other.x && y < other.y; }
bool Vec2::operator>(const Vec2& other) const { return x > other.x && y > other.y; }

bool Vec2::operator<=(const float& v) const { return x <= v && y <= v; }
bool Vec2::operator>=(const float& v) const { return x >= v && y >= v; }
bool Vec2::operator<(const float& v) const { return x < v && y < v; }
bool Vec2::operator>(const float& v) const { return x > v && y > v; }

float Vec2::lengthSq() const { return (x * x) + (y * y); }
float Vec2::length() const { return sqrtf(lengthSq()); }

Vec2 Vec2::normalize() const {
	float l = 1.0f / length();
	return Vec2(x * l, y * l);
}

float Vec2::dot(Vec2 v) const { return (x * v.x) + (y * v.y); }
float Vec2::cross(Vec2 v) const { return (x * v.y) - (y * v.x); }

// #################################################################################################
// Vec2i Implementation
// #################################################################################################

Vec2i::Vec2i() : x(0), y(0) {}
Vec2i::Vec2i(int a) : x(a), y(a) {}
Vec2i::Vec2i(int _x, int _y) : x(_x), y(_y) {}
Vec2i::Vec2i(const Vec2& v) : x(static_cast<int>(v.x)), y(static_cast<int>(v.y)) {}
Vec2i::Vec2i(const Vec2u& v) : x(static_cast<int>(v.x)), y(static_cast<int>(v.y)) {}
Vec2i::Vec2i(const Vec3& v) : x(static_cast<int>(v.x)), y(static_cast<int>(v.y)) {}

Vec2i Vec2i::operator+(const Vec2i& v) const { return Vec2i(x + v.x, y + v.y); }
Vec2i Vec2i::operator-(const Vec2i& v) const { return Vec2i(x - v.x, y - v.y); }
Vec2i Vec2i::operator+=(const Vec2i& v) { x += v.x; y += v.y; return *this; }
Vec2i Vec2i::operator-=(const Vec2i& v) { x -= v.x; y -= v.y; return *this; }

Vec2i Vec2i::operator*(const float& v) const { return Vec2i(static_cast<int>(x * v), static_cast<int>(y * v)); }
Vec2i Vec2i::operator/(const float& v) const { return Vec2i(static_cast<int>(x / v), static_cast<int>(y / v)); }
Vec2i Vec2i::operator*=(const float& v) { x = static_cast<int>(x * v); y = static_cast<int>(y * v); return *this; }
Vec2i Vec2i::operator/=(const float& v) { x = static_cast<int>(x / v); y = static_cast<int>(y / v); return *this; }

Vec2i Vec2i::operator-() const { return Vec2i(-x, -y); }

bool Vec2i::operator<=(const Vec2i& other) const { return x <= other.x && y <= other.y; }
bool Vec2i::operator>=(const Vec2i& other) const { return x >= other.x && y >= other.y; }
bool Vec2i::operator<(const Vec2i& other) const { return x < other.x && y < other.y; }
bool Vec2i::operator>(const Vec2i& other) const { return x > other.x && y > other.y; }

Vec2i Vec2i::Min(const Vec2i& other) { return Vec2i(x < other.x ? x : other.x, y < other.y ? y : other.y); }
Vec2i Vec2i::Max(const Vec2i& other) { return Vec2i(x > other.x ? x : other.x, y > other.y ? y : other.y); }

// #################################################################################################
// Vec2u Implementation
// #################################################################################################

Vec2u::Vec2u() : x(0), y(0) {}
Vec2u::Vec2u(unsigned int a) : x(a), y(a) {}
Vec2u::Vec2u(unsigned int _x, unsigned int _y) : x(_x), y(_y) {}
Vec2u::Vec2u(const Vec2& v) : x(static_cast<unsigned int>(v.x)), y(static_cast<unsigned int>(v.y)) {}
Vec2u::Vec2u(const Vec2i& v) : x(static_cast<unsigned int>(v.x)), y(static_cast<unsigned int>(v.y)) {}
Vec2u::Vec2u(const Vec3& v) : x(static_cast<unsigned int>(v.x)), y(static_cast<unsigned int>(v.y)) {}


Vec2u Vec2u::operator+(const Vec2u& v) const { return Vec2u(x + v.x, y + v.y); }
Vec2u Vec2u::operator-(const Vec2u& v) const { return Vec2u(x - v.x, y - v.y); }
Vec2u Vec2u::operator+=(const Vec2u& v) { x += v.x; y += v.y; return *this; }
Vec2u Vec2u::operator-=(const Vec2u& v) { x -= v.x; y -= v.y; return *this; }

Vec2u Vec2u::operator*(const float& v) const { return Vec2u(static_cast<unsigned int>(x * v), static_cast<unsigned int>(y * v)); }
Vec2u Vec2u::operator/(const float& v) const { return Vec2u(static_cast<unsigned int>(x / v), static_cast<unsigned int>(y / v)); }
Vec2u Vec2u::operator*=(const float& v) { x = static_cast<unsigned int>(x * v); y = static_cast<unsigned int>(y * v); return *this; }
Vec2u Vec2u::operator/=(const float& v) { x = static_cast<unsigned int>(x / v); y = static_cast<unsigned int>(y / v); return *this; }

bool Vec2u::operator<=(const Vec2u& other) const { return x <= other.x && y <= other.y; }
bool Vec2u::operator>=(const Vec2u& other) const { return x >= other.x && y >= other.y; }
bool Vec2u::operator<(const Vec2u& other) const { return x < other.x && y < other.y; }
bool Vec2u::operator>(const Vec2u& other) const { return x > other.x && y > other.y; }

Vec2u Vec2u::Min(const Vec2u& other) { return Vec2u(x < other.x ? x : other.x, y < other.y ? y : other.y); }
Vec2u Vec2u::Max(const Vec2u& other) { return Vec2u(x > other.x ? x : other.x, y > other.y ? y : other.y); }


// #################################################################################################
// Vec3 Implementation
// #################################################################################################

Vec3::Vec3() : x(0), y(0), z(0), w(1.0f) {}
Vec3::Vec3(float a) : x(a), y(a), z(a), w(1.0f) {}
Vec3::Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z), w(1.0f) {}
Vec3::Vec3(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}

Vec3::Vec3(const Vec2& v) : x(v.x), y(v.y), z(0), w(1.0f) {}
Vec3::Vec3(const Vec2i& v) : x(static_cast<float>(v.x)), y(static_cast<float>(v.y)), z(0), w(1.0f) {}
Vec3::Vec3(const Vec2u& v) : x(static_cast<float>(v.x)), y(static_cast<float>(v.y)), z(0), w(1.0f) {}

Vec3 Vec3::operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
Vec3 Vec3::operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
Vec3 Vec3::operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
Vec3 Vec3::operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }

Vec3 Vec3::operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
Vec3 Vec3::operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }

Vec3 Vec3::operator*(const float& v) const { return Vec3(x * v, y * v, z * v, w); }
Vec3 Vec3::operator/(const float& v) const { return Vec3(x / v, y / v, z / v, w / v); }
Vec3 Vec3::operator*=(const float& v) { x *= v; y *= v; z *= v; return *this; }
Vec3 Vec3::operator/=(const float& v) { x /= v; y /= v; z /= v; return *this; }

Vec3 Vec3::operator-() const { return Vec3(-x, -y, -z, w); }

Vec3 Vec3::perspectiveDivide() const { return Vec3(x / w, y / w, z / w, 1.0f / w); }
float Vec3::lengthSq() const { return (x * x) + (y * y) + (z * z); }
float Vec3::length() const { return sqrtf(lengthSq()); }

Vec3 Vec3::normalize() const {
	float l = 1.0f / length();
	return Vec3(x * l, y * l, z * l);
}

float Vec3::dot(Vec3 v) const { return (x * v.x) + (y * v.y) + (z * v.z); }
Vec3 Vec3::cross(Vec3 v) const { return Vec3((y * v.z) - (z * v.y), (z * v.x) - (x * v.z), (x * v.y) - (y * v.x)); }

// #################################################################################################
// Global functions
// #################################################################################################

float Dot(const Vec3 v1, const Vec3 v2) {
	return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

Vec3 Cross(const Vec3& v1, const Vec3& v2) {
	return Vec3((v1.y * v2.z) - (v1.z * v2.y), (v1.z * v2.x) - (v1.x * v2.z), (v1.x * v2.y) - (v1.y * v2.x));
}

Vec3 Max(const Vec3& a, const Vec3& b) {
	return Vec3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
}

Vec3 Min(const Vec3& a, const Vec3& b) {
	return Vec3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
}

// #################################################################################################
// Ostream operators
// #################################################################################################

std::ostream& operator<<(std::ostream& os, const Vec2& v) {
	os << "Vec2(" << v.x << ", " << v.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Vec2i& v) {
	os << "Vec2i(" << v.x << ", " << v.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Vec2u& v) {
	os << "Vec2u(" << v.x << ", " << v.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Vec3& v) {
	os << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
	return os;
}