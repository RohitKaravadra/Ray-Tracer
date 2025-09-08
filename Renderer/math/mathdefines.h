#pragma once

#define NOMINMAX

#define SQ(x)    ((x)*(x))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

template<typename T>
T clamp01(const T& x) { return (x < 0) ? 0 : (x > 1) ? 1 : x; }
template<typename T>
T clamp(const T& x, const T& low, const T& high) { return (x < low) ? low : (x > high) ? high : x; }
template<typename T>
T wrap(const T& x, const T& low, const T& high) { return (x < low) ? high - (low - x) : (x > high) ? low + (x - high) : x; }
template<typename T>
T lerp(const T& a, const T& b, float t)
{
	t = clamp01(t);
	return a * (1.0f - t) + b * t;
}
