#pragma once
#include "mathdefines.h"

class Color
{
public:
	union
	{
		struct { float r, g, b; };
		float rgb[3];
	};

	Color();
	Color(float a);
	Color(float _r, float _g, float _b);
	Color(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a);

	float Lum() const;
	void ToRGB(unsigned char& cr, unsigned char& cg, unsigned char& cb) const;

	Color operator+(const Color& colour) const;
	Color operator-(const Color& colour) const;
	Color operator+=(const Color& colour);
	Color operator-=(const Color& colour);

	Color operator*(const Color& colour) const;
	Color operator/(const Color& colour) const;
	Color operator*=(const Color& colour);
	Color operator/=(const Color& colour);

	Color operator*(const float v) const;
	Color operator/(const float v) const;
	Color operator*=(const float v);
	Color operator/=(const float v);
};