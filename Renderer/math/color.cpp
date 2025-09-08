#include "color.h"

Color::Color() { r = g = b = 0.0f; }
Color::Color(float a) { r = g = b = a; }
Color::Color(float _r, float _g, float _b) { r = _r; g = _g; b = _b; }
Color::Color(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a)
{
	r = (float)_r * (1.0f / 255.0f);
	g = (float)_g * (1.0f / 255.0f);
	b = (float)_b * (1.0f / 255.0f);
}

float Color::Lum()const { return 0.2126f * r + 0.7152f * g + 0.0722f * b; }

void Color::ToRGB(unsigned char& cr, unsigned char& cg, unsigned char& cb) const
{
	cr = (unsigned char)(clamp(r, 0.0f, 1.0f) * 255.0f);
	cg = (unsigned char)(clamp(g, 0.0f, 1.0f) * 255.0f);
	cb = (unsigned char)(clamp(b, 0.0f, 1.0f) * 255.0f);
}

Color Color::operator*(const float v) const { return Color(r * v, g * v, b * v); }
Color Color::operator/(const float v) const { return Color(r / v, g / v, b / v); }
Color Color::operator*=(const float v) { r *= v; g *= v; b *= v; return *this; }
Color Color::operator/=(const float v) { r /= v; g /= v; b /= v;	return *this; }

Color Color::operator+(const Color& colour) const
{
	Color col;
	col.r = r + colour.r;
	col.g = g + colour.g;
	col.b = b + colour.b;
	return col;
}

Color Color::operator-(const Color& colour) const
{
	Color col;
	col.r = r - colour.r;
	col.g = g - colour.g;
	col.b = b - colour.b;
	return col;
}

Color Color::operator+=(const Color& colour)
{
	r += colour.r;
	g += colour.g;
	b += colour.b;
	return *this;
}

Color Color::operator-=(const Color& colour)
{
	r -= colour.r;
	g -= colour.g;
	b -= colour.b;
	return *this;
}

Color Color::operator*(const Color& colour) const
{
	Color col;
	col.r = r * colour.r;
	col.g = g * colour.g;
	col.b = b * colour.b;
	return col;
}

Color Color::operator/(const Color& colour) const
{
	Color col;
	col.r = r / colour.r;
	col.g = g / colour.g;
	col.b = b / colour.b;
	return col;
}

Color Color::operator*=(const Color& colour)
{
	r *= colour.r;
	g *= colour.g;
	b *= colour.b;
	return *this;
}

Color Color::operator/=(const Color& colour)
{
	r /= colour.r;
	g /= colour.g;
	b /= colour.b;
	return *this;
}
