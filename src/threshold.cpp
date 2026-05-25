#include "threshold.h"

Bundle Bundle::operator* (const Bundle &b) const
{
    Bundle result = *this;
    result.x *= b.x;
    result.y *= b.y;
    result.angle *= b.angle;
    result.width *= b.width;
    result.length *= b.length;
    return result;
}

Bundle Bundle::operator* (float f) const
{
    Bundle result = *this;
    result.x *= f;
    result.y *= f;
    result.angle *= f;
    result.width *= f;
    result.length *= f;
    return result;
}

bool Bundle::operator< (const Bundle &bf)
{
    return bf.radius () < radius () && bf.get_angle () < angle
           && bf.get_width () < width && bf.get_length () < length;
}

Bundle Bundle::operator+ (const Bundle &b) const
{
    Bundle result = *this;
    result.x += b.get_x ();
    result.y += b.get_y ();
    result.angle += b.get_angle ();
    result.width += b.get_width ();
    result.length += b.get_length ();
    return result;
}

Bundle Bundle::operator- (const Bundle &b) const
{
    Bundle result = *this;
    result.x -= b.get_x ();
    result.y -= b.get_y ();
    result.angle -= b.get_angle ();
    result.width -= b.get_width ();
    result.length -= b.get_length ();
    return result;
}

Bundle linear_rectify (const Bundle &b)
{
    float x = linear_rectify (b.get_x ());
    float y = linear_rectify (b.get_y ());
    float angle = linear_rectify (b.get_angle ());
    float w = linear_rectify (b.get_width ());
    float l = linear_rectify (b.get_length ());
    return Bundle (x, y, angle, w, l);
}
