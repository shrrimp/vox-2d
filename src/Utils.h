#pragma once
#include <SFML/Graphics.hpp>
#include <math.h>

namespace Utils {

    // rotate by +a, so d/da rotVec(v, a) == rotVec(v, a).perpendicular() and omega is
    // exactly d(ang)/dt. pass (c, -s) to rotate by -a.
    inline sf::Vector2f rotVec(const sf::Vector2f &v, const float &a) {
        float c = cosf(a);
        float s = sinf(a);
        return sf::Vector2f(v.x * c - v.y * s, v.x * s + v.y * c);
    }

    inline sf::Vector2f rotVec(const sf::Vector2f &v, const float c, const float s) {
        return sf::Vector2f(v.x * c - v.y * s, v.x * s + v.y * c);
    }

    inline sf::Vector2f rotVec(const sf::Vector2f &v, const sf::Vector2f &cs) {
        return sf::Vector2f(v.x * cs.x - v.y * cs.y, v.x * cs.y + v.y * cs.x);
    }
    
    // x y : normal --- z : distance
    inline sf::Vector3f sdRoundedSquare(float px, float py, float half, float r) {
        const float sx = std::copysign(1.f, px);   // d|p|/dp
        const float sy = std::copysign(1.f, py);

        const float qx = std::fabs(px) - half + r;
        const float qy = std::fabs(py) - half + r;

        const float mx = qx > 0.f ? qx : 0.f;
        const float my = qy > 0.f ? qy : 0.f;
        const float l2 = mx*mx + my*my;
        const float l  = std::sqrt(l2);

        const float inside = std::fmin(std::fmax(qx, qy), 0.f);
        sf::Vector3f h;
        h.z = inside + l - r;

        if (l2 > 0.f) {                      // edge or corner: gradient = normalize(m)
            const float inv = 1.f / l;
            h.x = sx * mx * inv;
            h.y = sy * my * inv;
        } else {                             // deep interior: nearest axis
            const bool xdom = qx > qy;
            h.x = xdom ? sx : 0.f;
            h.y = xdom ? 0.f : sy;
        }

        return h;
    }

    // returns the distance to the center of a rounded square -> being over PX_SIZE/2 is being outside of the square
    inline sf::Vector3f PixelSDF(sf::Vector2f p, sf::Vector2f s, sf::Vector2f cs, float size) {
        sf::Vector2f lp = rotVec(p - s, {cs.x, -cs.y});
        
        return sdRoundedSquare(lp.x, lp.y, size / 2.f, size / 4.f);
    }
}