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
    
    inline float wrapPi(float a) {
        a = fmodf(a + M_PIf, 2.f * M_PIf);
        if (a < 0.f) a += 2.f * M_PIf;
        return a - M_PIf;
    }
}