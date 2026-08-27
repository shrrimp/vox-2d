#pragma once
#include "Settings.h"
#include <SFML/Graphics.hpp>
#include <math.h>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "Handle.h"

struct CornerPx {
    sf::Vector2u p; // pixel coordinate
    char mask; // 0000 -> [left right top down] residency
};

struct Material {
    std::string name; // the XPM symbolic name, so files can refer to it
    sf::Color color = sf::Color::White;
    float density = 1.f;
    float friction = 0.9f;
    float restitution = 0.f;
    float strength = 1.f;
};

struct PixelBody {
    BodyId self;
    std::vector<BodyId> f_bodies;

    sf::Vector2f pos;
    sf::Vector2f vel = sf::Vector2f(0.f, 0.f);
    sf::Vector2f forces = sf::Vector2f(0.f, 0.f);
    
    float ang = 0.f;//-M_PI / 5.f;
    float omega = 0.f;
    float a_forces = 0.f;

    // mutable angle cache
    mutable float cc = cosf(ang);
    mutable float cs = sinf(ang);
    mutable float ca = ang;
    
    float inv_mass = 0.f;
    float inv_inertia = 0.f;

    bool fixed = false;

    sf::Vector2f centroid;
    float radius = 0.f; // bounding circle, everything outside can't collide

    sf::Color color = sf::Color::White;
    sf::Color outline_color = sf::Color::Red;

    sf::Vector2u size;
    std::vector<std::string> data;

    std::vector<std::vector<sf::Vector2f>> offsets;

    std::vector<CornerPx> corners; // pixels that make a corner -> they are the source of collision detection, they will test against any other pixel in range

    std::vector<Material> materials; // empty -> every pixel uses the defaults
    std::array<uint8_t, 128> mat_of{}; // pixel char -> index into materials, 0xFF = none

    PixelBody(const std::vector<std::string> &d, const sf::Vector2u &s, const sf::Vector2f &p, bool isFixed);
    PixelBody(const std::vector<std::string> &d, const sf::Vector2u &s, const sf::Vector2f &p);
    PixelBody(const std::string &xpm_file, const sf::Vector2f &p, bool fixed, float angle);
    PixelBody(const std::string &xpm_file, const sf::Vector2f &p, bool fixed);
    PixelBody(const std::string &xpm_file, const sf::Vector2f &p);

    // shared constructor body: expects pos, size and data-shaped d to be ready
    void build(const std::vector<std::string> &d);

    const Material &material(const size_t x, const size_t y) const;
    float density(const char c) const;

    float imass() const;
    float iinertia() const;

    // cos and sin of the angle - cached - output : x = cos, y = sin
    sf::Vector2f getCS() const;

    // conversion utils
    sf::Vector2f worldVertex(const sf::Vector2u &coords) const;
    sf::Vector2f worldPxCenter(const sf::Vector2u &coords) const;
    sf::Vector2f worldPxCenter(const size_t x, const size_t y) const;
    sf::Vector2i pxAt(const sf::Vector2f &p) const;
    sf::Vector2f fpxAt(const sf::Vector2f &p) const;

    bool filter(const BodyId &b) const; // check if a body is filtered

    char getPx(const sf::Vector2u &p) const;
    char getPx(const size_t x, const size_t y) const;

    char getMask(const int x, const int y) const;

    void draw(sf::RenderWindow &w) const;
};