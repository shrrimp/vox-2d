#pragma once
#include "BodyId.h"
#include "PixelBoxy.h"
#include <SFML/Graphics.hpp>
#include <vector>

// Joint data
struct Joint {
    BodyId a, b;
    sf::Vector2u pa, pb;    // anchor pixel in each body

    sf::Vector2f lambda{};  // accumulated linear impulse
    float lambda_ang = 0.f; // accumulated angular impulse
    float rest_angle = 0.f;

    float beta  = 0.2f;     // positional stiffness
    float break_impulse = INFINITY;
    bool  collide = false;  // if false, register a BodyStorage::filter
};

// Joint solving data, regenerated each substep
struct JointSolve {
    Joint *j;
    PixelBody *A, *B;
    sf::Vector2f ra, rb;
    float k00, k01, k11, inv_det; // 2x2 inverse, prebuilt
    sf::Vector2f bias;
};