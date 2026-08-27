#pragma once
#include "PixelBoxy.h"
#include "Joint.h"
#include "BodyStorage.h"
#include "ContactCache.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

struct Collision {
    BodyId o1; // body A
    BodyId o2; // body B
    sf::Vector2u p1;

    float mu;
    sf::Vector2f n;
    float delta;
    sf::Vector2f p;
    float ln;
    float lt;
};

inline ContactKey keyOf(const Collision &c) {
      return {
          (uint64_t(c.o1.slot) << 40) | (uint64_t(c.o1.gen & 0xFF) << 32)
              | (uint64_t(c.p1.x) << 16) | uint64_t(c.p1.y),
          (c.o2.slot << 8) | (c.o2.gen & 0xFF)
      };
  }

struct Engine {
    // fixed rate for constant dt.
    static constexpr float timestep = 1.f / 120.f;
    // clamp so a long frame doesn't queue up an unbounded number of steps.
    static constexpr float max_frame_time = 0.25f;

    BodyStorage bodies;
    JointStorage joints;

    std::vector<BodyId> ommit_mouse;

    bool ommitted(const BodyId &id) const {
        return std::find(ommit_mouse.begin(), ommit_mouse.end(), id) != ommit_mouse.end();
    }

    ContactCache contacts;

    PixelBody floor_body = PixelBody({}, {0, 0}, sf::Vector2f(0.f, 0.f));
    
    float floor_height = 1000.f;
    float accumulator = 0.f;

    sf::RenderWindow window;

    Engine();

    void init();
    void handleEvents();
    void update(float dt);
    void step(float h);
    void draw();
    void loop();

    // joints
    JointId join(BodyId a, sf::Vector2u pa, BodyId b, sf::Vector2u pb, Joint::JointType type);
    void disconnect(JointId id);
};
