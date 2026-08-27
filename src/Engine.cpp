#include "Settings.h"
#include "Engine.h"
#include "Utils.h"
#include <math.h>
#include <SFML/System/Angle.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>

Engine::Engine() {
    this->init();
}

void Engine::init() {
    window = sf::RenderWindow(sf::VideoMode({800u, 600u}), "Physics");
    window.setFramerateLimit(60);
    contacts.map.reserve(2048);

    const std::vector<std::string> assets = {
        "assets/ball.xpm",
        "assets/box.xpm",
        "assets/bar.xpm",
        "assets/triangle.xpm",
        "assets/wheel.xpm",
    };
    
    if (false) { // Car and wall
        bodies.create("assets/bar.xpm", sf::Vector2f(50.f, floor_height - 200.f), true, 0.f);
    
        BodyId bar = bodies.create("assets/bar.xpm", sf::Vector2f(300.f, floor_height - 150.f), false, -M_PI_2);
        BodyId bar2 = bodies.create("assets/bar.xpm", sf::Vector2f(300.f, floor_height - 300.f));
        BodyId bar3 = bodies.create("assets/bar.xpm", sf::Vector2f(600.f, floor_height - 300.f));
        BodyId w1 = bodies.create("assets/ball.xpm", sf::Vector2f(100.f, floor_height - 150.f), false, M_PI_4);
        BodyId w2 = bodies.create("assets/ball.xpm", sf::Vector2f(500.f, floor_height - 150.f));
    
        ommit_mouse.push_back(w1);
        ommit_mouse.push_back(w2);
    
        join({ Joint::MOTOR, bar, w1, {2, 2}, {7, 7}, .target = 1.f, .max_force = 50000000.f });
        join({ Joint::MOTOR, bar, w2, {2, 47}, {7, 7}, .target = 1.f, .max_force = 50000000.f });
        join({ Joint::FIXED, bar, bar2, {2, 16}, {2, 47} });
        join({ Joint::FIXED, bar, bar3, {2, 34}, {2, 47} });
    }
    {
        // Linear bearing + Spring
        BodyId bar = bodies.create("assets/bar.xpm", sf::Vector2f(300.f, floor_height - 250.f));
        BodyId bar2 = bodies.create("assets/thin_bar.xpm", sf::Vector2f(300.f, floor_height - 300.f));
        BodyId bucket = bodies.create("assets/bucket.xpm", sf::Vector2f(300.f, floor_height - 600.f));

        join({ Joint::LIN_BEARING, bar, bar2, .pb = {1, 48}, .axis_pos = {2.5, 25.5}, .axis_angle = M_PI_2, .course_min = -200.f, .course_max = 200.f});
        join({ Joint::SPRING, bar, bar2, {1, 49}, {1, 48}, .hertz = 1.f, .damping = 0.2f, .rest_length = 200.f});
        join({ Joint::FIXED, bar2, bucket, {1, 1}, {15, 13}});
    }

    {
        // Angular spring
        BodyId bar = bodies.create("assets/bar.xpm", sf::Vector2f(800.f, floor_height - 250.f), true, M_PI_2);
        BodyId wheel = bodies.create("assets/wheel.xpm", sf::Vector2f(1000.f, floor_height - 250.f));

        join({ Joint::ANG_SPRING, bar, wheel, {2, 2}, {7, 7}, .hertz = 1.8f, .damping = .1f});
    }

    if (false) {
        // cube sea
        size_t amount = 200;
        size_t x = 0;
        size_t y = 0;
        while (x * y < amount) {
            for (x = 0; x < 20; x++) {
                bodies.create("assets/px.xpm", sf::Vector2f(float(x + 30) * PX_SIZE * 2.f, floor_height - y * PX_SIZE * 2.5f));
            }
            y++;
        }
    }
}

void Engine::loop() {
    sf::Clock clock;

    while (window.isOpen()) {
        float frame_time = clock.restart().asSeconds();
        if (frame_time > max_frame_time) frame_time = max_frame_time;

        handleEvents();
        // temporary to allow debug rendering outside of draw
        window.clear(sf::Color::Black);
        update(frame_time);

        accumulator += frame_time;
        while (accumulator >= timestep) {
            step(timestep);
            accumulator -= timestep;
        }

        draw();
    }
}

void Engine::handleEvents() {
    while (const std::optional event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
        else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
            window.setView(sf::View(sf::FloatRect({0.f, 0.f}, sf::Vector2f(resized->size))));
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
        {
            if (keyPressed->scancode == sf::Keyboard::Scancode::Escape)
                window.close();
        }
    }
}

void Engine::update(float dt) {
    // controls ?
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::Up)) floor_height -= 100.f * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scancode::Down)) floor_height += 100.f * dt;
    floor_body.pos.y = floor_height;

    static BodyId selected;
    static sf::Vector2f grab_local; // grab point in A's local frame, same space as offsets
    sf::Vector2i mpos = sf::Mouse::getPosition(window);
    sf::Vector2f mposf(float(mpos.x), float(mpos.y));

    if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
        if (selected.gen == 0 && bodies.liveSlots().size() > 0) {
            float mindist = INFINITY;

            for (auto &id : bodies.liveSlots()) {
                if (ommitted(bodies.bodyId(id))) continue;

                float dist = (bodies[id].pos - mposf).lengthSquared();
                if (dist < mindist) {
                    mindist = dist;
                    selected = bodies.bodyId(id);
                }
            }

            grab_local = {0.f, 0.f};
            PixelBody *body = bodies.get(selected);
            if (body != nullptr) {
                sf::Vector2f cs = body->getCS();
                sf::Vector2f local = Utils::rotVec(mposf - body->pos, {cs.x, -cs.y});
                // inverse of worldPxCenter: local/PX_SIZE + centroid lands in (x+.5, y+.5) space
                sf::Vector2f px = local / PX_SIZE + body->centroid;
                // grab off-centre only on a real pixel, so a click into empty space
                // cannot hand a far-away body a huge lever arm
                if (px.x >= 0.f && px.y >= 0.f
                 && body->getPx(sf::Vector2u(unsigned(px.x), unsigned(px.y))) != PX_EMPTY) {
                    grab_local = local;
                }
            }
        }

        if (selected.gen != 0) {
            PixelBody *body = bodies.get(selected);

            if (body == nullptr) selected = {0, 0};
            else {
                sf::Vector2f r = Utils::rotVec(grab_local, body->getCS());
                sf::Vector2f grab = body->pos + r;
                sf::Vector2f force = (mposf - grab) * 5.f * GRAVITY;

                body->forces += force;
                body->a_forces += r.cross(force); // off-centre pull also turns the body

                sf::Vertex v[2] = {
                    {.position = grab, .color = sf::Color::Red},
                    {.position = mposf, .color = sf::Color::Red}
                };

                window.draw(v, 2, sf::PrimitiveType::Lines);
            }
        }
    } else {
        selected = {0, 0};
    }

    if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) {
        sf::Vector2i mpos = sf::Mouse::getPosition(window);
        
        for (auto &bp : bodies.liveSlots()) {
            PixelBody &body = bodies[bp];

            if (ommitted(body.self)) continue;

            sf::Vector2f force_dir = (mposf - body.pos);
            body.forces += force_dir * 5.f * GRAVITY;

            sf::Vertex v[2] = {
                {.position = body.pos, .color = sf::Color::Green},
                {.position = mposf, .color = sf::Color::Green}
            };

            window.draw(v, 2, sf::PrimitiveType::Lines);
        }
    }


    for (auto &id : joints.liveSlots()) {
        const float piston_speed = 150.f; // world units per second
        Joint &j = joints[id];
        if (j.type == Joint::PISTON) {
            if (sf::Keyboard::isKeyPressed(j.key1)) {
                j.target = fminf(j.course_max, j.target + dt * piston_speed);
            }
            if (sf::Keyboard::isKeyPressed(j.key2)) {
                j.target = fmaxf(j.course_min, j.target - dt * piston_speed);
            }
        }
    }

    // for (PixelBody &b : bodies) {
    //     b.ang += dt;
    // }
    
}

void Engine::step(float h) {
    // cache stamp update
    contacts.stamp++;

    // FORCES & AIR FRICTION
    
    for (auto &bp : bodies.liveSlots()) {
        PixelBody &b = bodies[bp];
        if (b.fixed) { // reset fixed bodies
            b.vel = {0.f, 0.f};
            b.omega = 0.f;
            continue;
        }

        b.vel += h * (b.inv_mass * b.forces + sf::Vector2f(0.f, GRAVITY)); //apply forces and gravity;
        b.omega += h * b.inv_inertia * b.a_forces; // apply angular forces
        
        // reset forces
        b.forces = {0.f, 0.f};
        b.a_forces = 0.f;
    }
    
    // COLLISIONS
    // Naive O(n²) approach
    std::vector<Collision> collisions;

    size_t checks = 0;
    size_t radius_passes = 0;
    size_t corner_checks = 0;
    size_t rejections = 0;

    for (auto &aid : bodies.liveSlots()) {
        PixelBody &a = bodies[aid];

        // test floor
        if (a.pos.y + a.radius > floor_height) {
            for (auto &p : a.corners) {
                sf::Vector2f wpos = a.worldPxCenter(p.p);
                float fd = wpos.y + (.5f * PX_SIZE) - floor_height; // pixels as spheres
                if (fd > 0.f) {
                    // floor collision
                    collisions.push_back(Collision{
                        .o1 = bodies.bodyId(aid),
                        .o2 = {0, 0},
                        .p1 = p.p,
                        // .p2 = {0, 0},
                        .mu = a.material(p.p.x, p.p.y).friction,
                        .n = sf::Vector2f(0.f, 1.f), // vertical normal for the floor
                        .delta = fd,
                        .p = wpos + sf::Vector2f(0.f, .5f * (PX_SIZE - fd)),
                        .ln = 0,
                        .lt = 0
                    });
                }
            }
        } 

        // for (int j = i + 1; j < (int)bodies.size(); j++) {
        for (auto &bid : bodies.liveSlots()) {
            if (aid == bid) continue;
            PixelBody &b = bodies[bid];
            if (a.filter(b.self)) continue; // skip filtered bodies
            checks++;

            float collision_dist_squared = (a.radius + b.radius) * (a.radius + b.radius);
            float dist_squared = (b.pos - a.pos).lengthSquared(); // vectors go from A to B

            if (dist_squared < collision_dist_squared) { // collision POSSIBLE
                radius_passes++;
                // check every edge pixel against every other one
                for (const CornerPx &p1 : a.corners) {
                    corner_checks++;
                    sf::Vector2f px = a.worldPxCenter(p1.p); // center of the origin pixel // WORLD SPACE
                    // skip out of bounds
                    float pxbound = ((M_SQRT1_2f * PX_SIZE) + b.radius); // half diagonal of pixel + radius
                    pxbound *= pxbound;
                    if ((b.pos - px).lengthSquared() >= pxbound) continue;

                    sf::Vector2f fpxat = b.fpxAt(px); // B LOCAL SPACE
                    sf::Vector2i pxat = {(int)floorf(fpxat.x), (int)floorf(fpxat.y)}; 
                    sf::Vector2i dirSign{
                        1 - (2 * std::signbit(fpxat.x - (.5f + float(pxat.x)))),
                        1 - (2 * std::signbit(fpxat.y - (.5f + float(pxat.y)))),
                    }; // direction from pxat pixel center to the origin pixel center (center at floor (top left corner) + .5)
                    std::vector<Collision> pxCols;
                    float maxdelta = 0.f;
                    int maxid = -1;
                    for (int x = 0; x < 2; x++) {
                        for (int y = 0; y < 2; y++) {
                            sf::Vector2i other{
                                pxat.x + (dirSign.x * x),
                                pxat.y + (dirSign.y * y)
                            };
                            sf::Vector2u uother((unsigned int)other.x, (unsigned int)other.y);

                            if (b.getPx(uother) != PX_EMPTY) {
                                // collision
                                // calculate point of collision
                                sf::Vector2f opx = b.worldPxCenter(uother); // WORLD SPACE
                                float length = (opx - px).length();
                                float delta = PX_SIZE - length;
                                sf::Vector2f normal = (opx - px) / (length != 0.f ? length : 1e-6f);
                                sf::Vector2f cp = px + .5f * (opx - px);

                                // find outside directions for both bodies
                                // masks built live, no cache
                                // BODY A
                                char amask = a.getMask(p1.p.x, p1.p.y);
                                sf::Vector2f localAOut(
                                    float(amask >> 2 & 1) - float(amask >> 3 & 1), // right - left (if both then 0, else the direction of the outside)
                                    float(amask & 1) - float(amask >> 1 & 1)  // bottom - top (same)
                                ); // local space of body A; this vector cannot be 0 since the pixel is guaranteed to be an edge pixel
                                sf::Vector2f AOut = Utils::rotVec(localAOut, a.getCS());
                                if (localAOut.x == 0.f && localAOut.y == 0.f) AOut = normal; // take normal in ambiguous symetrical or lone pixel cases, to not cancel collision
                                
                                // BODY B
                                char bmask = b.getMask(other.x, other.y);
                                sf::Vector2f localBOut(
                                    float(bmask >> 2 & 1) - float(bmask >> 3 & 1), // right - left (if both then 0, else the direction of the outside)
                                    float(bmask & 1) - float(bmask >> 1 & 1)  // bottom - top (same)
                                ); // local space of body A; this vector cannot be 0 since the pixel is guaranteed to be an edge pixel
                                
                                // UNSAFE, rejecting collisions from 1px thin bars that should push towards the centroid
                                if (bmask == 0) {
                                    localBOut = sf::Vector2f(.5f + other.x, .5f + other.y) - b.centroid;
                                }
                                sf::Vector2f BOut = Utils::rotVec(localBOut, b.getCS());
                                if (localBOut.x == 0.f && localBOut.y == 0.f) BOut = -normal; // lone pixel case
                                
                                if (normal.dot(AOut) > 0.f && normal.dot(BOut) < 0.f) {
                                    if (delta > maxdelta) {
                                        maxdelta = delta;
                                        maxid = pxCols.size();
                                    }
                                    // one collision max per corner pixel, the one with highest delta
                                    pxCols.push_back(Collision{
                                        .o1 = bodies.bodyId(aid),
                                        .o2 = bodies.bodyId(bid),
                                        .p1 = p1.p,
                                        // .p2 = uother,
                                        .mu = a.material(p1.p.x, p1.p.y).friction * b.material(uother.x, uother.y).friction,
                                        .n = normal,
                                        .delta = delta,
                                        .p = cp,
                                        .ln = 0,
                                        .lt = 0
                                    });
                                } else {
                                    rejections++;
                                }
                            }
                        }
                    }

                    if (maxid != -1) collisions.push_back(pxCols[maxid]);
                }
            }
        }
        
    }

    // draw collisions
    for (auto &c : collisions) {
        sf::CircleShape point;
        point.setRadius(1);
        point.setOrigin({1.f, 1.f});
        point.setFillColor(sf::Color::Red);
        point.setPosition(c.p);

        window.draw(point);


        sf::Vertex v[2] = {
            {.position = c.p, .color = sf::Color::Blue},
            {.position = c.p + (.5f * PX_SIZE * c.n), .color = sf::Color::Blue}
        };

        window.draw(v, 2, sf::PrimitiveType::Lines);
    }

    
    sf::Font font("assets/fonts/PXFont.otf");
    sf::Text text(font);
    text.setString("collisions : " + std::to_string(collisions.size()));
    text.setCharacterSize(12);
    text.setPosition({10.f, 10.f});
    window.draw(text);
    // collision rejection display
    text.setString("rejections : " + std::to_string(rejections));
    text.setPosition({10.f, 26.f});
    window.draw(text);
    // passes / checks percentage display
    if (checks > 0) {
        text.setString("radius pass : " + std::to_string(100 * radius_passes / checks) + "%");
        text.setPosition({10.f, 42.f});
        window.draw(text);
    }
    // corner checks / collisions percentage display
    if (corner_checks > 0) {
        text.setString("corner pass : " + std::to_string(100 * collisions.size() / corner_checks) + "%");
        text.setPosition({10.f, 58.f});
        window.draw(text);
    }

    text.setString("bodies : " + std::to_string(bodies.liveSlots().size()));
    text.setPosition({10.f, 74.f});
    window.draw(text);

    text.setString("joints : " + std::to_string(joints.liveSlots().size()));
    text.setPosition({10.f, 90.f});
    window.draw(text);

    // build JointSolve array
    std::vector<JointSolve> sjoints;

    for (size_t id_i = joints.liveSlots().size(); id_i-- > 0; ) {
        size_t id = joints.liveSlots()[id_i];
        PixelBody *a = bodies.get(joints[id].a);
        PixelBody *b = bodies.get(joints[id].b);

        if (a == nullptr || b == nullptr) {
            joints.destroy(joints.jointId(id));
            continue;
        }

        Joint *j = &(joints[id]);
        sjoints.push_back(j->prepare(a, b, h));
    }
    

    for (Collision &c : collisions) {
        ContactState st = contacts.find(keyOf(c));
        c.ln = st.ln;
        c.lt = st.lt;

        PixelBody &s1 = bodies[c.o1.slot];
        PixelBody &s2 = (c.o2.gen == 0) ? floor_body : bodies[c.o2.slot];

        sf::Vector2f P = c.ln * c.n + c.lt * c.n.perpendicular();

        if (!s1.fixed) {
            s1.vel   -= s1.imass() * P;
            s1.omega -= s1.iinertia() * (c.p - s1.pos).cross(P);
        }

        if (!s2.fixed) {
            s2.vel   += s2.imass() * P;
            s2.omega += s2.iinertia() * (c.p - s2.pos).cross(P);
        }
    }

    for (JointSolve &s : sjoints) {
        if (s.has_anchor) {
            const sf::Vector2f P = s.j->lambda;
            s.A->vel   += s.A->imass()    * P;
            s.A->omega += s.A->iinertia() * s.ra.cross(P);
            s.B->vel   -= s.B->imass()    * P;
            s.B->omega -= s.B->iinertia() * s.rb.cross(P);
        }

        // warm start scalar rows
        for (int i = 0; i < s.row_count; i++) {
            const JointSolve::Row &r = s.rows[i];
            const float P = *r.lambda;
            s.A->vel   += s.A->imass()    * P * r.dir;
            s.A->omega += s.A->iinertia() * r.ja * P;
            s.B->vel   -= s.B->imass()    * P * r.dir;
            s.B->omega -= s.B->iinertia() * r.jb * P;
        }
    }
    
    // Solve collisions

    for (int N = 0; N < 8; N++) {
        for (Collision &c : collisions) {
            // lever arms
            bool flr = c.o2.gen == 0;
            PixelBody &s1 = bodies[c.o1.slot];
            // all values at 0 to make everything easy from now on
            // could test for bool flr but I'd rather compute 0s than write ifs
            PixelBody &s2 = flr ? floor_body : bodies[c.o2.slot];

            if (s1.fixed && flr) continue;
    
            sf::Vector2f r1 = c.p - s1.pos;
            sf::Vector2f r2 = c.p - s2.pos;
            
            sf::Vector2f vp1 = s1.vel + s1.omega * r1.perpendicular();
            sf::Vector2f vp2 = s2.vel + s2.omega * r2.perpendicular();
    
            sf::Vector2f vrel = vp2 - vp1;
            float vn = vrel.dot(c.n);
    
            float r1cn = r1.cross(c.n);
            float r2cn = r2.cross(c.n);
            float kn = s1.imass() + s2.imass() + s1.iinertia() * r1cn * r1cn + s2.iinertia() * r2cn * r2cn;
    
            float beta = SIM_BETA;
            float s = SIM_S; // accepted penetration
            float vtarget = SIM_MAX_BIAS > 0.f
                 ? fminf(SIM_MAX_BIAS, (beta * fmaxf(0.f, c.delta - s)) / h)
                 : (beta * fmaxf(0.f, c.delta - s)) / h;
    
            float lambda = (vtarget - vn) / kn;
    
            float lnn = fmaxf(0.f, c.ln + lambda);

            sf::Vector2f t = c.n.perpendicular();
            float vt = vrel.dot(t);
            float r1ct = r1.cross(t);
            float r2ct = r2.cross(t);
            float kt = s1.imass() + s2.imass() + s1.iinertia() * r1ct * r1ct + s2.iinertia() * r2ct * r2ct;

            float mu = c.mu;

            float lt = -vt / kt;
            float ltn = fmaxf(-mu * lnn, fminf(c.lt + lt, mu * lnn));

            float lapp = lnn - c.ln;
            c.ln = lnn;
    
            // apply forces
            if (!s1.fixed) {

                s1.vel   -= s1.imass() * lapp * c.n;
                s1.omega -= s1.iinertia() * r1cn * lapp;
            }
    
            if (!s2.fixed) {
                s2.vel   += s2.imass() * lapp * c.n;
                s2.omega += s2.iinertia() * r2cn * lapp;
            }

            float ltapp = ltn - c.lt;
            c.lt = ltn;
    
            // apply friction forces
            // no forces stored in the body's force fields? idk why
            if (!s1.fixed) {
                s1.vel   -= s1.imass() * ltapp * t;
                s1.omega -= s1.iinertia() * r1ct * ltapp;
            }
    
            if (!s2.fixed) {
                s2.vel   += s2.imass() * ltapp * t;
                s2.omega += s2.iinertia() * r2ct * ltapp;
            }
        }
        for (JointSolve &s : sjoints) {
            if (s.has_anchor) {
                sf::Vector2f vp_a = s.A->vel + s.A->omega * s.ra.perpendicular();
                sf::Vector2f vp_b = s.B->vel + s.B->omega * s.rb.perpendicular();
                sf::Vector2f cdot = vp_a - vp_b;
    
                sf::Vector2f rhs = -(cdot + s.bias);
    
                sf::Vector2f dl(
                    s.inv_det * ( s.k11 * rhs.x - s.k01 * rhs.y),
                    s.inv_det * (-s.k01 * rhs.x + s.k00 * rhs.y)
                );
    
                s.j->lambda += dl;
    
                s.A->vel   += s.A->imass()    * dl;
                s.A->omega += s.A->iinertia() * s.ra.cross(dl);
                s.B->vel   -= s.B->imass()    * dl;
                s.B->omega -= s.B->iinertia() * s.rb.cross(dl);
            }

            for (int i = 0; i < s.row_count; i++) {
                JointSolve::Row &r = s.rows[i];

                float cdot = (s.A->vel - s.B->vel).dot(r.dir)
                        + s.A->omega * r.ja - s.B->omega * r.jb;

                float dl  = -(cdot + r.bias + r.gamma * (*r.lambda)) * r.inv_k;
                float old = *r.lambda;
                float nl  = std::clamp(old + dl, r.lo, r.hi);
                dl = nl - old;
                *r.lambda = nl;

                s.A->vel   += s.A->imass()    * dl * r.dir;
                s.A->omega += s.A->iinertia() * r.ja * dl;
                s.B->vel   -= s.B->imass()    * dl * r.dir;
                s.B->omega -= s.B->iinertia() * r.jb * dl;
            }
        }
    }
    
    // integrate positions
    sf::Vector2u wsize = window.getSize();
    for (auto &bp : bodies.liveSlots()) {
        PixelBody &b = bodies[bp];
        if (b.fixed) continue;
        // if (s.pos.x - s.rad < 0.f && s.vel.x < 0.f) s.vel.x = fminf(s.vel.x, s.pos.x - s.rad) * -1.f;
        // if (s.pos.x + s.rad > wsize.x && s.vel.x > 0.f) s.vel.x = fmaxf(s.vel.x, s.pos.x + s.rad - wsize.x) * -1.f;
        b.pos += b.vel * h;
        b.ang += b.omega * h;
        // apply friction
        b.vel *= VEL_FRICTION;
        b.omega *= ROT_FRICTION;
    }

    for (const Collision &c : collisions) contacts.store(keyOf(c), c.ln, c.lt);
    if ((contacts.stamp & 127) == 0) contacts.sweep(128);
}

void Engine::draw() {
    //window.clear(sf::Color::Black);

    sf::Vertex floor1;
    floor1.color = sf::Color::Green;
    floor1.position = {0.f, floor_height};
    sf::Vertex floor2;
    floor2.color = sf::Color::Green;
    floor2.position = {float(window.getView().getSize().x), floor_height};

    sf::Vertex floor[2] = {
        floor1,
        floor2
    };

    window.draw(floor, 2, sf::PrimitiveType::Lines);

    for (const auto &bp : bodies.liveSlots()) {
        PixelBody &b = bodies[bp];
        b.draw(window);
    }

    drawJoints();

    // MOUSE COLLISION DETECTOR --- DISABLED
    // sf::Vector2i mpos = sf::Mouse::getPosition(window);

    // sf::CircleShape mouse;
    // mouse.setRadius(4);
    // mouse.setOrigin(sf::Vector2f(4.f, 4.f));
    // mouse.setPosition(sf::Vector2f(mpos.x, mpos.y));

    // sf::Vector2i pxat = bodies[0]->pxAt(mouse.getPosition());
    // if (pxat.x >= 0 && pxat.y >= 0 && bodies[0]->getPx(sf::Vector2u(pxat)) != '0') {
    //     mouse.setFillColor(sf::Color::Green);
    // } else {
    //     mouse.setFillColor(sf::Color::Red);
    // }

    // window.draw(mouse);

    window.display();
}


// joints
JointId Engine::join(JointSettings settings) {
    JointSettings s = settings;
    // validate bodies
    PixelBody *ba = bodies.get(settings.a);
    PixelBody *bb = bodies.get(settings.b);
    if (ba == nullptr || bb == nullptr || ba == bb) {
        return {};
    }
    // ignore pixel check on A for linear bearings
    bool skipA = settings.type == Joint::LIN_BEARING || settings.type == Joint::PISTON;
    if ((!skipA && ba->getPx(settings.pa) == PX_EMPTY) || bb->getPx(settings.pb) == PX_EMPTY) {
        return {};
    }
    
    // create filter
    bodies.filter(settings.a, settings.b, true);
    
    float rest_ang = ba->ang - bb->ang;
    s.rest_angle = rest_ang;

    s.axis_pos = (s.axis_pos - ba->centroid) * PX_SIZE; // convert axis position

    return joints.create(s);
}

void Engine::disconnect(JointId id) {
    Joint *j = joints.get(id);
    if (j == nullptr) return;

    // destroy filter
    bodies.filter(j->a, j->b, false);
    
    joints.destroy(id);
}

void Engine::drawJoints() {
    for (auto &id : joints.liveSlots()) {
        Joint &j = joints[id];
        PixelBody *a = bodies.get(j.a);
        if (a == nullptr) continue;
        PixelBody *b = bodies.get(j.b);
        if (b == nullptr) continue;

        sf::Vector2f pb = b->worldPxCenter(j.pb);
        
        if (j.type == Joint::FIXED
         || j.type == Joint::BEARING
         || j.type == Joint::MOTOR
         || j.type == Joint::ANG_SPRING
        ) {
            sf::Vector2f pa = a->worldPxCenter(j.pa);
            
            sf::Color col = sf::Color::Cyan;
    
            sf::Vertex line[] = {
                {pa, col},
                {pb, col},
            };
    
            window.draw(line, 2, sf::PrimitiveType::Lines);

            sf::CircleShape circ;
            circ.setRadius(3.f);
            circ.setOrigin({3.f, 3.f});
            circ.setFillColor(sf::Color::Red);
            // draw anchor A
            circ.setPosition(pa);
            window.draw(circ);
            // draw anchor B
            circ.setPosition(pb);
            window.draw(circ);

        } else if (j.type == Joint::LIN_BEARING
         || j.type == Joint::PISTON
        ) {
            sf::Vector2f apos = a->pos + Utils::rotVec(j.axis_pos, a->getCS());

            // draw course
            sf::Vector2f cv = Utils::rotVec({std::cos(j.axis_angle), std::sin(j.axis_angle)}, a->getCS());
            // an unlimited course has no finite endpoint to draw; run it off-screen instead
            const float reach = float(window.getSize().x + window.getSize().y);
            sf::Vector2f ca = apos + fmaxf(j.course_min, -reach) * cv;
            sf::Vector2f cb = apos + fminf(j.course_max,  reach) * cv;

            sf::Color course_color = sf::Color::Magenta;
            sf::Vertex line[] = {
                {ca, course_color},
                {cb, course_color},
            };

            window.draw(line, 2, sf::PrimitiveType::Lines);

            // draw anchor A & B
            sf::CircleShape circ;
            circ.setRadius(3.f);
            circ.setOrigin({3.f, 3.f});
            circ.setFillColor(sf::Color::Red);
            
            circ.setPosition(apos);
            window.draw(circ);
            circ.setPosition(pb);
            window.draw(circ);
        }
    }
}