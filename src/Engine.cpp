#include "Settings.h"
#include "Engine.h"
#include "Utils.h"
#include <math.h>
#include <SFML/System/Angle.hpp>
#include <algorithm>
#include <cassert>

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
    
    BodyId bar = bodies.create("assets/bar.xpm", sf::Vector2f(300.f, floor_height - 150.f), false, -M_PI_2);
    BodyId w1 = bodies.create("assets/ball.xpm", sf::Vector2f(100.f, floor_height - 150.f));
    BodyId w2 = bodies.create("assets/ball.xpm", sf::Vector2f(500.f, floor_height - 150.f));

    ommit_mouse.push_back(w1);
    ommit_mouse.push_back(w2);

    join(bar, {2, 2}, w1, {7, 7}, Joint::BEARING);
    join(bar, {2, 47}, w2, {7, 7}, Joint::BEARING);

    size_t amount = 100;
    for (size_t i = 0; i < amount; i++) {
        bodies.create("assets/px.xpm", sf::Vector2f(float(i + 1) * PX_SIZE, floor_height - PX_SIZE * 2));
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
        }

        if (selected.gen != 0) {
            PixelBody *body = bodies.get(selected);

            if (body == nullptr) selected = {0, 0};
            else {
                sf::Vector2f force_dir = (mposf - body->pos);
                body->forces += force_dir * 5.f * GRAVITY;
        
                sf::Vertex v[2] = {
                    {.position = body->pos, .color = sf::Color::Red},
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

    // build JointSolve array
    std::vector<JointSolve> sjoints;

    for (size_t id_i = joints.live.size(); id_i-- > 0; ) {
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

        s1.vel   -= s1.imass() * P;
        s1.omega -= s1.iinertia() * (c.p - s1.pos).cross(P);
        s2.vel   += s2.imass() * P;
        s2.omega += s2.iinertia() * (c.p - s2.pos).cross(P);
    }

    for (JointSolve &s : sjoints) {
        const sf::Vector2f P = s.j->lambda;
        s.A->vel   += s.A->imass()    * P;
        s.A->omega += s.A->iinertia() * s.ra.cross(P);
        s.B->vel   -= s.B->imass()    * P;
        s.B->omega -= s.B->iinertia() * s.rb.cross(P);
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
            s1.vel   -= s1.imass() * lapp * c.n;
            s1.omega -= s1.iinertia() * r1cn * lapp;
    
            s2.vel   += s2.imass() * lapp * c.n;
            s2.omega += s2.iinertia() * r2cn * lapp;

            float ltapp = ltn - c.lt;
            c.lt = ltn;
    
            // apply friction forces
            // no forces stored in the body's force fields? idk why
            s1.vel   -= s1.imass() * ltapp * t;
            s1.omega -= s1.iinertia() * r1ct * ltapp;
    
            s2.vel   += s2.imass() * ltapp * t;
            s2.omega += s2.iinertia() * r2ct * ltapp;
        }
        for (JointSolve &s : sjoints) {
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

    for (auto &id : joints.liveSlots()) {
        PixelBody *a = bodies.get(joints[id].a);
        if (a == nullptr) continue;
        PixelBody *b = bodies.get(joints[id].b);
        if (b == nullptr) continue;

        
        sf::Vector2f pa = a->worldPxCenter(joints[id].pa);
        sf::Vector2f pb = b->worldPxCenter(joints[id].pb);
        
        sf::Color col = sf::Color::Cyan;

        sf::Vertex line[] = {
            {pa, col},
            {pb, col},
        };

        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    

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
JointId Engine::join(BodyId a, sf::Vector2u pa, BodyId b, sf::Vector2u pb, Joint::JointType type) {
    // validate bodies
    PixelBody *ba = bodies.get(a);
    PixelBody *bb = bodies.get(b);
    if (ba == nullptr || bb == nullptr) return {};
    if (ba->getPx(pa) == PX_EMPTY || bb->getPx(pb) == PX_EMPTY) return {}; // Joints only work on valid pixels, not empty space

    // create filter
    bodies.filter(a, b, true);

    float rest_ang = ba->ang - bb->ang;
    return joints.create( a, pa, b, pb, type, rest_ang);
}

void Engine::disconnect(JointId id) {
    Joint *j = joints.get(id);
    if (j == nullptr) return;

    // destroy filter
    bodies.filter(j->a, j->b, false);
    
    joints.destroy(id);
}