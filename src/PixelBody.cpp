#include "Settings.h"
#include "PixelBoxy.h"
#include "Utils.h"
#include "XPM.h"
#include <algorithm>
#include <cstdlib>

bool compPos(const CornerPx &c1, const CornerPx &c2) {
    return c1.p == c2.p;
}

static const Material default_material{};

namespace {
    // "XPMEXT material <name> <key> <value> ..." -- unknown keys are ignored so a
    // file can carry properties this build doesn't know about yet.
    void applyExtensions(std::vector<Material> &materials, const std::vector<XPM::Extension> &exts) {
        for (const XPM::Extension &ex : exts) {
            if (ex.name != "material" || ex.words.empty()) continue;

            for (Material &m : materials) {
                if (m.name != ex.words[0]) continue;
                for (size_t i = 1; i + 1 < ex.words.size(); i += 2) {
                    const float v = strtof(ex.words[i + 1].c_str(), nullptr);
                    if      (ex.words[i] == "density")     m.density = v;
                    else if (ex.words[i] == "friction")    m.friction = v;
                    else if (ex.words[i] == "restitution") m.restitution = v;
                    else if (ex.words[i] == "strength")    m.strength = v;
                }
            }
        }
    }
}

const Material &PixelBody::material(const size_t x, const size_t y) const {
    if (materials.empty()) return default_material;
    const unsigned char c = (unsigned char)getPx(x, y);
    if (c >= 128 || mat_of[c] >= materials.size()) return default_material;
    return materials[mat_of[c]];
}

float PixelBody::density(const char c) const {
    if (materials.empty()) return 1.f;
    const unsigned char u = (unsigned char)c;
    if (u >= 128 || mat_of[u] >= materials.size()) return 1.f;
    return materials[mat_of[u]].density;
}

float PixelBody::imass() const {
    return (fixed ? 0.f : inv_mass);
}

float PixelBody::iinertia() const {
    return (fixed ? 0.f : inv_inertia);
}

PixelBody::PixelBody(const std::vector<std::string> &d, const sf::Vector2u &s, const sf::Vector2f &p) {
    pos = p;
    size = s;
    build(d);
}

PixelBody::PixelBody(const std::vector<std::string> &d, const sf::Vector2u &s, const sf::Vector2f &p, bool isFixed) {
    pos = p;
    size = s;
    fixed = isFixed;
    build(d);
}

PixelBody::PixelBody(const std::string &xpm_file, const sf::Vector2f &p) {
    const XPM::Image img = XPM::load(xpm_file);

    // XPM is row major, data is column major -> transpose
    std::vector<std::string> d(img.width, std::string(img.height, PX_EMPTY));
    for (unsigned int y = 0; y < img.height; y++)
        for (unsigned int x = 0; x < img.width; x++)
            d[x][y] = img.rows[y][x];

    // one material per solid colour-table entry, reachable straight from the pixel char
    mat_of.fill(0xFF);
    for (const XPM::Entry &e : img.palette) {
        if (e.empty) continue;
        Material m;
        m.name = e.material;
        m.color = e.color;
        mat_of[(unsigned char)e.key] = (uint8_t)materials.size();
        materials.push_back(m);
    }
    applyExtensions(materials, img.extensions);

    pos = p;
    size = sf::Vector2u(img.width, img.height);
    build(d);
}

PixelBody::PixelBody(const std::string &xpm_file, const sf::Vector2f &p, bool fixed) {
    const XPM::Image img = XPM::load(xpm_file);

    // XPM is row major, data is column major -> transpose
    std::vector<std::string> d(img.width, std::string(img.height, PX_EMPTY));
    for (unsigned int y = 0; y < img.height; y++)
        for (unsigned int x = 0; x < img.width; x++)
            d[x][y] = img.rows[y][x];

    // one material per solid colour-table entry, reachable straight from the pixel char
    mat_of.fill(0xFF);
    for (const XPM::Entry &e : img.palette) {
        if (e.empty) continue;
        Material m;
        m.name = e.material;
        m.color = e.color;
        mat_of[(unsigned char)e.key] = (uint8_t)materials.size();
        materials.push_back(m);
    }
    applyExtensions(materials, img.extensions);

    pos = p;
    this->fixed = fixed;
    size = sf::Vector2u(img.width, img.height);
    build(d);
}

PixelBody::PixelBody(const std::string &xpm_file, const sf::Vector2f &p, bool fixed, float angle) {
    const XPM::Image img = XPM::load(xpm_file);

    // XPM is row major, data is column major -> transpose
    std::vector<std::string> d(img.width, std::string(img.height, PX_EMPTY));
    for (unsigned int y = 0; y < img.height; y++)
        for (unsigned int x = 0; x < img.width; x++)
            d[x][y] = img.rows[y][x];

    // one material per solid colour-table entry, reachable straight from the pixel char
    mat_of.fill(0xFF);
    for (const XPM::Entry &e : img.palette) {
        if (e.empty) continue;
        Material m;
        m.name = e.material;
        m.color = e.color;
        mat_of[(unsigned char)e.key] = (uint8_t)materials.size();
        materials.push_back(m);
    }
    applyExtensions(materials, img.extensions);

    pos = p;
    ang = angle;
    this->fixed = fixed;
    size = sf::Vector2u(img.width, img.height);
    build(d);
}

void PixelBody::build(const std::vector<std::string> &d) {
    const sf::Vector2u s = size;

    size_t pixels = 0;
    float mass = 0.f;
    sf::Vector2f px_pos = sf::Vector2f(0.f, 0.f);

    for (size_t x = 0; x < s.x; x++) {
        data.push_back(d[x]);
        for (size_t y = 0; y < s.y; y++) {
            if (d[x][y] != PX_EMPTY) {
                const float dens = density(d[x][y]);
                pixels++;
                mass += dens;
                px_pos += dens * sf::Vector2f(.5f + x, .5f + y);
            }
        }
    }

    if (pixels == 0 || mass <= 0.f) {
        size = sf::Vector2u(0, 0);
        pos = sf::Vector2f(0.f, 0.f);
        data.clear();
        return;
    }

    centroid = px_pos / mass; // density weighted
    inv_mass = 1.f / mass;

    // calculate bounding radius, inertia -> all need data and centroid to be valid hense another loop
    float inertia = 0.f;
    float max = 0.f;
    for (unsigned int x = 0; x < size.x; x++) {
        offsets.push_back({});
        for (unsigned int y = 0; y < size.y; y++) {
            sf::Vector2f px_center = sf::Vector2f(.5f + x, .5f + y);
            offsets[x].push_back((px_center - centroid) * PX_SIZE);
            if (d[x][y] != PX_EMPTY) {
                float lsqrd = (px_center - centroid).lengthSquared();
                if (lsqrd > max) max = lsqrd;

                float pixel_mass = density(d[x][y]); // before d is shadowed below

                // store edge pixels LRTD
                bool l = (getPx(x - 1, y) == PX_EMPTY);
                bool r = (getPx(x + 1, y) == PX_EMPTY);
                bool t = (getPx(x, y - 1) == PX_EMPTY);
                bool d = (getPx(x, y + 1) == PX_EMPTY);
                char mask =
                    l * 0b1000 |
                    r * 0b0100 |
                    t * 0b0010 |
                    d * 0b0001;

                // if ((mask & 0b1100) && (mask & 0b0011)) // this is a corner (optimisation probably useless without live updates)
                // if (mask) { // this is an edge
                if ((l ||  r) && (t || d)) { // this is a corner
                    corners.push_back({{x, y}, mask});
                };

                float di2 = offsets[x][y].lengthSquared();
                float px_inertia = (pixel_mass * PX_SIZE * PX_SIZE) / 6.f + pixel_mass * di2;
                inertia += px_inertia;
            }
        }
    }
    inv_inertia = inertia == 0.f ? 0.f : 1.f / inertia;

    radius = (sqrtf(max) + .70710678f) * PX_SIZE; // +[sqrt(2) / 2] to offset and encapsulate the whole outer pixel and not just it's center
}

bool PixelBody::filter(const BodyId &b) const {
    return find(f_bodies.begin(), f_bodies.end(), b) != f_bodies.end();
}

sf::Vector2f PixelBody::getCS() const {
    if (ca != ang) {
        ca = ang;
        cc = cosf(ang);
        cs = sinf(ang);
    }

    return sf::Vector2f(cc, cs);
}

char PixelBody::getPx(const sf::Vector2u &p) const {
    if (p.x >= size.x || p.y >= size.y) return PX_EMPTY; // empty for out of bounds
    return data[p.x][p.y];
}
char PixelBody::getPx(const size_t x, const size_t y) const {
    if (x >= size.x || y >= size.y) return PX_EMPTY; // empty for out of bounds
    return data[x][y];
}

char PixelBody::getMask(const int x, const int y) const {
    bool l = (getPx(x - 1, y) == PX_EMPTY);
    bool r = (getPx(x + 1, y) == PX_EMPTY);
    bool t = (getPx(x, y - 1) == PX_EMPTY);
    bool d = (getPx(x, y + 1) == PX_EMPTY);
    char mask =
        l * 0b1000 |
        r * 0b0100 |
        t * 0b0010 |
        d * 0b0001;
    return mask;
}

sf::Vector2f PixelBody::worldVertex(const sf::Vector2u &coords) const {
    sf::Vector2f cossin = getCS();
    return pos - Utils::rotVec(centroid * PX_SIZE, cossin) + Utils::rotVec(
        sf::Vector2f(coords.x * PX_SIZE, coords.y * PX_SIZE),
        cossin
    );
}

sf::Vector2f PixelBody::worldPxCenter(const sf::Vector2u &coords) const {
    sf::Vector2f cossin = getCS();
    return pos - Utils::rotVec(centroid * PX_SIZE, cossin) + Utils::rotVec(
        sf::Vector2f((.5f + coords.x) * PX_SIZE, (.5f + coords.y) * PX_SIZE),
        cossin
    );
}

sf::Vector2f PixelBody::worldPxCenter(const size_t x, const size_t y) const {
    sf::Vector2f cossin = getCS();
    return pos - Utils::rotVec(centroid * PX_SIZE, cossin) + Utils::rotVec(
        sf::Vector2f((.5f + x) * PX_SIZE, (.5f + y) * PX_SIZE),
        cossin
    );
}

sf::Vector2i PixelBody::pxAt(const sf::Vector2f &p) const {
    sf::Vector2f cossin = getCS(); // cos(-a) = cos(a) & sin(-a) = -sin(a) -> we avoid recomputing cos and sin
    sf::Vector2f local = Utils::rotVec(p - pos, cossin.x, -cossin.y) + (centroid * PX_SIZE);

    if (local.x < 0.f || local.y < 0.f ||
        local.x >= size.x * PX_SIZE || local.y >= size.y * PX_SIZE)
        return sf::Vector2i(-1,-1);

    return sf::Vector2i(floorf(local.x / PX_SIZE), floorf(local.y / PX_SIZE));
}

sf::Vector2f PixelBody::fpxAt(const sf::Vector2f &p) const {
    sf::Vector2f cossin = getCS(); // cos(-a) = cos(a) & sin(-a) = -sin(a) -> we avoid recomputing cos and sin
    sf::Vector2f local = Utils::rotVec(p - pos, cossin.x, -cossin.y) + (centroid * PX_SIZE);

    // no clamp to have valid local values even if they're outside of the boundaries

    return local / PX_SIZE; // return floating point value
}

void PixelBody::draw(sf::RenderWindow &w) const {
    sf::Vector2f cossin = getCS();

    sf::ConvexShape px;
    px.setPointCount(4);
    px.setPoint(0, Utils::rotVec(sf::Vector2f(- PX_SIZE,   PX_SIZE) / 2.f, cossin));
    px.setPoint(1, Utils::rotVec(sf::Vector2f(- PX_SIZE, - PX_SIZE) / 2.f, cossin));
    px.setPoint(2, Utils::rotVec(sf::Vector2f(  PX_SIZE, - PX_SIZE) / 2.f, cossin));
    px.setPoint(3, Utils::rotVec(sf::Vector2f(  PX_SIZE,   PX_SIZE) / 2.f, cossin));
    px.setFillColor(color);

    sf::VertexArray outline(sf::PrimitiveType::Lines);
    auto edge = [&](const sf::Vector2u &a, const sf::Vector2u &b) {
        outline.append({worldVertex(a), outline_color});
        outline.append({worldVertex(b), outline_color});
    };

    for (unsigned int x = 0; x < size.x; x++) {
        for (unsigned int y = 0; y < size.y; y++) {
            if (data[x][y] != PX_EMPTY) {
                CornerPx p_{{x, y}, 0};
                // if (find_if(corners.begin(), corners.end(), [&p_](const CornerPx& p){return p_.p == p.p;}) != corners.end()) {
                    // } else px.setFillColor(color);
                px.setFillColor(material(x, y).color);
                px.setPosition(pos - Utils::rotVec(centroid * PX_SIZE, cossin) + Utils::rotVec(
                    sf::Vector2f((.5f + x) * PX_SIZE, (.5f + y) * PX_SIZE),
                    cossin
                ));

                w.draw(px);

                const char mask = getMask(x, y);
                if (mask & 0b1000) edge({x,     y    }, {x,     y + 1});
                if (mask & 0b0100) edge({x + 1, y    }, {x + 1, y + 1});
                if (mask & 0b0010) edge({x,     y    }, {x + 1, y    });
                if (mask & 0b0001) edge({x,     y + 1}, {x + 1, y + 1});
            }
        }
    }

    w.draw(outline);

    // sf::CircleShape center;
    // center.setRadius(2.f);
    // center.setOrigin({2.f, 2.f});
    // center.setPosition(pos);
    // center.setFillColor(sf::Color::Red);

    // w.draw(center);
    
    // sf::CircleShape bound;
    // bound.setRadius(radius);
    // bound.setOrigin({radius, radius});
    // bound.setPosition(pos);
    // bound.setFillColor(sf::Color::Transparent);
    // bound.setOutlineThickness(1);
    // bound.setOutlineColor(sf::Color::Yellow);

    // w.draw(bound);
}