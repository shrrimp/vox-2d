#include "XPM.h"
#include "PixelBoxy.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <cctype>

namespace {

    std::string readFile(const std::string &path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("XPM: cannot open '" + path + "'");
        std::ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    // Every double-quoted literal in the file, in order. C block comments outside
    // of literals are skipped, so annotated colour tables parse fine.
    std::vector<std::string> literals(const std::string &src, const std::string &path) {
        std::vector<std::string> out;
        size_t i = 0;
        while (i < src.size()) {
            if (src[i] == '/' && i + 1 < src.size() && src[i + 1] == '*') {
                size_t end = src.find("*/", i + 2);
                i = (end == std::string::npos) ? src.size() : end + 2;
            } else if (src[i] == '"') {
                std::string lit;
                i++;
                while (i < src.size() && src[i] != '"') {
                    if (src[i] == '\\' && i + 1 < src.size()) { lit.push_back(src[i + 1]); i += 2; }
                    else lit.push_back(src[i++]);
                }
                if (i >= src.size()) throw std::runtime_error("XPM: unterminated string in '" + path + "'");
                i++; // closing quote
                out.push_back(lit);
            } else i++;
        }
        return out;
    }

    std::vector<std::string> words(const std::string &s) {
        std::vector<std::string> t;
        std::istringstream is(s);
        std::string w;
        while (is >> w) t.push_back(w);
        return t;
    }

    bool ieq(const std::string &a, const char *b) {
        if (a.size() != strlen(b)) return false;
        for (size_t i = 0; i < a.size(); i++)
            if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) return false;
        return true;
    }

    int hexDigit(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    }

    // #RGB, #RRGGBB, #RRRGGGBBB and #RRRRGGGGBBBB are all legal XPM. Named colours
    // are not resolved (no table to resolve them against) and fall back to white.
    sf::Color parseColor(const std::string &spec) {
        if (spec.empty() || ieq(spec, "None")) return sf::Color::Transparent;
        if (spec[0] != '#') return sf::Color::White;

        const std::string h = spec.substr(1);
        if (h.size() % 3 != 0 || h.empty() || h.size() > 12) return sf::Color::White;
        const size_t per = h.size() / 3;
        const unsigned full = (1u << (4 * per)) - 1u;

        uint8_t ch[3];
        for (size_t c = 0; c < 3; c++) {
            unsigned v = 0;
            for (size_t k = 0; k < per; k++) {
                const int d = hexDigit(h[c * per + k]);
                if (d < 0) return sf::Color::White;
                v = v * 16u + unsigned(d);
            }
            ch[c] = uint8_t((v * 255u + full / 2u) / full);
        }
        return sf::Color(ch[0], ch[1], ch[2]);
    }

    bool isTableKey(const std::string &w) {
        return w == "c" || w == "m" || w == "g" || w == "g4" || w == "s";
    }

    // "<char> {<key> <value...>}+"  -- values may be multi-word ("light blue")
    XPM::Entry parseEntry(const std::string &line, const std::string &path) {
        if (line.empty()) throw std::runtime_error("XPM: empty colour table line in '" + path + "'");

        XPM::Entry e;
        e.key = line[0];

        std::string key, val;
        auto flush = [&]() {
            if (key == "c") { e.color = parseColor(val); e.empty = ieq(val, "None"); }
            else if (key == "s") e.material = val;
            key.clear();
            val.clear();
        };

        for (const std::string &w : words(line.substr(1))) {
            if (isTableKey(w)) { flush(); key = w; }
            else { if (!val.empty()) val += ' '; val += w; }
        }
        flush();
        return e;
    }
}

XPM::Image XPM::load(const std::string &path) {
    const std::vector<std::string> lit = literals(readFile(path), path);
    if (lit.empty()) throw std::runtime_error("XPM: no string literals in '" + path + "'");

    const std::vector<std::string> head = words(lit[0]);
    if (head.size() < 4)
        throw std::runtime_error("XPM: bad header in '" + path + "': \"" + lit[0] + "\"");

    long w, h, ncolors, cpp;
    try {
        w       = std::stol(head[0]);
        h       = std::stol(head[1]);
        ncolors = std::stol(head[2]);
        cpp     = std::stol(head[3]);
    } catch (const std::exception &) {
        throw std::runtime_error("XPM: non-numeric header in '" + path + "': \"" + lit[0] + "\"");
    }

    if (w <= 0 || h <= 0 || ncolors <= 0)
        throw std::runtime_error("XPM: degenerate header in '" + path + "': \"" + lit[0] + "\"");
    if (cpp != 1)
        throw std::runtime_error("XPM: '" + path + "' uses " + std::to_string(cpp) +
                                 " chars per pixel; the pixel grid holds one char per pixel");

    const size_t body = 1 + size_t(ncolors);
    if (lit.size() < body + size_t(h))
        throw std::runtime_error("XPM: '" + path + "' declares " + std::to_string(ncolors) +
                                 " colours and " + std::to_string(h) + " rows but holds only " +
                                 std::to_string(lit.size()) + " strings");

    Image img;
    img.width  = (unsigned int)w;
    img.height = (unsigned int)h;

    bool known[256] = {false};
    for (long i = 0; i < ncolors; i++) {
        Entry e = parseEntry(lit[1 + size_t(i)], path);
        if ((unsigned char)e.key >= 128)
            throw std::runtime_error("XPM: '" + path + "' uses a non-ASCII pixel character");
        known[(unsigned char)e.key] = true;
        img.palette.push_back(e);
    }

    for (long y = 0; y < h; y++) {
        const std::string &row = lit[body + size_t(y)];
        if (row.size() != size_t(w))
            throw std::runtime_error("XPM: '" + path + "' row " + std::to_string(y) + " is " +
                                     std::to_string(row.size()) + " chars, header says " + std::to_string(w));
        for (char c : row)
            if (!known[(unsigned char)c])
                throw std::runtime_error("XPM: '" + path + "' row " + std::to_string(y) +
                                         " uses '" + std::string(1, c) + "', absent from the colour table");
        img.rows.push_back(row);
    }

    // --- empty character normalisation ---
    // Built as a whole table first, then applied in one pass, so a swap between
    // two characters can't alias.
    char remap[256];
    for (int i = 0; i < 256; i++) remap[i] = char(i);

    for (Entry &e : img.palette) {
        const unsigned char k = (unsigned char)e.key;
        if (e.empty) {
            remap[k] = PX_EMPTY;
            e.key = PX_EMPTY;
        } else if (e.key == PX_EMPTY) {
            // solid pixel declared with the empty character: move it out of the way
            char sub = 0;
            for (int c = '!'; c <= '~'; c++)
                if (!known[c]) { sub = char(c); break; }
            if (sub == 0)
                throw std::runtime_error("XPM: '" + path + "' has no free character to relocate a solid ' '");
            known[(unsigned char)sub] = true;
            remap[k] = sub;
            e.key = sub;
        }
    }

    for (std::string &row : img.rows)
        for (char &c : row) c = remap[(unsigned char)c];

    // --- extensions ---
    for (size_t i = body + size_t(h); i < lit.size(); i++) {
        const std::vector<std::string> t = words(lit[i]);
        if (t.empty()) continue;
        if (t[0] == "XPMENDEXT") break;
        if (t[0] != "XPMEXT" || t.size() < 2) continue;

        Extension ex;
        ex.name = t[1];
        ex.words.assign(t.begin() + 2, t.end());
        img.extensions.push_back(ex);
    }

    return img;
}
