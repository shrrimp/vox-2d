#pragma once
#include <SFML/Graphics/Color.hpp>
#include <string>
#include <vector>

// Runtime XPM3 reader. Knows nothing about physics: it hands back the image and
// the colour table, PixelBody decides what that means.
namespace XPM {

    struct Entry { // one line of the XPM colour table
        char key = ' ';              // the pixel character, after normalisation
        bool empty = false;          // declared "c None" -> transparent -> no pixel
        sf::Color color = sf::Color::White;
        std::string material;        // the XPM "s" symbolic name, empty if absent
    };

    struct Extension { // one "XPMEXT <name> <words...>" line
        std::string name;
        std::vector<std::string> words;
    };

    struct Image {
        unsigned int width = 0;
        unsigned int height = 0;
        std::vector<std::string> rows; // height strings of width chars, row major
        std::vector<Entry> palette;
        std::vector<Extension> extensions;
    };

    // Transparent characters are rewritten to PX_EMPTY, and any *solid* character
    // that happens to be PX_EMPTY is moved to a free character, so PX_EMPTY always
    // means "no pixel" no matter what the file declared.
    // Throws std::runtime_error on anything malformed.
    Image load(const std::string &path);
}
