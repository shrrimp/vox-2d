#include "config.h"
#include "Engine.h"
#include <iostream>
#include <optional>
#include <SFML/Graphics.hpp>
using namespace std;

int main() {
    cout << "Version " << VERSION_MAJOR << "." << VERSION_MINOR << endl;

    Engine App;
    App.loop();

    return 0;
}
