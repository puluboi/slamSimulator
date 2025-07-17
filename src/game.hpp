#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include "agent.hpp"
#include "landmark.hpp"

class Game{
public:
    Game();
    void init();
    void run();

private:
    void handleEvents();
    void update();
    void render();


    void createLandmark(float x, float y);

    void createWall(float xStart, float yStart, float xEnd, float yEnd);

    sf::RenderWindow window;
    sf::Event event;
        std::vector<Landmark> landmarks;
    // agent
    Agent agent;
    float landmarkSize = 40.0f; 
    
    // ui elements for error display
    sf::Font font;
    sf::Text errorText;
    
    // input handling
    bool keys[4]; // W, A, S, D
};