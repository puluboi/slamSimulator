#include "game.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include "math.h"

Game::Game() : window(sf::VideoMode(800, 600), "SLAM Simulator"){
    // initialize input keys
    for(int i = 0; i < 4; i++) {
        keys[i] = false;
    }
}

void Game::init(){
    // initialize agent
    agent.init(sf::Vector2f(100.0f, 100.0f));
    
    // initialize UI elements for error display
    // use Ubuntu font which should be available on most Linux systems
    if (!font.loadFromFile("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf")) {
        // try alternative font paths
        if (!font.loadFromFile("/usr/share/fonts/TTF/arial.ttf")) {
            std::cout << "Warning: Could not load font, using default rendering" << std::endl;
        }
    }
    
    errorText.setFont(font);
    errorText.setCharacterSize(16);
    errorText.setFillColor(sf::Color::Black);
    errorText.setPosition(10.0f, 10.0f); // Top-left corner
    
    // initialize performance metrics text
    performanceText.setFont(font);
    performanceText.setCharacterSize(14);
    performanceText.setFillColor(sf::Color::Blue);
    performanceText.setPosition(10.0f, 200.0f); // Below error text
    
    // walls around
    createWall(0.f,0.f, 0.f, 600.f);
    createWall(800.f,0.f, 800.f, 600.f);
    createWall(0.f,0.f, 800.f, 0.f);
    createWall(0.f,600.f, 800.f, 600.f);
    // inner walls
    createWall(300.f,0.f, 300.f, 450.f);
    createWall(400.f,0.f, 400.f, 400.f);
    createWall(300.f,0.f, 300.f, 200.f);

    createWall(300.f, 200.f, 100.f, 200.f);
    createWall(0.f, 300.f, 220.f, 300.f);
    createWall(100.f, 400.f, 300.f, 400.f);

    createWall(500.f, 400.f, 800.f, 400.f);
    agent.setLandmarks(landmarks);
    
}

void Game::run(){
while (window.isOpen()){
    handleEvents();
    update();
    render();
}
}


void Game::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
                
        // handle key presses
        if (event.type == sf::Event::KeyPressed) {
            if (event.key.code == sf::Keyboard::G) {
                // toggle GPS with G key
                agent.setGPSEnabled(!agent.isGPSEnabled());
            }
            if (event.key.code == sf::Keyboard::Space) {
                // toggle exploration mode with spacebar
                agent.toggleExplorationMode();
            }
            if (event.key.code == sf::Keyboard::T) {
                // toggle SLAM algorithm with T key
                agent.toggleSLAMAlgorithm();
                std::cout << "switched slam algorithm to: " 
                          << (agent.getSLAMAlgorithm() == SLAMAlgorithm::EKF_SLAM ? "ekf-slam" : "ufastslam") 
                          << std::endl;
            }
        }
    }
    
    // handle continuous key presses (only used when not in exploration mode)
    keys[0] = sf::Keyboard::isKeyPressed(sf::Keyboard::W); // Up
    keys[1] = sf::Keyboard::isKeyPressed(sf::Keyboard::A); // Left
    keys[2] = sf::Keyboard::isKeyPressed(sf::Keyboard::S); // Down
    keys[3] = sf::Keyboard::isKeyPressed(sf::Keyboard::D); // Right
}

void Game::update() {
    // calculate delta time (simplified for this example)
    float deltaTime = 1.0f / 60.0f; // Assuming 60 FPS
    
    // update agent
    agent.update(deltaTime, keys, window.getSize(), landmarks);
    
    
}

void Game::render() {
    window.clear(sf::Color::White);
    
    // let agent render itself
    agent.render(window);
    for (const auto& landmark : landmarks) {
        window.draw(landmark.getShape());
    }
    
    // update and render error information
    // get sensor error data
    float frameAvgError = agent.getSensors().getAverageLandmarkError();
    float totalAvgError = agent.getTotalAverageError();
    const std::vector<float>& individualErrors = agent.getSensors().getLandmarkErrors();
    
    // create error display string with debug info
    std::string errorDisplay = "Landmark Errors:\n";
    errorDisplay += "Frame Avg: " + std::to_string(frameAvgError) + " px\n";
    errorDisplay += "Total Avg: " + std::to_string(totalAvgError) + " px\n";
    errorDisplay += "Count: " + std::to_string(individualErrors.size()) + "\n";
    
    // add slam algorithm info
    errorDisplay += "\nSLAM Algorithm: ";
    errorDisplay += (agent.getSLAMAlgorithm() == SLAMAlgorithm::EKF_SLAM ? "EKF-SLAM" : "UFastSLAM");
    errorDisplay += "\n(Press T to switch)\n";
    
    if (!individualErrors.empty()) {
        errorDisplay += "Individual:\n";
        for (size_t i = 0; i < std::min(size_t(5), individualErrors.size()); ++i) {
            errorDisplay += "L" + std::to_string(i) + ": " + std::to_string(individualErrors[i]) + "\n";
        }
    } else {
        errorDisplay += "No landmarks detected\n";
    }
    
    errorText.setString(errorDisplay);
    errorText.setFillColor(sf::Color::Red); // Make it more visible
    
    // draw error text with background
    sf::RectangleShape textBackground;
    sf::FloatRect textBounds = errorText.getLocalBounds();
    textBackground.setSize(sf::Vector2f(textBounds.width + 10, textBounds.height + 10));
    textBackground.setPosition(errorText.getPosition().x - 5, errorText.getPosition().y - 5);
    textBackground.setFillColor(sf::Color(255, 255, 255, 200)); // Semi-transparent white
    textBackground.setOutlineColor(sf::Color::Black);
    textBackground.setOutlineThickness(1.0f);
    
    window.draw(textBackground);
    window.draw(errorText);
    
    // create and render performance metrics text
    std::string performanceDisplay = "=== PERFORMANCE METRICS ===\n";
    performanceDisplay += "SLAM Update Time: " + std::to_string(agent.getSLAMUpdateTime()) + " μs\n";
    performanceDisplay += "SLAM Position Error: " + std::to_string(agent.getSLAMPositionError()) + " units\n";
    performanceDisplay += "Odometry Position Error: " + std::to_string(agent.getOdometryPositionError()) + " units\n";
    
    float slamError = agent.getSLAMPositionError();
    float odoError = agent.getOdometryPositionError();
    float improvement = odoError - slamError;
    
    performanceDisplay += "SLAM Improvement: " + std::to_string(improvement) + " units\n";
    performanceDisplay += "SLAM LSE: " + std::to_string(slamError * slamError) + "\n";
    performanceDisplay += "Odometry LSE: " + std::to_string(odoError * odoError) + "\n";
    
    performanceText.setString(performanceDisplay);
    
    // draw performance text with background
    sf::RectangleShape perfTextBackground;
    sf::FloatRect perfTextBounds = performanceText.getLocalBounds();
    perfTextBackground.setSize(sf::Vector2f(perfTextBounds.width + 10, perfTextBounds.height + 10));
    perfTextBackground.setPosition(performanceText.getPosition().x - 5, performanceText.getPosition().y - 5);
    perfTextBackground.setFillColor(sf::Color(240, 240, 255, 200)); // Light blue background
    perfTextBackground.setOutlineColor(sf::Color::Blue);
    perfTextBackground.setOutlineThickness(1.0f);
    
    window.draw(perfTextBackground);
    window.draw(performanceText);
    
    // render minimap on the right side
    sf::Vector2f minimapSize(150.0f, 150.0f); // 150x150 pixel minimap
    sf::Vector2f minimapPosition(window.getSize().x - minimapSize.x - 10.0f, 10.0f); // Top-right corner with 10px margin
    agent.renderMinimap(window, minimapPosition, minimapSize, window.getSize(), agent.getDetectedLandmarks());
    
    window.display();
}

void Game::createLandmark(float x, float y){
    sf::Vector2f Size(landmarkSize, landmarkSize);
    sf::RectangleShape rectangle(Size);
    rectangle.setFillColor(sf::Color(100,100,100,255));
    sf::Vector2f Pos(x, y);
    rectangle.setPosition(x-Size.x*0.5, y-Size.y*0.5);
    Landmark landmark(rectangle, static_cast<int>(landmarks.size()));
    landmarks.push_back(landmark);
}
void Game::createWall(float xStart, float yStart, float xEnd, float yEnd){
    float distance = sqrt(pow(xEnd - xStart, 2) + pow(yEnd - yStart, 2));
    sf::Vector2f direction((xEnd - xStart) / distance, (yEnd - yStart) / distance);

    
    for (float i = 0; i <= distance; i += landmarkSize) {
        sf::Vector2f pos(xStart + direction.x * i, yStart + direction.y * i);
        createLandmark(pos.x, pos.y);
    }
}