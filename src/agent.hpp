#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include "sensors.hpp"
#include "odometry.hpp"
#include "movement.hpp"
#include "renderer.hpp"
#include "landmark.hpp"
#include "pathfinding.hpp"
#include "ekf.hpp"

class Agent {
public:
    Agent();
    void init(sf::Vector2f startPosition);
    void update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                const std::vector<Landmark>& landmarks = {});
    void render(sf::RenderWindow& window);
    void renderMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, sf::Vector2f minimapSize, sf::Vector2u worldSize,
                       const std::vector<Landmark>& landmarks = {});
    
    // Exploration control
    void toggleExplorationMode() { explorationMode = !explorationMode; }
    bool isExplorationMode() const { return explorationMode; }
    void setExplorationMode(bool enabled) { explorationMode = enabled; }
                    
    void setLandmarks(const std::vector<Landmark>& newLandmarks) {
        sensors.setLandmarks(newLandmarks);
    }
    
    // Getters
    sf::Vector2f getPosition() const { return movement.getPosition(); }
    sf::RectangleShape getShape() const { return movement.getShape(); }
    const std::vector<sf::Vertex>& getLineTracers() const { return sensors.getLineTracers(); }
    const std::vector<AccelerometerData>& getAccelerometerData() const { return sensors.getAccelerometerData(); }
    const std::vector<GyroscopeData>& getGyroscopeData() const { return sensors.getGyroscopeData(); }
    const std::vector<GPSData>& getGPSData() const { return sensors.getGPSData(); }
    sf::Vector2f getCurrentGPSPosition() const { return sensors.getCurrentGPSPosition(); }
    sf::Vector2f getOdometryPosition() const { return odometry.getPosition(); }
    float getOdometryDirection() const { return odometry.getDirection(); }
    const std::vector<Landmark> getDetectedLandmarks() const { return sensors.getdetectedLandmarks(); }
    const Sensors& getSensors() const { return sensors; }
    
    // Get total average error across all detected landmarks
    float getTotalAverageError() const { return sensors.getTotalAverageError(); }
    
    // Get average error across all detected landmarks
    float getAverageDetectedLandmarkError() const { return sensors.calculateAverageDetectedLandmarkError(); }
    
    // GPS control
    void setGPSEnabled(bool enabled) { sensors.setGPSEnabled(enabled); }
    bool isGPSEnabled() const { return sensors.isGPSEnabled(); }
    
    // Pathfinding and exploration getters
    const std::vector<sf::Vector2f>& getCurrentPath() const { return currentPath; }
    sf::Vector2f getCurrentTarget() const { return currentTarget; }
    size_t getCurrentPathIndex() const { return currentPathIndex; }
    
    // Agent's internal map access (for honest SLAM)
    const std::vector<Landmark>& getAgentKnownLandmarks() const { return agentKnownLandmarks; }
    const std::map<std::pair<int, int>, bool>& getKnownOccupancyGrid() const { return knownOccupancyGrid; }
    const std::map<std::pair<int, int>, bool>& getExploredGrid() const { return exploredGrid; }

   

private:
    // Component objects
    Sensors sensors;
    Odometry odometry;
    Movement movement;
    Pathfinding pathfinder;

    // EKF components
    

    // Exploration state
    bool explorationMode;
    std::vector<sf::Vector2f> currentPath;
    size_t currentPathIndex;
    sf::Vector2f currentTarget;
    float pathUpdateTimer;
    
    // Agent's internal SLAM map (what it thinks the world looks like)
    std::map<std::pair<int, int>, bool> knownOccupancyGrid;  // true = occupied, false = free
    std::map<std::pair<int, int>, bool> exploredGrid;        // true = explored, false = unknown
    std::vector<Landmark> agentKnownLandmarks;               // Only landmarks the agent has seen
    sf::Vector2f mapOrigin;                                  // Agent's coordinate system origin
    const int gridCellSize = 20;
    
    // Helper methods for honest SLAM
    void updateAgentMap();
    sf::Vector2f findNearestUnexploredArea();
    void initializeAgentMap(sf::Vector2f startPosition);
    
    // State tracking
    float gameTime;
    float tracerLength;
    sf::Vector2f previousPosition;
    sf::Vector2f previousVelocity;
    float previousDirection;
};