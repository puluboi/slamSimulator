#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <map>
#include "sensors.hpp"
#include "odometry.hpp"
#include "movement.hpp"
#include "renderer.hpp"
#include "landmark.hpp"
#include "pathfinding.hpp"
#include "slam.hpp"
#include "iostream"

class Agent {
public:
    Agent();
    void init(sf::Vector2f startPosition);
    void updateState(float intervalTime);
    void update(float deltaTime, bool keys[4], sf::Vector2u windowSize,
                const std::vector<Landmark>& landmarks = {});
    void render(sf::RenderWindow& window);
    void renderMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, sf::Vector2f minimapSize, sf::Vector2u worldSize,
                       const std::vector<Landmark>& landmarks = {});
    
    // exploration control methods
    void toggleExplorationMode() { explorationMode = !explorationMode; }
    bool isExplorationMode() const { return explorationMode; }
    void setExplorationMode(bool enabled) { explorationMode = enabled; }
                    
    void setLandmarks(const std::vector<Landmark>& newLandmarks) {
        sensors.setLandmarks(newLandmarks);
    }
    
    // getter methods for agent state and sensor data
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
    
    // get total average error across all detected landmarks
    float getTotalAverageError() const { return sensors.getTotalAverageError(); }
    
    // get average error across all detected landmarks
    float getAverageDetectedLandmarkError() const { return sensors.calculateAverageDetectedLandmarkError(); }
    
    // gps control methods
    void setGPSEnabled(bool enabled) { sensors.setGPSEnabled(enabled); }
    bool isGPSEnabled() const { return sensors.isGPSEnabled(); }
    
    // pathfinding and exploration getter methods
    const std::vector<sf::Vector2f>& getCurrentPath() const { return currentPath; }
    sf::Vector2f getCurrentTarget() const { return currentTarget; }
    size_t getCurrentPathIndex() const { return currentPathIndex; }
    
    // agent's internal map access for honest slam implementation
    const std::vector<Landmark>& getAgentKnownLandmarks() const { return agentKnownLandmarks; }
    const std::map<std::pair<int, int>, bool>& getKnownOccupancyGrid() const { return knownOccupancyGrid; }
    const std::map<std::pair<int, int>, bool>& getExploredGrid() const { return exploredGrid; }

   

private:
    // component objects for agent functionality
    Sensors sensors;
    Odometry odometry;
    Movement movement;
    Pathfinding pathfinder;

    SLAM slam;
    
    // landmark id mapping for slam updates
    std::map<int, int> landmarkIdToSlamIndex; // maps landmark id to slam state index

    float sensorUpdateTimer = 0.0f;
    float stateUpdateTimer = 0.0f;
    

    // exploration state variables
    bool explorationMode;
    std::vector<sf::Vector2f> currentPath;
    size_t currentPathIndex;
    sf::Vector2f currentTarget;
    float pathUpdateTimer;
    
    // agent's internal slam map representation
    std::map<std::pair<int, int>, bool> knownOccupancyGrid;  // true = occupied, false = free
    std::map<std::pair<int, int>, bool> exploredGrid;        // true = explored, false = unknown
    std::vector<Landmark> agentKnownLandmarks;               // only landmarks the agent has seen
    sf::Vector2f mapOrigin;                                  // agent's coordinate system origin
    const int gridCellSize = 20;
    
    // helper methods for honest slam implementation
    void updateAgentMap();
    sf::Vector2f findNearestUnexploredArea();
    void initializeAgentMap(sf::Vector2f startPosition);
    
    // state tracking variables
    float gameTime;
    float tracerLength;
    sf::Vector2f previousPosition;
    sf::Vector2f previousVelocity;
    float previousDirection;
};