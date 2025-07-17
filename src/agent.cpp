#include "agent.hpp"

Agent::Agent() : pathfinder(Pathfinding(800, 600, 20)) { // Initialize pathfinder with world size
    gameTime = 0.0f;
    tracerLength = 150.0f;
    previousPosition = sf::Vector2f(0.0f, 0.0f);
    previousVelocity = sf::Vector2f(0.0f, 0.0f);
    previousDirection = 0.0f;
    explorationMode = true;
    currentPathIndex = 0;
    currentTarget = sf::Vector2f(0.0f, 0.0f);
    pathUpdateTimer = 0.0f;
}

void Agent::init(sf::Vector2f startPosition) {
    // Initialize all components
    movement.init(startPosition);
    odometry.init(startPosition, 0.0f);
    
    // Initialize state
    previousPosition = startPosition;
    previousDirection = 0.0f;
    currentTarget = startPosition;
}


void Agent::update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                const std::vector<Landmark>& landmarks) {
    // Update game time
    gameTime += deltaTime;
    
    // Update pathfinder with detected landmarks using odometry position
    pathfinder.updateOccupancyGrid(sensors.getdetectedLandmarks(), odometry.getPosition());
    
    // Handle exploration mode
    bool explorationKeys[4] = {false, false, false, false};
    if (explorationMode) {
        pathUpdateTimer += deltaTime;
        
        // Update path every 0.5 seconds or when current path is completed
        if (pathUpdateTimer > 5.0f || currentPathIndex >= currentPath.size()) {
            // Find nearest unexplored area based on odometry position
            sf::Vector2f nearestUnexplored = pathfinder.findNearestUnexplored(odometry.getPosition());
            
            // If we found a new target, plan a path to it
            if (nearestUnexplored != odometry.getPosition()) {
                currentPath = pathfinder.findPath(odometry.getPosition(), nearestUnexplored);
                currentPathIndex = 0;
                currentTarget = nearestUnexplored;
            }
            
            pathUpdateTimer = 0.0f;
        }
        
        // Follow the current path
        if (!currentPath.empty() && currentPathIndex < currentPath.size()) {
            sf::Vector2f targetPos = currentPath[currentPathIndex]-sf::Vector2f(2,2);
            sf::Vector2f currentPos = odometry.getPosition(); // Use odometry position for navigation
            sf::Vector2f direction = targetPos - currentPos;
            
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            
            // Move towards target
            if (distance > 10.0f) { // Threshold for reaching waypoint
                // Calculate which keys to press to move towards target
                if (direction.y < -5.0f) explorationKeys[0] = true; // Up (W)
                if (direction.x < -5.0f) explorationKeys[1] = true; // Left (A)
                if (direction.y > 5.0f) explorationKeys[2] = true; // Down (S)
                if (direction.x > 5.0f) explorationKeys[3] = true; // Right (D)
            } else {
                // Reached current waypoint, move to next
                currentPathIndex++;
            }
        }
        
        // Use exploration keys instead of manual keys
        movement.update(deltaTime, explorationKeys, windowSize, landmarks);
    } else {
        // Manual control mode
        movement.update(deltaTime, keys, windowSize, landmarks);
    }
    
    // Update sensors
    sensors.update(deltaTime, movement.getPosition(), previousPosition, previousVelocity, 
                  movement.getDirection(), previousDirection, gameTime, movement.getCurrentAcceleration());
    
    
    odometry.update(deltaTime, movement.getPosition(), previousPosition, 
                   movement.getDirection(), sensors.getGyroscopeData(), 
                   sensors.getAccelerometerData());
    // Update line tracers
    sensors.updateLineTracers(movement.getPosition(), movement.getShape().getSize(), 
                             movement.getDirection(), tracerLength, windowSize, getOdometryPosition());
    
    // Update odometry
    // Update landmarks based on odometry position
    
    
    // Update previous values for next frame
    sf::Vector2f currentVelocity = (movement.getPosition() - previousPosition) / deltaTime;
    previousVelocity = currentVelocity;
    previousPosition = movement.getPosition();
    previousDirection = movement.getDirection();
}

void Agent::render(sf::RenderWindow& window) {
    Renderer::renderAgent(window, movement.getShape(), sensors.getLineTracers());
}

void Agent::renderMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, sf::Vector2f minimapSize, sf::Vector2u worldSize,
                       const std::vector<Landmark>& landmarks) {
    Renderer::renderMinimap(window, minimapPosition, minimapSize, worldSize,
                           movement.getPosition(), movement.getShape().getSize(),
                           odometry.getPosition(), sensors.getCurrentGPSPosition(),
                           movement.getDirection(), odometry.getDirection(), 
                           sensors.getGPSData(), sensors.isGPSEnabled(), landmarks,
                           currentPath, currentTarget, currentPathIndex, explorationMode);
}
