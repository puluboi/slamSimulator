#include "agent.hpp"

Agent::Agent() : pathfinder(Pathfinding(800, 600, 20)) { // initialize pathfinder with world size
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
    // initialize all agent components
    movement.init(startPosition);
    odometry.init(startPosition, 0.0f);
    
    // initialize state variables
    previousPosition = startPosition;
    previousDirection = 0.0f;
    currentTarget = startPosition;
    slam.initializeState(startPosition, 0.0f);
}

void Agent::updateState(float intervalTime){
    static size_t previousLandmarkCount = 0;
    const auto& detectedLandmarks = sensors.getdetectedLandmarks();

    // check if new landmarks have been detected
    if (detectedLandmarks.size() > previousLandmarkCount) {
        for (size_t i = previousLandmarkCount; i < detectedLandmarks.size(); ++i) {
            slam.addLandmarkToState(detectedLandmarks[i].getObservedPos());
        }
        previousLandmarkCount = detectedLandmarks.size();
    }
    slam.updateControlInputWithAcceleration(sensors.getAccelerometerData(), sensors.getGyroscopeData());
    slam.ekfPredict(intervalTime);

    std::cout<<"SLAM Predict diagnostics:" << std::endl;
    std::cout<<"  GT: Pos[" << movement.getPosition().x << ", " << movement.getPosition().y 
             << "] Dir=" << movement.getDirection() << "°" << std::endl;
    std::cout<<"  ODOMETRY: Pos[" << odometry.getPosition().x << ", " << getOdometryPosition().y 
             << "] Dir=" << odometry.getDirection() << "°" << std::endl;
    std::cout<<"  SLAM:     Pos[" << slam.getRobotPosition().x << ", " << slam.getRobotPosition().y 
             << "] Dir=" << slam.getRobotDirection() << "°" << std::endl;
    
    // print control inputs for debugging purposes
    if (!sensors.getAccelerometerData().empty() && !sensors.getGyroscopeData().empty()) {
        auto accel = sensors.getAccelerometerData().back();
        auto gyro = sensors.getGyroscopeData().back();
        std::cout<<"  CONTROL:  Accel[" << accel.x << ", " << accel.y 
                 << "] AngVel=" << gyro.z << " rad/s (" << (gyro.z * 180.0 / M_PI) << "°/s)" << std::endl;
    }
    
    float posError = std::sqrt(std::pow(odometry.getPosition().x - slam.getRobotPosition().x, 2) + 
                              std::pow(getOdometryPosition().y - slam.getRobotPosition().y, 2));
    float dirError = std::abs(odometry.getDirection() - slam.getRobotDirection());
    while (dirError > 180.0f) dirError -= 360.0f;
    while (dirError < -180.0f) dirError += 360.0f;
    dirError = std::abs(dirError);
    
    std::cout<<"  ERRORS:   Pos=" << posError << " units, Dir=" << dirError << "°" << std::endl;
}

void Agent::update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                const std::vector<Landmark>& landmarks) {
    // update game time
    gameTime += deltaTime;
    
    
    // update pathfinder with detected landmarks using odometry position
    pathfinder.updateOccupancyGrid(sensors.getdetectedLandmarks(), odometry.getPosition());
    
    // handle exploration mode
    bool explorationKeys[4] = {false, false, false, false};
    if (explorationMode) {
        pathUpdateTimer += deltaTime;
        
        // update path every 0.5 seconds or when current path is completed
        if (pathUpdateTimer > 5.0f || currentPathIndex >= currentPath.size()) {
            // find nearest unexplored area based on odometry position
            sf::Vector2f nearestUnexplored = pathfinder.findNearestUnexplored(odometry.getPosition());
            
            // if we found a new target, plan a path to it
            if (nearestUnexplored != odometry.getPosition()) {
                currentPath = pathfinder.findPath(odometry.getPosition(), nearestUnexplored);
                currentPathIndex = 0;
                currentTarget = nearestUnexplored;
            }
            
            pathUpdateTimer = 0.0f;
        }
        
        // follow the current path
        if (!currentPath.empty() && currentPathIndex < currentPath.size()) {
            sf::Vector2f targetPos = currentPath[currentPathIndex]-sf::Vector2f(2,2);
            sf::Vector2f currentPos = odometry.getPosition(); // use odometry position for navigation
            sf::Vector2f direction = targetPos - currentPos;
            
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            
            // move towards target
            if (distance > 10.0f) { // threshold for reaching waypoint
                // calculate which keys to press to move towards target
                if (direction.y < -5.0f) explorationKeys[0] = true; // up (w)
                if (direction.x < -5.0f) explorationKeys[1] = true; // left (a)
                if (direction.y > 5.0f) explorationKeys[2] = true; // down (s)
                if (direction.x > 5.0f) explorationKeys[3] = true; // right (d)
            } else {
                // reached current waypoint, move to next
                currentPathIndex++;
            }
        }
        
        // use exploration keys instead of manual keys
        movement.update(deltaTime, explorationKeys, windowSize, landmarks);
    } else {
        // manual control mode
        movement.update(deltaTime, keys, windowSize, landmarks);
    }
    
    static float sensorUpdateTimer = 0.0f;
    sensorUpdateTimer += deltaTime;

    if (sensorUpdateTimer >= 0.01f) {
        // Update sensors
        sensors.update(deltaTime, movement.getPosition(), previousPosition, previousVelocity, 
                  movement.getDirection(), previousDirection, gameTime, movement.getCurrentAcceleration());
    
    
        odometry.update(deltaTime, movement.getPosition(), previousPosition, 
                   movement.getDirection(), sensors.getGyroscopeData(), 
                   sensors.getAccelerometerData());
        updateState(sensorUpdateTimer);
        //slam.printState();
        
        sensorUpdateTimer = 0.0f; // Reset the timer
    }
    // Update line tracers
    sensors.updateLineTracers(movement.getPosition(), movement.getShape().getSize(), 
                             movement.getDirection(), tracerLength, windowSize, getOdometryPosition());

    
    
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
                           slam.getRobotPosition(), sensors.getCurrentGPSPosition(),
                           movement.getDirection(), slam.getRobotDirection(), 
                           sensors.getGPSData(), sensors.isGPSEnabled(), landmarks,
                           currentPath, currentTarget, currentPathIndex, explorationMode);
}
