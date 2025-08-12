#include "agent.hpp"
#include <cmath>

Agent::Agent() : pathfinder(Pathfinding(800, 600, 20)) { // initialize pathfinder with world size
    gameTime = 0.0f;
    tracerLength = 300.0f; // increase sensor range to re-observe distant landmarks
    previousPosition = sf::Vector2f(0.0f, 0.0f);
    previousVelocity = sf::Vector2f(0.0f, 0.0f);
    previousDirection = 0.0f;
    explorationMode = true;
    currentPathIndex = 0;
    currentTarget = sf::Vector2f(0.0f, 0.0f);
    pathUpdateTimer = 0.0f;
    slam.setAlgorithm(SLAMAlgorithm::UFASTSLAM);
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
    const auto& currentlyVisibleLandmarks = sensors.getCurrentlyVisibleLandmarks();

    // check if new landmarks have been detected (use full detected list for state expansion)
    if (detectedLandmarks.size() > previousLandmarkCount) {
        for (size_t i = previousLandmarkCount; i < detectedLandmarks.size(); ++i) {
            int landmarkId = detectedLandmarks[i].getId();
            
            if (slam.getCurrentAlgorithm() == SLAMAlgorithm::EKF_SLAM) {
                int slamIndex = slam.getNumLandmarks(); // next available slam index
                slam.addLandmarkToState(detectedLandmarks[i].getObservedPos());
                landmarkIdToSlamIndex[landmarkId] = slamIndex; // store mapping
                
                std::cout << "mapped landmark id " << landmarkId << " to slam index " << slamIndex << std::endl;
            } else if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
                // for ufastslam, use a simple sequential mapping since landmarks are added dynamically
                landmarkIdToSlamIndex[landmarkId] = landmarkId; // direct mapping for ufastslam
                std::cout << "mapped landmark id " << landmarkId << " for ufastslam" << std::endl;
            }
        }
        previousLandmarkCount = detectedLandmarks.size();
    }
    slam.updateControlInputWithAcceleration(sensors.getAccelerometerData(), sensors.getGyroscopeData());
    slam.ekfPredict(intervalTime);
    
    // convert currently visible landmark cartesian coordinates to range-bearing observations
    // use proper landmark id mapping for slam updates
    std::vector<std::pair<Eigen::VectorXd, int>> landmarkObservationPairs;
    sf::Vector2f robotPos = slam.getRobotPosition();
    float robotDir = slam.getRobotDirection() * M_PI / 180.0f; // convert degrees to radians
    
    for (const auto& landmark : currentlyVisibleLandmarks) {
        // calculate relative position from robot to landmark
        sf::Vector2f landmarkPos = landmark.getObservedPos();
        float dx = landmarkPos.x - robotPos.x;
        float dy = landmarkPos.y - robotPos.y;
        
        // compute range and bearing in robot frame
        float range = std::sqrt(dx * dx + dy * dy);
        float absoluteBearing = std::atan2(dy, dx);
        float relativeBearing = absoluteBearing - robotDir;
        
        // normalize bearing to [-π, π] for consistent angular representation
        while (relativeBearing > M_PI) relativeBearing -= 2.0 * M_PI;
        while (relativeBearing < -M_PI) relativeBearing += 2.0 * M_PI;
        
        // create range-bearing observation vector
        Eigen::VectorXd observation(2);
        observation(0) = range;
        observation(1) = relativeBearing;
        
        // find the slam index for this landmark id using proper mapping
        int landmarkId = landmark.getId();
        auto it = landmarkIdToSlamIndex.find(landmarkId);
        if (it != landmarkIdToSlamIndex.end()) {
            int slamLandmarkIndex = it->second;
            
            // different validation for different algorithms
            if (slam.getCurrentAlgorithm() == SLAMAlgorithm::EKF_SLAM) {
                if (slamLandmarkIndex >= 0 && slamLandmarkIndex < slam.getNumLandmarks()) {
                    landmarkObservationPairs.push_back(std::make_pair(observation, slamLandmarkIndex));
                }
            } else if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
                // for ufastslam, landmarks are created dynamically, so just use the mapping
                landmarkObservationPairs.push_back(std::make_pair(observation, slamLandmarkIndex));
            }
        } else {
            std::cout << "warning: landmark id " << landmarkId << " not found in slam mapping" << std::endl;
        }
    }
    
    // apply updates for detected landmarks using their proper ids
    if (!landmarkObservationPairs.empty()) {
        if (slam.getCurrentAlgorithm() == SLAMAlgorithm::EKF_SLAM) {
            // ekf-slam: process observations individually
            for (const auto& obsPair : landmarkObservationPairs) {
                slam.ekfUpdate(obsPair.first, obsPair.second);
            }
        } else if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
            // ufastslam: process all observations together
            std::vector<Eigen::VectorXd> observations;
            std::vector<int> landmarkIds;
            
            for (const auto& obsPair : landmarkObservationPairs) {
                observations.push_back(obsPair.first);
                landmarkIds.push_back(obsPair.second);
            }
            
            slam.ekfUpdateMultiple(observations, landmarkIds);
        }
    }
    
    // debug landmark observations
    std::cout << "DEBUG: " << detectedLandmarks.size() << " total landmarks, " 
              << currentlyVisibleLandmarks.size() << " currently visible, "
              << landmarkObservationPairs.size() << " valid observations, " 
              << slam.getNumLandmarks() << " landmarks in slam state" << std::endl;
    
    // print algorithm-specific diagnostics
    if (slam.getCurrentAlgorithm() == SLAMAlgorithm::EKF_SLAM) {
        std::cout<<"SLAM Predict diagnostics:" << std::endl;
        std::cout<<"  GT: Pos[" << movement.getPosition().x << ", " << movement.getPosition().y 
                 << "] Dir=" << movement.getDirection() << "°" << std::endl;
        std::cout<<"  ODOMETRY: Pos[" << odometry.getPosition().x << ", " << getOdometryPosition().y 
                 << "] Dir=" << odometry.getDirection() << "°" << std::endl;
        std::cout<<"  SLAM:     Pos[" << slam.getRobotPosition().x << ", " << slam.getRobotPosition().y 
                 << "] Dir=" << slam.getRobotDirection() << "°" << std::endl;
    } else if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
        // ufastslam specific debugging
        static int ufastDebugCounter = 0;
        if (++ufastDebugCounter % 50 == 0) { // print every 50th update
            std::cout << "=== ufastslam debug round " << ufastDebugCounter << " ===" << std::endl;
            std::cout << "  GT: Pos[" << movement.getPosition().x << ", " << movement.getPosition().y 
                     << "] Dir=" << movement.getDirection() << "°" << std::endl;
            std::cout << "  ODOMETRY: Pos[" << odometry.getPosition().x << ", " << getOdometryPosition().y 
                     << "] Dir=" << odometry.getDirection() << "°" << std::endl;
            std::cout << "  UFASTSLAM: Pos[" << slam.getRobotPosition().x << ", " << slam.getRobotPosition().y 
                     << "] Dir=" << slam.getRobotDirection() << "°" << std::endl;
            
            // detailed ufastslam analysis every 100th update
            if (ufastDebugCounter % 100 == 0) {
                slam.printUFastSLAMDetails();
            }
        }
    }
    
    // add ufastslam specific debugging
    if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
        static int ufastDebugCounter = 0;
        if (++ufastDebugCounter % 200 == 0) { // less frequent debug output
            std::cout << "=== ufastslam debug snapshot ===" << std::endl;
            slam.printUFastSLAMDetails();
        }
        
        // periodic landmark analysis
        if (ufastDebugCounter % 500 == 0) {
            slam.printUFastSLAMLandmarks();
        }
        
        // convergence analysis
        if (ufastDebugCounter % 1000 == 0) {
            slam.printUFastSLAMConvergence();
        }
    }
    
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
    
    // compare odometry and slam errors against ground truth
    float slamPosError = std::sqrt(std::pow(movement.getPosition().x - slam.getRobotPosition().x, 2) + 
                                  std::pow(movement.getPosition().y - slam.getRobotPosition().y, 2));
    float slamDirError = std::abs(movement.getDirection() - slam.getRobotDirection());
    while (slamDirError > 180.0f) slamDirError -= 360.0f;
    while (slamDirError < -180.0f) slamDirError += 360.0f;
    slamDirError = std::abs(slamDirError);
    
    float odometryPosError = std::sqrt(std::pow(movement.getPosition().x - odometry.getPosition().x, 2) + 
                                      std::pow(movement.getPosition().y - odometry.getPosition().y, 2));
    float odometryDirError = std::abs(movement.getDirection() - odometry.getDirection());
    while (odometryDirError > 180.0f) odometryDirError -= 360.0f;
    while (odometryDirError < -180.0f) odometryDirError += 360.0f;
    odometryDirError = std::abs(odometryDirError);
    
    std::cout<<"  ERROR COMPARISON:" << std::endl;
    std::cout<<"    odometry: pos=" << odometryPosError << " units, dir=" << odometryDirError << "°" << std::endl;
    std::cout<<"    slam:     pos=" << slamPosError << " units, dir=" << slamDirError << "°" << std::endl;
    std::cout<<"    slam improvement: pos=" << (odometryPosError - slamPosError) << " units, dir=" 
             << (odometryDirError - slamDirError) << "°" << std::endl;
}

void Agent::debugCurrentSLAM() const {
    std::cout << "=== agent slam debug ===" << std::endl;
    std::cout << "current slam algorithm: " 
              << (slam.getCurrentAlgorithm() == SLAMAlgorithm::EKF_SLAM ? "ekf-slam" : "ufastslam") << std::endl;
    
    slam.printState();
    
    if (slam.getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
        slam.printUFastSLAMDetails();
    }
    
    std::cout << "========================" << std::endl;
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
                           currentPath, currentTarget, currentPathIndex, explorationMode,
                           &slam);  // Pass SLAM pointer for particle access
}

float Agent::getSLAMPositionError() const {
    // calculate position error between true position and slam estimate
    sf::Vector2f truePos = movement.getPosition();
    sf::Vector2f slamPos = slam.getRobotPosition();
    return std::sqrt(std::pow(truePos.x - slamPos.x, 2) + std::pow(truePos.y - slamPos.y, 2));
}

float Agent::getOdometryPositionError() const {
    // calculate position error between true position and odometry estimate
    sf::Vector2f truePos = movement.getPosition();
    sf::Vector2f odoPos = odometry.getPosition();
    return std::sqrt(std::pow(truePos.x - odoPos.x, 2) + std::pow(truePos.y - odoPos.y, 2));
}

double Agent::getSLAMUpdateTime() const {
    // return average slam update time in microseconds
    return slam.getAverageUpdateTime();
}
