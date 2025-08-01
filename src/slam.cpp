#include "slam.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

SLAM::SLAM() {
    // initialize empty state vector for robot pose estimation
    state = Eigen::VectorXd::Zero(robotStateSize);
    observations = Eigen::VectorXd::Zero(0);
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    numLandmarks = 0;
    lastControlInput = Eigen::Vector3d::Zero();
}   

void SLAM::initializeState(sf::Vector2f initialPosition, float initialDirection) {
    // convert sfml types to doubles and call the main initialization function
    initializeState(static_cast<double>(initialPosition.x), 
                   static_cast<double>(initialPosition.y), 
                   static_cast<double>(initialDirection * M_PI / 180.0)); // convert degrees to radians
}

void SLAM::initializeState(double x, double y, double theta) {
    // initialize robot state vector with pose and velocity components
    state = Eigen::VectorXd::Zero(robotStateSize);
    state(0) = x;     // robot x position in world coordinates
    state(1) = y;     // robot y position in world coordinates
    state(2) = theta; // robot orientation in radians
    state(3) = 0.0;   // robot x velocity component
    state(4) = 0.0;   // robot y velocity component
    state(5) = 0.0;   // robot angular velocity
    
    // initialize covariance matrix for robot state uncertainty
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    // set initial uncertainty values for pose estimation
    covariance(0, 0) = 0.1; // x position uncertainty
    covariance(1, 1) = 0.1; // y position uncertainty
    covariance(2, 2) = 0.05; // orientation uncertainty in radians
    covariance(3, 3) = 0.5; // x velocity uncertainty
    covariance(4, 4) = 0.5; // y velocity uncertainty
    covariance(5, 5) = 0.1; // angular velocity uncertainty
    
    numLandmarks = 0;
    
    std::cout << "SLAM state initialized with robot pose: [" 
              << x << ", " << y << ", " << theta << "]" << std::endl;
}

void SLAM::addLandmarkToState(sf::Vector2f landmarkPosition) {
    addLandmarkToState(static_cast<double>(landmarkPosition.x), 
                      static_cast<double>(landmarkPosition.y));
}

void SLAM::addLandmarkToState(double x, double y) {
    // resize state vector to accommodate new landmark position
    int newStateSize = robotStateSize + (numLandmarks + 1) * landmarkStateSize;
    Eigen::VectorXd newState = Eigen::VectorXd::Zero(newStateSize);
    
    // copy existing state information to new vector
    newState.head(state.size()) = state;
    
    // add new landmark coordinates at the end of state vector
    int landmarkStartIndex = robotStateSize + numLandmarks * landmarkStateSize;
    newState(landmarkStartIndex) = x;
    newState(landmarkStartIndex + 1) = y;
    
    // update state vector with expanded dimensions
    state = newState;
    
    // resize covariance matrix for new landmark uncertainty
    Eigen::MatrixXd newCovariance = Eigen::MatrixXd::Zero(newStateSize, newStateSize);
    newCovariance.topLeftCorner(covariance.rows(), covariance.cols()) = covariance;
    
    // set initial uncertainty for new landmark position
    newCovariance(landmarkStartIndex, landmarkStartIndex) = 10.0;     // x uncertainty
    newCovariance(landmarkStartIndex + 1, landmarkStartIndex + 1) = 10.0; // y uncertainty
    
    covariance = newCovariance;
    numLandmarks++;
    
    std::cout << "Added landmark " << numLandmarks << " at position: [" 
              << x << ", " << y << "]" << std::endl;
}

void SLAM::resizeState(int newSize) {
    if (newSize > state.size()) {
        Eigen::VectorXd newState = Eigen::VectorXd::Zero(newSize);
        newState.head(state.size()) = state;
        state = newState;
        
        Eigen::MatrixXd newCovariance = Eigen::MatrixXd::Zero(newSize, newSize);
        newCovariance.topLeftCorner(covariance.rows(), covariance.cols()) = covariance;
        covariance = newCovariance;
    }
}

void SLAM::updateRobotPose(sf::Vector2f position, float direction) {
    updateRobotPose(static_cast<double>(position.x), 
                   static_cast<double>(position.y), 
                   static_cast<double>(direction * M_PI / 180.0));
}

void SLAM::updateRobotPose(double x, double y, double theta) {
    if (state.size() >= robotStateSize) {
        state(0) = x;
        state(1) = y;
        state(2) = theta;
    }
}

void SLAM::updateRobotPoseAndVelocity(sf::Vector2f position, float direction, 
                                     sf::Vector2f velocity, float angularVelocity) {
    updateRobotPoseAndVelocity(static_cast<double>(position.x), 
                              static_cast<double>(position.y), 
                              static_cast<double>(direction * M_PI / 180.0), // convert to radians
                              static_cast<double>(velocity.x),
                              static_cast<double>(velocity.y),
                              static_cast<double>(angularVelocity * M_PI / 180.0)); // convert to radians
}

void SLAM::updateRobotPoseAndVelocity(double x, double y, double theta, 
                                     double vx, double vy, double omega) {
    if (state.size() >= robotStateSize) {
        state(0) = x;     // position x coordinate
        state(1) = y;     // position y coordinate
        state(2) = theta; // orientation in radians
        state(3) = vx;    // velocity x component
        state(4) = vy;    // velocity y component
        state(5) = omega; // angular velocity in radians per second
    }
}

void SLAM::updateControlInput(bool keys[4], float currentSpeed, float deltaTime, float currentDirection) {
    // calculate control input based on key presses and movement parameters
    // this represents the commanded motion for the robot
    
    float deltaX = 0.0f, deltaY = 0.0f, deltaTheta = 0.0f;
    
    // calculate linear motion commands from keyboard input
    if (keys[0]) deltaY -= currentSpeed * deltaTime; // w key - move up
    if (keys[1]) deltaX -= currentSpeed * deltaTime; // a key - move left  
    if (keys[2]) deltaY += currentSpeed * deltaTime; // s key - move down
    if (keys[3]) deltaX += currentSpeed * deltaTime; // d key - move right
    
    // calculate angular motion from direction changes
    // in a real robot this would come from steering commands
    static float previousDirection = currentDirection;
    deltaTheta = (currentDirection - previousDirection) * M_PI / 180.0f; // convert to radians
    previousDirection = currentDirection;
    
    // store the control input for motion model
    lastControlInput = Eigen::Vector3d(deltaX, deltaY, deltaTheta);
}

void SLAM::updateControlInputWithAcceleration(const std::vector<AccelerometerData>& accelData, 
                                             const std::vector<GyroscopeData>& gyroData) {
    // use actual sensor data for acceleration-based control input
    float ax = 0.0f, ay = 0.0f, alpha = 0.0f;
    
    // get latest accelerometer reading for linear motion
    if (!accelData.empty()) {
        const auto& latestAccel = accelData.back();
        ax = latestAccel.x;  // linear acceleration in x direction
        ay = latestAccel.y;  // linear acceleration in y direction
    }
    
    // get latest gyroscope reading for angular acceleration
    if (!gyroData.empty()) {
        const auto& latestGyro = gyroData.back();
        // for angular acceleration, we differentiate angular velocity
        // for now, use the angular velocity directly as a proxy
        static float previousOmega = 0.0f;
        float currentOmega = latestGyro.z;
        alpha = (currentOmega - previousOmega)*180/M_PI; // angular acceleration in rad/s²
        previousOmega = currentOmega;
    }
    
    // store acceleration as control input for motion model
    lastControlInput = Eigen::Vector3d(ax, ay, alpha);
    
    // debug output for acceleration values
    //std::cout << "acceleration control: [ax=" << ax << ", ay=" << ay << ", alpha=" << alpha << "]" << std::endl;
}

Eigen::Vector3d SLAM::getRobotPose() const {
    if (state.size() >= robotStateSize) {
        return Eigen::Vector3d(state(0), state(1), state(2));
    }
    return Eigen::Vector3d::Zero();
}

sf::Vector2f SLAM::getRobotPosition() const {
    if (state.size() >= 2) {
        return sf::Vector2f(static_cast<float>(state(0)), static_cast<float>(state(1)));
    }
    return sf::Vector2f(0.0f, 0.0f);
}

float SLAM::getRobotDirection() const {
    if (state.size() >= 3) {
        return static_cast<float>(state(2) * 180.0 / M_PI); // convert radians to degrees
    }
    return 0.0f;
}

std::vector<Eigen::Vector2d> SLAM::getLandmarkPositions() const {
    std::vector<Eigen::Vector2d> landmarks;
    
    for (int i = 0; i < numLandmarks; i++) {
        int landmarkIndex = robotStateSize + i * landmarkStateSize;
        if (landmarkIndex + 1 < state.size()) {
            landmarks.push_back(Eigen::Vector2d(state(landmarkIndex), state(landmarkIndex + 1)));
        }
    }
    
    return landmarks;
}

Eigen::Vector2d SLAM::getLandmark(int landmarkIndex) const {
    if (landmarkIndex >= 0 && landmarkIndex < numLandmarks) {
        int stateIndex = robotStateSize + landmarkIndex * landmarkStateSize;
        if (stateIndex + 1 < state.size()) {
            return Eigen::Vector2d(state(stateIndex), state(stateIndex + 1));
        }
    }
    return Eigen::Vector2d::Zero();
}

void SLAM::printState() const {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "SLAM State Vector (size: " << state.size() << "):" << std::endl;
    
    // print robot pose and velocity
    if (state.size() >= robotStateSize) {
        std::cout << "  Robot pose: [" 
                  << state(0) << ", " << state(1) << ", " << state(2) << "]" << std::endl;
        std::cout << "  Robot velocity: [" 
                  << state(3) << ", " << state(4) << ", " << state(5) << "]" << std::endl;
    }
    
    // print landmarks
    /*
    std::cout << "  Landmarks (" << numLandmarks << "):" << std::endl;
    for (int i = 0; i < numLandmarks; i++) {
        int landmarkIndex = robotStateSize + i * landmarkStateSize;
        if (landmarkIndex + 1 < state.size()) {
            std::cout << "    Landmark " << i << ": [" 
                      << state(landmarkIndex) << ", " << state(landmarkIndex + 1) << "]" << std::endl;
        }
    }*/
    
    // print the jacobians
    //std::cout << "  Jacobians:" << std::endl;

    // Example: Print state transition Jacobian
    /*Eigen::VectorXd dummyState = state;
    Eigen::VectorXd actualControl(3);
    actualControl << lastControlInput(0), lastControlInput(1), lastControlInput(2); // Use actual control input
    Eigen::MatrixXd stateTransitionJacobian = calculateJacobian(
        JacobianType::STATE_TRANSITION, dummyState, actualControl, -1);
    std::cout << "    State Transition Jacobian (6x6 Robot dynamics):" << std::endl;
    std::cout << stateTransitionJacobian.block(0,0,6,6) << std::endl;
    
    // Print control input for reference
    std::cout << "    Control Input (accelerations): [" 
              << lastControlInput(0) << ", " << lastControlInput(1) << ", " << lastControlInput(2) << "]" << std::endl;
    */
    // Example: Print observation Jacobians for each landmark
    /*for (int i = 0; i < numLandmarks; i++) {
        Eigen::MatrixXd observationJacobian = calculateJacobian(
            JacobianType::OBSERVATION, state, Eigen::VectorXd(), i);
        std::cout << "    Observation Jacobian for Landmark " << i << ":" << std::endl;
        std::cout << observationJacobian << std::endl;
    }*/
}

void SLAM::reset() {
    state = Eigen::VectorXd::Zero(robotStateSize);
    observations = Eigen::VectorXd::Zero(0);
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    numLandmarks = 0;
    
    std::cout << "SLAM state reset to initial conditions." << std::endl;
}

// state transition model f(x, u) - predicts next state given current state and control input
Eigen::VectorXd SLAM::stateTransitionModel(const Eigen::VectorXd& currentState, 
                                          const Eigen::Vector3d& controlInput, 
                                          float deltaTime) const {
    // Motion model should match actual robot odometry
    Eigen::VectorXd nextState = currentState;
    
    if (currentState.size() >= robotStateSize) {
        double x = currentState(0);
        double y = currentState(1);
        double theta = currentState(2);
        double vx = currentState(3);     // linear velocity
        double vy = currentState(4);     // linear velocity
        double omega = currentState(5);  // angular velocity
        
        double dt = deltaTime;
        
        // Use velocity-based model to match odometry
        // This should match exactly how the robot actually moves
        nextState(0) = x + vx * dt;  // position x
        nextState(1) = y + vy * dt;  // position y  
        nextState(2) = theta + omega * dt;  // orientation
        
        // Velocity updates from control input (accelerations)
        nextState(3) = vx + controlInput(0) * dt;  // velocity x
        nextState(4) = vy + controlInput(1) * dt;  // velocity y
        nextState(5) = omega + controlInput(2) * dt;  // angular velocity
        
        // Normalize angle
        while (nextState(2) > M_PI) nextState(2) -= 2.0 * M_PI;
        while (nextState(2) < -M_PI) nextState(2) += 2.0 * M_PI;
    }
    
    return nextState;
}

// universal jacobian calculator using numerical differentiation
template<typename Function>
Eigen::MatrixXd SLAM::calculateJacobian(Function func, 
                                        const Eigen::VectorXd& input, 
                                        double epsilon) const {
    // Get function output size by calling the function once
    Eigen::VectorXd f0 = func(input);
    int output_size = f0.size();
    int input_size = input.size();
    
    // Initialize Jacobian matrix
    Eigen::MatrixXd jacobian(output_size, input_size);
    
    // Calculate partial derivatives using finite differences
    for (int j = 0; j < input_size; j++) {
        // Create perturbed input vectors
        Eigen::VectorXd input_plus = input;
        Eigen::VectorXd input_minus = input;
        
        input_plus(j) += epsilon;
        input_minus(j) -= epsilon;
        
        // Calculate function values at perturbed points
        Eigen::VectorXd f_plus = func(input_plus);
        Eigen::VectorXd f_minus = func(input_minus);
        
        // Compute partial derivative using central difference
        jacobian.col(j) = (f_plus - f_minus) / (2.0 * epsilon);
    }
    
    return jacobian;
}

// Convenience wrapper for backward compatibility with enum-based interface
Eigen::MatrixXd SLAM::calculateJacobian(JacobianType type, 
                                        const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& parameters,
                                        int landmarkIndex) const {
    switch (type) {
        case JacobianType::STATE_TRANSITION: {
            // Create lambda for state transition function
            auto stateTransitionFunc = [this, &parameters](const Eigen::VectorXd& x) -> Eigen::VectorXd {
                if (parameters.size() >= 3) {
                    Eigen::Vector3d control(parameters(0), parameters(1), parameters(2));
                    return this->stateTransitionModel(x, control, 0.1f); // default deltaTime
                }
                return x; // Identity if no control input
            };
            return calculateJacobian(stateTransitionFunc, state);
        }
        
        case JacobianType::OBSERVATION: {
            // Create lambda for observation function
            auto observationFunc = [this, landmarkIndex](const Eigen::VectorXd& x) -> Eigen::VectorXd {
                return this->observationModel(x, landmarkIndex);
            };
            return calculateJacobian(observationFunc, state);
        }
        
        case JacobianType::OBSERVATION_LANDMARK: {
            // Extract just the landmark part for Jacobian
            if (landmarkIndex >= 0 && landmarkIndex < numLandmarks) {
                int landmarkStart = robotStateSize + landmarkIndex * landmarkStateSize;
                Eigen::VectorXd landmarkState = state.segment(landmarkStart, landmarkStateSize);
                
                auto landmarkObsFunc = [this, &state, landmarkIndex](const Eigen::VectorXd& lm_pos) -> Eigen::VectorXd {
                    // Create modified state with new landmark position
                    Eigen::VectorXd modified_state = state;
                    int landmarkStart = robotStateSize + landmarkIndex * landmarkStateSize;
                    modified_state.segment(landmarkStart, landmarkStateSize) = lm_pos;
                    return this->observationModel(modified_state, landmarkIndex);
                };
                return calculateJacobian(landmarkObsFunc, landmarkState);
            }
            return Eigen::MatrixXd::Zero(2, 2);
        }
        
        default:
            std::cerr << "Unknown Jacobian type!" << std::endl;
            return Eigen::MatrixXd::Identity(state.size(), state.size());
    }
}

// Observation model for range-bearing measurements
Eigen::VectorXd SLAM::observationModel(const Eigen::VectorXd& state, int landmarkIndex) const {
    if (state.size() < robotStateSize || landmarkIndex < 0 || landmarkIndex >= numLandmarks) {
        return Eigen::VectorXd::Zero(2); // [range, bearing]
    }
    
    // Extract robot pose
    double robot_x = state(0);
    double robot_y = state(1);
    double robot_theta = state(2);
    
    // Extract landmark position
    int landmarkStart = robotStateSize + landmarkIndex * landmarkStateSize;
    double landmark_x = state(landmarkStart);
    double landmark_y = state(landmarkStart + 1);
    
    // Calculate relative position
    double dx = landmark_x - robot_x;
    double dy = landmark_y - robot_y;
    
    // Calculate range and bearing
    double range = std::sqrt(dx * dx + dy * dy);
    double bearing = std::atan2(dy, dx) - robot_theta;
    
    // Normalize bearing to [-π, π]
    while (bearing > M_PI) bearing -= 2.0 * M_PI;
    while (bearing < -M_PI) bearing += 2.0 * M_PI;
    
    Eigen::VectorXd observation(2);
    observation(0) = range;
    observation(1) = bearing;
    return observation;
}
// Predict covariance matrix using the state transition Jacobian and process noise
Eigen::MatrixXd SLAM::predictCovariance(const Eigen::MatrixXd& stateTransitionJacobian, 
                                        const Eigen::MatrixXd& processNoise) const {
    // P_(k|k-1) = F_x * P_(k-1) * F_x^T + Q_k
    return stateTransitionJacobian * covariance * stateTransitionJacobian.transpose() + processNoise;
}

Eigen::MatrixXd SLAM::constructProcessNoiseMatrix(float deltaTime, float accelNoise, 
                                                 float gyroNoise, float modelNoise) const {
    int stateSize = state.size();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(stateSize, stateSize);
    
    // Much smaller noise values to prevent instability
    float dt = deltaTime;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    // Reduce noise by orders of magnitude
    float posNoise = 0.01f;      // 1cm position uncertainty
    float velNoise = 0.1f;       // 10cm/s velocity uncertainty  
    float angleNoise = 0.001f;   // ~0.06 degree uncertainty
    float angVelNoise = 0.01f;   // 0.01 rad/s uncertainty
    
    // Position-velocity coupling for robot state (first 6 elements)
    // Position uncertainty from velocity
    Q(0,0) = (dt4/4.0f) * velNoise + dt2 * posNoise;  // x
    Q(1,1) = (dt4/4.0f) * velNoise + dt2 * posNoise;  // y  
    Q(2,2) = (dt4/4.0f) * angVelNoise + dt2 * angleNoise; // θ
    
    // Velocity uncertainty
    Q(3,3) = dt2 * velNoise;     // vx
    Q(4,4) = dt2 * velNoise;     // vy
    Q(5,5) = dt2 * angVelNoise;  // ω
    
    // Cross-correlations (position-velocity coupling)
    Q(0,3) = Q(3,0) = (dt3/2.0f) * velNoise;  // x-vx
    Q(1,4) = Q(4,1) = (dt3/2.0f) * velNoise;  // y-vy  
    Q(2,5) = Q(5,2) = (dt3/2.0f) * angVelNoise; // θ-ω
    
    // Landmark positions: very small process noise (they don't move)
    for (int i = 6; i < stateSize; i++) {
        Q(i,i) = 0.0001f;  // 0.1mm landmark drift
    }
    
    return Q;
}

void SLAM::ekfPredict(float deltaTime) {
    // Store previous state for debugging
    Eigen::VectorXd previousState = state;
    
    // Step 1: Predict new state using motion model
    state = stateTransitionModel(state, lastControlInput, deltaTime);
    
    // Step 2: Calculate state transition Jacobian
    auto stateTransitionFunc = [this, deltaTime](const Eigen::VectorXd& s) -> Eigen::VectorXd {
        return stateTransitionModel(s, lastControlInput, deltaTime);
    };
    Eigen::MatrixXd F = calculateJacobian(stateTransitionFunc, previousState);
    
    // Step 3: Construct process noise matrix
    Eigen::MatrixXd Q = constructProcessNoiseMatrix(deltaTime);
    
    // Step 4: Update covariance using your predictCovariance function
    covariance = predictCovariance(F, Q);
    
    // Debug output (optional)
    static int debugCounter = 0;
    if (++debugCounter % 50 == 0) {
        std::cout << "EKF Predict - State change: [" 
                  << (state.head(6) - previousState.head(6)).transpose() << "]" << std::endl;
        std::cout << "Covariance trace: " << covariance.trace() << std::endl;
    }
}

Eigen::VectorXd SLAM::predictObservation(int landmarkIndex) const {
    // Predict what the observation should be for a given landmark
    // ẑ_(k|k-1) = h(X̂_(k|k-1), k)
    // Note: We don't add observation noise v_k here - that's for simulation
    
    if (landmarkIndex < 0 || landmarkIndex >= numLandmarks) {
        std::cerr << "Invalid landmark index: " << landmarkIndex << std::endl;
        return Eigen::VectorXd::Zero(2);
    }
    
    // Use the observation model to predict range-bearing measurement
    Eigen::VectorXd predictedObservation = observationModel(state, landmarkIndex);
    
    return predictedObservation;  // [predicted_range, predicted_bearing]
}

std::vector<Eigen::VectorXd> SLAM::predictAllObservations() const {
    // Predict observations for all landmarks in the state
    std::vector<Eigen::VectorXd> predictions;
    predictions.reserve(numLandmarks);
    
    for (int i = 0; i < numLandmarks; i++) {
        predictions.push_back(predictObservation(i));
    }
    
    return predictions;
}

Eigen::VectorXd SLAM::calculateInnovation(const Eigen::VectorXd& actualObservation, int landmarkIndex) const {
    // Calculate innovation: z̃_k = z_k - ẑ_(k|k-1)
    
    if (landmarkIndex < 0 || landmarkIndex >= numLandmarks) {
        std::cerr << "Invalid landmark index for innovation: " << landmarkIndex << std::endl;
        return Eigen::VectorXd::Zero(2);
    }
    
    if (actualObservation.size() != 2) {
        std::cerr << "Invalid observation size for innovation. Expected 2 (range, bearing), got " 
                  << actualObservation.size() << std::endl;
        return Eigen::VectorXd::Zero(2);
    }
    
    // Get predicted observation
    Eigen::VectorXd predictedObservation = predictObservation(landmarkIndex);
    
    // Calculate innovation
    Eigen::VectorXd innovation = actualObservation - predictedObservation;
    
    // Important: Normalize bearing innovation to [-π, π]
    // This handles the circular nature of angles
    while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
    while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;
    
    return innovation;  // [range_innovation, bearing_innovation]
}

std::vector<Eigen::VectorXd> SLAM::calculateAllInnovations(const std::vector<Eigen::VectorXd>& actualObservations) const {
    // Calculate innovations for all landmarks
    std::vector<Eigen::VectorXd> innovations;
    innovations.reserve(std::min(static_cast<size_t>(numLandmarks), actualObservations.size()));
    
    size_t maxObservations = std::min(static_cast<size_t>(numLandmarks), actualObservations.size());
    
    for (size_t i = 0; i < maxObservations; i++) {
        innovations.push_back(calculateInnovation(actualObservations[i], static_cast<int>(i)));
    }
    
    return innovations;
}
Eigen::MatrixXd SLAM::calculateKalmanGain(const Eigen::MatrixXd& observationJacobian, 
                                          const Eigen::MatrixXd& observationNoise) const {
    // K_k = P_(k|k-1) H_k^T (H_k P_(k|k-1) H_k^T + R_k)^(-1)
    Eigen::MatrixXd PHt = covariance * observationJacobian.transpose();
    Eigen::MatrixXd S = observationJacobian * PHt + observationNoise; // Innovation covariance
    Eigen::MatrixXd K = PHt * S.inverse(); // Kalman gain
    return K;
}
void SLAM::ekfUpdate(const Eigen::VectorXd& observation, int landmarkIndex) {
    // Validate inputs
    if (landmarkIndex < 0 || landmarkIndex >= numLandmarks) {
        std::cerr << "Invalid landmark index for EKF update: " << landmarkIndex << std::endl;
        return;
    }
    
    if (observation.size() != 2) {
        std::cerr << "Invalid observation size. Expected 2 (range, bearing), got " 
                  << observation.size() << std::endl;
        return;
    }
    
    // Step 1: Calculate innovation
    Eigen::VectorXd innovation = calculateInnovation(observation, landmarkIndex);
    
    // Step 2: Calculate observation Jacobian
    Eigen::MatrixXd H = calculateJacobian(JacobianType::OBSERVATION, state, Eigen::VectorXd(), landmarkIndex);
    
    // Step 3: Construct observation noise matrix
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
    R(0, 0) = 0.1;   // range measurement noise variance (10cm std dev)
    R(1, 1) = 0.01;  // bearing measurement noise variance (~5.7 degree std dev)
    
    // Step 4: Calculate Kalman gain
    Eigen::MatrixXd K = calculateKalmanGain(H, R);
    
    // Step 5: Update state estimate
    // X̂_(k|k) = X̂_(k|k-1) + K_k z̃_(k|k-1)
    state = state + K * innovation;
    
    // Step 6: Update covariance matrix
    // P_(k|k) = (I - K_k H_k) P_(k|k-1)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
    covariance = (I - K * H) * covariance;
    
    // Normalize robot orientation to [-π, π]
    if (state.size() >= 3) {
        while (state(2) > M_PI) state(2) -= 2.0 * M_PI;
        while (state(2) < -M_PI) state(2) += 2.0 * M_PI;
    }
}

void SLAM::ekfUpdateMultiple(const std::vector<Eigen::VectorXd>& observations) {
    // Update with multiple observations sequentially
    size_t maxUpdates = std::min(static_cast<size_t>(numLandmarks), observations.size());
    
    for (size_t i = 0; i < maxUpdates; i++) {
        ekfUpdate(observations[i], static_cast<int>(i));
    }
}