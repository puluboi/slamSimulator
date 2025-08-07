#include "slam.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

SLAM::SLAM(SLAMAlgorithm algorithm) : currentAlgorithm(algorithm), ufastslam(100) {
    // initialize empty state vector for robot pose estimation
    state = Eigen::VectorXd::Zero(robotStateSize);
    observations = Eigen::VectorXd::Zero(0);
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    numLandmarks = 0;
    lastControlInput = Eigen::Vector3d::Zero();
    
    std::cout << "slam initialized with algorithm: " 
              << (algorithm == SLAMAlgorithm::EKF_SLAM ? "ekf-slam" : "ufastslam") << std::endl;
}   

void SLAM::setAlgorithm(SLAMAlgorithm algorithm) {
    currentAlgorithm = algorithm;
    std::cout << "===== slam algorithm switched to: " 
              << (algorithm == SLAMAlgorithm::EKF_SLAM ? "ekf-slam" : "ufastslam") 
              << " =====" << std::endl;
    
    // add initialization message for ufastslam
    if (algorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "ufastslam debugging enabled - expect detailed output every 25-50 iterations" << std::endl;
        std::cout << "particle filter initialized with " << ufastslam.getNumParticles() 
                  << " particles" << std::endl;
    }
}

void SLAM::initializeState(sf::Vector2f initialPosition, float initialDirection) {
    // convert sfml types to doubles and call the main initialization function
    initializeState(static_cast<double>(initialPosition.x), 
                   static_cast<double>(initialPosition.y), 
                   static_cast<double>(initialDirection * M_PI / 180.0)); // convert degrees to radians
}

void SLAM::initializeState(double x, double y, double theta) {
    // initialize both algorithms
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
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
        covariance(0, 0) = 0.01; // x position uncertainty
        covariance(1, 1) = 0.01; // y position uncertainty
        covariance(2, 2) = 0.005; // orientation uncertainty in radians
        covariance(3, 3) = 0.1; // x velocity uncertainty
        covariance(4, 4) = 0.1; // y velocity uncertainty
        covariance(5, 5) = 0.05; // angular velocity uncertainty
        
        numLandmarks = 0;
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        // initialize ufastslam particles
        ufastslam.initializeParticles(x, y, theta);
    }
    
    std::cout << "slam state initialized with robot pose: [" 
              << x << ", " << y << ", " << theta << "]" << std::endl;
}

void SLAM::addLandmarkToState(sf::Vector2f landmarkPosition) {
    addLandmarkToState(static_cast<double>(landmarkPosition.x), 
                      static_cast<double>(landmarkPosition.y));
}

void SLAM::addLandmarkToState(double x, double y) {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
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
        
        // set initial uncertainty for new landmark position (much smaller to prevent filter corruption)
        newCovariance(landmarkStartIndex, landmarkStartIndex) = 0.001;     // x uncertainty 
        newCovariance(landmarkStartIndex + 1, landmarkStartIndex + 1) = 0.001; // y uncertainty
        
        covariance = newCovariance;
        numLandmarks++;
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        // ufastslam handles landmark addition dynamically during updates
        // just track the count for consistency
        numLandmarks++;
    }
    
    std::cout << "added landmark " << numLandmarks << " at position: [" 
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
    
    // get latest gyroscope reading for angular velocity (not acceleration)
    if (!gyroData.empty()) {
        const auto& latestGyro = gyroData.back();
        // use angular velocity directly to match odometry implementation
        alpha = latestGyro.z; // rad/s (angular velocity, not acceleration)
    }
    
    // store acceleration as control input for motion model
    lastControlInput = Eigen::Vector3d(ax, ay, alpha);
    
    // debug output for acceleration values
    static int debugCounter = 0;
    if (++debugCounter % 100 == 0) {
        std::cout << "control input debug: [ax=" << ax << ", ay=" << ay 
                  << ", alpha=" << alpha << " rad/s²]" << std::endl;
    }
}

Eigen::Vector3d SLAM::getRobotPose() const {
    if (state.size() >= robotStateSize) {
        return Eigen::Vector3d(state(0), state(1), state(2));
    }
    return Eigen::Vector3d::Zero();
}

sf::Vector2f SLAM::getRobotPosition() const {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        if (state.size() >= 2) {
            return sf::Vector2f(static_cast<float>(state(0)), static_cast<float>(state(1)));
        }
        return sf::Vector2f(0.0f, 0.0f);
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        return ufastslam.getRobotPosition();
    }
    return sf::Vector2f(0.0f, 0.0f);
}

float SLAM::getRobotDirection() const {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        if (state.size() >= 3) {
            return static_cast<float>(state(2) * 180.0 / M_PI); // convert radians to degrees
        }
        return 0.0f;
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        return ufastslam.getRobotDirection();
    }
    return 0.0f;
}

std::vector<Eigen::Vector2d> SLAM::getLandmarkPositions() const {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        std::vector<Eigen::Vector2d> landmarks;
        
        for (int i = 0; i < numLandmarks; i++) {
            int landmarkIndex = robotStateSize + i * landmarkStateSize;
            if (landmarkIndex + 1 < state.size()) {
                landmarks.push_back(Eigen::Vector2d(state(landmarkIndex), state(landmarkIndex + 1)));
            }
        }
        
        return landmarks;
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        return ufastslam.getLandmarkPositions();
    }
    return {};
}

Eigen::Vector2d SLAM::getLandmark(int landmarkIndex) const {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        if (landmarkIndex >= 0 && landmarkIndex < numLandmarks) {
            int stateIndex = robotStateSize + landmarkIndex * landmarkStateSize;
            if (stateIndex + 1 < state.size()) {
                return Eigen::Vector2d(state(stateIndex), state(stateIndex + 1));
            }
        }
        return Eigen::Vector2d::Zero();
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        auto landmarks = ufastslam.getLandmarkPositions();
        if (landmarkIndex >= 0 && landmarkIndex < static_cast<int>(landmarks.size())) {
            return landmarks[landmarkIndex];
        }
        return Eigen::Vector2d::Zero();
    }
    return Eigen::Vector2d::Zero();
}

int SLAM::getNumLandmarks() const { 
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        return numLandmarks; 
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        return ufastslam.getNumLandmarks();
    }
    return 0;
}

void SLAM::printState() const {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "EKF-SLAM State Vector (size: " << state.size() << "):" << std::endl;
        
        // print robot pose and velocity
        if (state.size() >= robotStateSize) {
            std::cout << "  Robot pose: [" 
                      << state(0) << ", " << state(1) << ", " << state(2) << "]" << std::endl;
            std::cout << "  Robot velocity: [" 
                      << state(3) << ", " << state(4) << ", " << state(5) << "]" << std::endl;
        }
        
        // debug output can be enabled by uncommenting the sections below
        /*
        std::cout << "  Landmarks (" << numLandmarks << "):" << std::endl;
        for (int i = 0; i < numLandmarks; i++) {
            int landmarkIndex = robotStateSize + i * landmarkStateSize;
            if (landmarkIndex + 1 < state.size()) {
                std::cout << "    Landmark " << i << ": [" 
                          << state(landmarkIndex) << ", " << state(landmarkIndex + 1) << "]" << std::endl;
            }
        }
        */
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "UFastSLAM Algorithm State:" << std::endl;
        ufastslam.printParticleStats();
        
        // additional ufastslam debugging (uncomment as needed)
        // ufastslam.printLandmarkInfo();
        // ufastslam.printConvergenceMetrics();
        // ufastslam.printDetailedParticleInfo(0); // show first particle details
    }
}

void SLAM::reset() {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        state = Eigen::VectorXd::Zero(robotStateSize);
        observations = Eigen::VectorXd::Zero(0);
        covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
        numLandmarks = 0;
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        ufastslam.reset();
        numLandmarks = 0; // reset the count for consistency
    }
    
    std::cout << "slam state reset to initial conditions." << std::endl;
}

void SLAM::printUFastSLAMDetails() const {
    if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "=== ufastslam detailed analysis ===" << std::endl;
        ufastslam.printDetailedParticleInfo(0); // show first particle
        std::cout << std::endl;
        ufastslam.printConvergenceMetrics();
        std::cout << "=================================" << std::endl;
    } else {
        std::cout << "current algorithm is not ufastslam" << std::endl;
    }
}

void SLAM::printUFastSLAMLandmarks() const {
    if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "=== ufastslam landmark analysis ===" << std::endl;
        ufastslam.printLandmarkInfo();
        std::cout << "===================================" << std::endl;
    } else {
        std::cout << "current algorithm is not ufastslam" << std::endl;
    }
}

void SLAM::printUFastSLAMConvergence() const {
    if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "=== ufastslam convergence analysis ===" << std::endl;
        ufastslam.printConvergenceMetrics();
        std::cout << "=====================================" << std::endl;
    } else {
        std::cout << "current algorithm is not ufastslam" << std::endl;
    }
}

void SLAM::debugUFastSLAMParticle(int particleIndex) const {
    if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        std::cout << "=== ufastslam particle debug ===" << std::endl;
        ufastslam.printDetailedParticleInfo(particleIndex);
        
        // debug motion model with last control input
        if (lastControlInput.norm() > 0) {
            std::cout << std::endl << "motion model debug with last control input:" << std::endl;
            ufastslam.debugMotionModel(lastControlInput, 0.1f); // use 0.1s as example
        }
        std::cout << "===============================" << std::endl;
    } else {
        std::cout << "current algorithm is not ufastslam" << std::endl;
    }
}

// state transition model f(x, u) - predicts next state given current state and control input
Eigen::VectorXd SLAM::stateTransitionModel(const Eigen::VectorXd& currentState, 
                                          const Eigen::Vector3d& controlInput, 
                                          float deltaTime) const {
    // Motion model should match actual robot odometry
    Eigen::VectorXd nextState = currentState;
    
    if (currentState.size() >= robotStateSize) {
        float x = currentState(0);
        float y = currentState(1);
        float theta = currentState(2);
        float vx = currentState(3);     // linear velocity
        float vy = currentState(4);     // linear velocity
        float omega = currentState(5);  // angular velocity
        
        float dt = deltaTime;
        
        // Use velocity-based model to match odometry
        // This should match exactly how the robot actually moves
        nextState(0) = x + vx * dt;  // position x
        nextState(1) = y + vy * dt;  // position y  
        nextState(2) = theta + omega * dt;  // orientation
        
        // velocity updates from control input (accelerations for linear, angular velocity for rotational)
        nextState(3) = vx + controlInput(0) * dt;  // velocity x from acceleration
        nextState(4) = vy + controlInput(1) * dt;  // velocity y from acceleration
        nextState(5) = controlInput(2);  // angular velocity directly matches current sensor reading
        
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
    
    // balance process noise for trustworthy observations (10cm sensor noise)
    float posNoise = 0.1f;      // 30cm position uncertainty - larger than sensor noise
    float velNoise = 0.5f;      // 70cm/s velocity uncertainty - allow sensor corrections
    float angleNoise = 0.001f;   // ~3 degree uncertainty - reasonable for odometry
    float angVelNoise = 0.005f;   // 0.1 rad/s uncertainty - moderate angular velocity error
    
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
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
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
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        // ufastslam prediction step
        ufastslam.predict(lastControlInput, deltaTime);
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
    
    // calculate innovation
    Eigen::VectorXd innovation = actualObservation - predictedObservation;
    
    // important: normalize bearing innovation to [-π, π] to handle angular discontinuity  
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
    // debug output for kalman gain analysis
    static int gainDebugCounter = 0;
    if (++gainDebugCounter % 20 == 0) {  // print every 20th gain calculation
        std::cout << "debug kalman gain - innovation covariance s determinant: " 
                  << S.determinant() << std::endl;
        std::cout << "debug kalman gain - gain matrix norm: " 
                  << K.norm() << std::endl;
        std::cout << "debug kalman gain - observation jacobian conditioning: " 
                  << observationJacobian.norm() << std::endl;
        
        // check for numerical stability issues
        if (S.determinant() < 1e-12) {
            std::cout << "warning: innovation covariance matrix is nearly singular" << std::endl;
        }
        
        if (K.norm() > 100.0) {
            std::cout << "warning: kalman gain matrix has large norm, potential instability" << std::endl;
        }
    }
    return K;
}
void SLAM::ekfUpdate(const Eigen::VectorXd& observation, int landmarkIndex) {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
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
        
        // debug: print innovation values (reduced output)
        static int updateCounter = 0;
        if (++updateCounter % 10 == 0) {  // print every 10th update
            std::cout << "DEBUG EKF Update - Landmark " << landmarkIndex 
                      << ": Innovation=[" << innovation(0) << ", " << innovation(1) 
                      << "] (range, bearing in rad)" << std::endl;
        }
        
        // innovation gating - reject observations with excessive errors to prevent filter corruption
        double rangeInnovationThreshold = 10;   // 50cm range threshold for outlier rejection
        double bearingInnovationThreshold = 1; // ~5.7 degree bearing threshold for outlier rejection
        
        /*if (std::abs(innovation(0)) > rangeInnovationThreshold || 
            std::abs(innovation(1)) > bearingInnovationThreshold) {
            std::cout << "DEBUG: REJECTING Landmark " << landmarkIndex 
                      << " - innovation too large (range=" << innovation(0) 
                      << ", bearing=" << innovation(1) << ")" << std::endl;
            return; // skip this update
        }*/
        innovation(1) = 0; // STOPGAP
        // step 2: calculate observation jacobian matrix for linearization of measurement model
        Eigen::MatrixXd H = calculateJacobian(JacobianType::OBSERVATION, state, Eigen::VectorXd(), landmarkIndex);
        
        // debug output: display observation jacobian for analysis (reduced output)
        if (updateCounter % 50 == 0) {  // print every 50th update
            std::cout << "debug ekf update - observation jacobian for landmark " << landmarkIndex 
                      << " (size " << H.rows() << "x" << H.cols() << "):" << std::endl;
            std::cout << H << std::endl;
        }
        
        // Step 3: match observation noise to actual sensor accuracy (10cm lidar noise)
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
        R(0, 0) = 0.01;  // 10cm range measurement noise variance (matches actual sensor)
        R(1, 1) = 0.001; // ~1.8 degree bearing noise variance (reasonable for 10cm at typical ranges)
        
        // Step 4: Calculate Kalman gain
        Eigen::MatrixXd K = calculateKalmanGain(H, R);
        
        // debug output: display kalman gain matrix for analysis of filter performance
        if (updateCounter % 50 == 0) {  // print every 50th update
            std::cout << "debug ekf update - kalman gain for landmark " << landmarkIndex 
                      << " (size " << K.rows() << "x" << K.cols() << "):" << std::endl;
            std::cout << K << std::endl;
            
            // diagnostic: check robot covariance conditioning
            std::cout << "robot pose covariance diagonal: [" 
                      << covariance(0,0) << ", " << covariance(1,1) << ", " << covariance(2,2) << "]" << std::endl;
            std::cout << "robot-landmark cross-covariance sample: " 
                      << covariance(0, robotStateSize + landmarkIndex*2) << std::endl;
        }
        
        // Step 5: Update state estimate
       // debug output: display the state update vector for analysis of ekf correction
        Eigen::VectorXd stateUpdate = K * innovation;
        if (updateCounter % 50 == 0) {  // print every 50th update
            std::cout << "debug ekf update - state correction (k*innovation) for landmark " << landmarkIndex 
                  << ":" << std::endl;
        std::cout << stateUpdate.transpose() << std::endl;
        }
        
        
        
        state = state + stateUpdate;
        
        // Step 6: Update covariance matrix using numerically stable Joseph form
        // P_(k|k) = (I - K_k H_k) P_(k|k-1) (I - K_k H_k)^T + K_k R_k K_k^T
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
        Eigen::MatrixXd A = I - K * H;
        covariance = A * covariance * A.transpose() + K * R * K.transpose();
        
        // Normalize robot orientation to [-π, π]
        if (state.size() >= 3) {
            while (state(2) > M_PI) state(2) -= 2.0 * M_PI;
            while (state(2) < -M_PI) state(2) += 2.0 * M_PI;
        }
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        // ufastslam handles individual updates differently
        // convert single observation to vector format
        std::vector<std::pair<Eigen::VectorXd, int>> observations;
        observations.push_back(std::make_pair(observation, landmarkIndex));
        ufastslam.update(observations);
    }
}

void SLAM::ekfUpdateMultiple(const std::vector<Eigen::VectorXd>& observations, 
                            const std::vector<int>& landmarkIds) {
    if (currentAlgorithm == SLAMAlgorithm::EKF_SLAM) {
        // validate input sizes match
        if (observations.size() != landmarkIds.size()) {
            std::cerr << "observation count must match landmark id count" << std::endl;
            return;
        }
        
        // validate input observations match expected range-bearing format
        for (const auto& obs : observations) {
            if (obs.size() != 2) {
                std::cerr << "invalid observation format in ekfUpdateMultiple. expected range-bearing [2], got " 
                          << obs.size() << std::endl;
                return;
            }
        }
        
        // process observations with correct landmark association
        for (size_t i = 0; i < observations.size(); i++) {
            int landmarkId = landmarkIds[i];
            
            // validate landmark id is within bounds
            if (landmarkId < 0 || landmarkId >= numLandmarks) {
                std::cerr << "invalid landmark id: " << landmarkId << std::endl;
                continue;
            }
            
            // normalize bearing angle before update to ensure consistent angular representation
            Eigen::VectorXd normalizedObs = observations[i];
            double bearing = normalizedObs(1);
            
            // constrain bearing to [-π, π] range for numerical stability
            while (bearing > M_PI) bearing -= 2.0 * M_PI;
            while (bearing < -M_PI) bearing += 2.0 * M_PI;
            normalizedObs(1) = bearing;
            
            // apply ekf update step for the correct landmarka
            ekfUpdate(normalizedObs, landmarkId);  //  landmark association
        }
    } else if (currentAlgorithm == SLAMAlgorithm::UFASTSLAM) {
        // prepare observation pairs for ufastslam
        std::vector<std::pair<Eigen::VectorXd, int>> observationPairs;
        for (size_t i = 0; i < observations.size(); i++) {
            if (i < landmarkIds.size()) {
                observationPairs.push_back(std::make_pair(observations[i], landmarkIds[i]));
            }
        }
        ufastslam.update(observationPairs);
    }
}