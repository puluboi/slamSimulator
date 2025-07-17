#include "slam.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

SLAM::SLAM() {
    // initialize empty state vector
    state = Eigen::VectorXd::Zero(robotStateSize);
    observations = Eigen::VectorXd::Zero(0);
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    numLandmarks = 0;
}   

void SLAM::initializeState(sf::Vector2f initialPosition, float initialDirection) {
    // convert sfml types to doubles and call the main initialization
    initializeState(static_cast<double>(initialPosition.x), 
                   static_cast<double>(initialPosition.y), 
                   static_cast<double>(initialDirection * M_PI / 180.0)); // convert degrees to radians
}

void SLAM::initializeState(double x, double y, double theta) {
    // initialize robot state vector [x, y, theta]
    state = Eigen::VectorXd::Zero(robotStateSize);
    state(0) = x;     // robot x position
    state(1) = y;     // robot y position
    state(2) = theta; // robot orientation in radians
    
    // initialize covariance matrix for robot state only
    covariance = Eigen::MatrixXd::Identity(robotStateSize, robotStateSize);
    // set initial uncertainty (small for initial position)
    covariance(0, 0) = 0.1; // x position uncertainty
    covariance(1, 1) = 0.1; // y position uncertainty
    covariance(2, 2) = 0.05; // orientation uncertainty (radians)
    
    numLandmarks = 0;
    
    std::cout << "SLAM state initialized with robot pose: [" 
              << x << ", " << y << ", " << theta << "]" << std::endl;
}

void SLAM::addLandmarkToState(sf::Vector2f landmarkPosition) {
    addLandmarkToState(static_cast<double>(landmarkPosition.x), 
                      static_cast<double>(landmarkPosition.y));
}

void SLAM::addLandmarkToState(double x, double y) {
    // resize state vector to accommodate new landmark
    int newStateSize = robotStateSize + (numLandmarks + 1) * landmarkStateSize;
    Eigen::VectorXd newState = Eigen::VectorXd::Zero(newStateSize);
    
    // copy existing state
    newState.head(state.size()) = state;
    
    // add new landmark at the end
    int landmarkStartIndex = robotStateSize + numLandmarks * landmarkStateSize;
    newState(landmarkStartIndex) = x;
    newState(landmarkStartIndex + 1) = y;
    
    // update state vector
    state = newState;
    
    // resize covariance matrix
    Eigen::MatrixXd newCovariance = Eigen::MatrixXd::Zero(newStateSize, newStateSize);
    newCovariance.topLeftCorner(covariance.rows(), covariance.cols()) = covariance;
    
    // set initial uncertainty for new landmark
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
        return static_cast<float>(state(2) * 180.0 / M_PI); // Convert radians to degrees
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
    
    // print robot pose
    if (state.size() >= robotStateSize) {
        std::cout << "  Robot pose: [" 
                  << state(0) << ", " << state(1) << ", " << state(2) << "]" << std::endl;
    }
    
    // print landmarks
    std::cout << "  Landmarks (" << numLandmarks << "):" << std::endl;
    for (int i = 0; i < numLandmarks; i++) {
        int landmarkIndex = robotStateSize + i * landmarkStateSize;
        if (landmarkIndex + 1 < state.size()) {
            std::cout << "    Landmark " << i << ": [" 
                      << state(landmarkIndex) << ", " << state(landmarkIndex + 1) << "]" << std::endl;
        }
    }
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
    // control input: [delta_x, delta_y, delta_theta] from odometry
    // state structure: [robot_x, robot_y, robot_theta, landmark1_x, landmark1_y, ...]
    
    Eigen::VectorXd nextState = currentState;
    
    if (currentState.size() >= robotStateSize) {
        // extract current robot pose
        double x = currentState(0);
        double y = currentState(1);
        double theta = currentState(2);
        
        // extract control inputs
        double delta_x = controlInput(0);
        double delta_y = controlInput(1);
        double delta_theta = controlInput(2);
        
        // motion model: simple odometry integration
        // transform control input from robot frame to world frame
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        // update robot pose (first 3 elements of state vector)
        nextState(0) = x + (delta_x * cos_theta - delta_y * sin_theta);
        nextState(1) = y + (delta_x * sin_theta + delta_y * cos_theta);
        nextState(2) = theta + delta_theta;
        
        // normalize angle to [-π, π]
        while (nextState(2) > M_PI) nextState(2) -= 2.0 * M_PI;
        while (nextState(2) < -M_PI) nextState(2) += 2.0 * M_PI;
        
        // landmarks remain stationary (no change to landmark positions)
        // this is the key assumption in slam - landmarks don't move
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
