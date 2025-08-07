#pragma once
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <chrono>
#include "sensors.hpp"  // include sensors for accelerometer and gyroscope data structures
#include "ufastslam.hpp" // include ufastslam implementation

// enumeration for slam algorithm types
enum class SLAMAlgorithm {
    EKF_SLAM,      // extended kalman filter slam
    UFASTSLAM      // unscented fastslam with particle filter
};

// enumeration for different types of jacobian matrices
enum class JacobianType {
    STATE_TRANSITION,        // jacobian of motion model f(x,u) with respect to state
    OBSERVATION,            // jacobian of observation model h(x) with respect to state
    OBSERVATION_LANDMARK    // jacobian of observation model with respect to landmark only
};

// this class contains the fundamental building blocks of simultaneous localization and mapping
// supports both ekf-slam and ufastslam algorithms with switchable implementation
// primary role involves converting simulation data to a format suitable for slam algorithms
class SLAM{
private:
    // algorithm selection
    SLAMAlgorithm currentAlgorithm;
    
    // ekf-slam specific members
    Eigen::VectorXd observations;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    
    // state vector structure: [x, y, theta, vx, vy, omega, landmark1_x, landmark1_y, ...]
    int robotStateSize = 6;  // [x, y, theta, vx, vy, omega] - position, orientation, velocities
    int landmarkStateSize = 2; // [x, y] per landmark
    int numLandmarks = 0;
    // store last control input for jacobian calculations
    Eigen::Vector3d lastControlInput; // [ax, ay, alpha] - accelerations
    
    // ufastslam implementation
    UFastSLAM ufastslam;
    
    // computational load tracking for algorithm performance analysis
    mutable std::chrono::high_resolution_clock::time_point algorithmRoundStartTime;
    mutable std::chrono::microseconds totalPredictTime{0};
    mutable std::chrono::microseconds totalUpdateTime{0};
    mutable int updateCount{0};
    mutable bool roundTimingActive{false};

public:
    // constructor and initialization methods
    SLAM(SLAMAlgorithm algorithm = SLAMAlgorithm::EKF_SLAM);
    
    // algorithm switching
    void setAlgorithm(SLAMAlgorithm algorithm);
    SLAMAlgorithm getCurrentAlgorithm() const { return currentAlgorithm; }
    
    // initialize state vector with robot's initial pose estimate
    void initializeState(sf::Vector2f initialPosition, float initialDirection);
    void initializeState(double x, double y, double theta);
    
    // add landmark positions to the state vector for mapping
    void addLandmarkToState(sf::Vector2f landmarkPosition);
    void addLandmarkToState(double x, double y);
    
    // state vector dimension management and pose updates
    void resizeState(int newSize);
    void updateRobotPose(sf::Vector2f position, float direction);
    void updateRobotPose(double x, double y, double theta);
    void updateRobotPoseAndVelocity(sf::Vector2f position, float direction, 
                                  sf::Vector2f velocity, float angularVelocity);
    void updateRobotPoseAndVelocity(double x, double y, double theta, 
                                  double vx, double vy, double omega);
    
    // control input processing for motion model updates
    void updateControlInput(bool keys[4], float currentSpeed, float deltaTime, float currentDirection);
    void updateControlInputWithAcceleration(
        const std::vector<AccelerometerData>& accelData,
        const std::vector<GyroscopeData>& gyroData);
    Eigen::Vector3d getLastControlInput() const { return lastControlInput; }
    
    // accessor methods for state estimation components
    const Eigen::VectorXd& getState() const { return state; }
    const Eigen::VectorXd& getObservations() const { return observations; }
    const Eigen::MatrixXd& getCovariance() const { return covariance; }
    
    // robot pose extraction from state vector
    Eigen::Vector3d getRobotPose() const;
    sf::Vector2f getRobotPosition() const;
    float getRobotDirection() const;
    
    // landmark position extraction from state vector
    std::vector<Eigen::Vector2d> getLandmarkPositions() const;
    Eigen::Vector2d getLandmark(int landmarkIndex) const;
    
    // state vector dimension information
    int getStateSize() const { return state.size(); }
    int getNumLandmarks() const;
    int getRobotStateSize() const { return robotStateSize; }
    
    // utility functions for state management
    void printState() const;
    void reset();
    
    // ufastslam debugging methods
    void printUFastSLAMDetails() const;
    void printUFastSLAMLandmarks() const;
    void printUFastSLAMConvergence() const;
    void debugUFastSLAMParticle(int particleIndex = 0) const;
    
    // state transition model for motion prediction in extended kalman filter
    Eigen::VectorXd stateTransitionModel(const Eigen::VectorXd& currentState, 
                                        const Eigen::Vector3d& controlInput, 
                                        float deltaTime) const;
    
    // observation model for measurement prediction
    Eigen::VectorXd observationModel(const Eigen::VectorXd& state, int landmarkIndex) const;

    Eigen::MatrixXd predictCovariance(
        const Eigen::MatrixXd& stateTransitionJacobian,
        const Eigen::MatrixXd& processNoise) const;

    // process noise matrix construction for motion uncertainty
    Eigen::MatrixXd constructProcessNoiseMatrix(float deltaTime, float accelNoise = 0.1f, 
                                               float gyroNoise = 0.05f, float modelNoise = 0.01f) const;

    // extended kalman filter prediction step
    void ekfPredict(float deltaTime);

    // observation prediction for landmark measurements
    Eigen::VectorXd predictObservation(int landmarkIndex) const;
    std::vector<Eigen::VectorXd> predictAllObservations() const;

    // innovation calculation for measurement updates
    Eigen::VectorXd calculateInnovation(const Eigen::VectorXd& actualObservation, int landmarkIndex) const;
    std::vector<Eigen::VectorXd> calculateAllInnovations(const std::vector<Eigen::VectorXd>& actualObservations) const;

    Eigen::MatrixXd calculateKalmanGain(
        const Eigen::MatrixXd& observationJacobian,
        const Eigen::MatrixXd& observationNoise) const;

    void ekfUpdate(const Eigen::VectorXd& observation, int landmarkIndex);

    void ekfUpdateMultiple(const std::vector<Eigen::VectorXd>& observations,
                           const std::vector<int>& landmarkIds);

    // computational load tracking methods for performance analysis
    void startAlgorithmRound() const;
    void endAlgorithmRound() const;
    void printComputationalLoad(const std::chrono::microseconds& totalRoundTime) const;

    // universal jacobian calculator using numerical differentiation
    template<typename Function>
    Eigen::MatrixXd calculateJacobian(Function func, 
                                     const Eigen::VectorXd& input, 
                                     double epsilon = 1e-6) const;
    
    // convenience wrapper for specific slam jacobian calculations
    Eigen::MatrixXd calculateJacobian(JacobianType type, 
                                     const Eigen::VectorXd& state,
                                     const Eigen::VectorXd& parameters = Eigen::VectorXd(),
                                     int landmarkIndex = -1) const;

private:
    
};