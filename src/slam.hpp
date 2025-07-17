#pragma once
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

// enum for different types of jacobians
enum class JacobianType {
    STATE_TRANSITION,        // jacobian of motion model f(x,u) w.r.t. state
    OBSERVATION,            // jacobian of observation model h(x) w.r.t. state
    OBSERVATION_LANDMARK    // jacobian of observation model w.r.t. landmark only
};

// this class contains the basic building blocks of slam. 
// main role is converting the data from simulation to a format where it's easy to use along the slam algorithms.
class SLAM{
private:
    Eigen::VectorXd observations;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    
    // state vector structure: [x, y, theta, landmark1_x, landmark1_y, landmark2_x, landmark2_y, ...]
    int robotStateSize = 3;  // [x, y, theta]
    int landmarkStateSize = 2; // [x, y] per landmark
    int numLandmarks = 0;

public:
    // constructor and initialization
    SLAM();
    
    // initialize state vector with robot's initial pose
    void initializeState(sf::Vector2f initialPosition, float initialDirection);
    void initializeState(double x, double y, double theta);
    
    // add landmarks to state vector
    void addLandmarkToState(sf::Vector2f landmarkPosition);
    void addLandmarkToState(double x, double y);
    
    // state vector management
    void resizeState(int newSize);
    void updateRobotPose(sf::Vector2f position, float direction);
    void updateRobotPose(double x, double y, double theta);
    
    // getters
    const Eigen::VectorXd& getState() const { return state; }
    const Eigen::VectorXd& getObservations() const { return observations; }
    const Eigen::MatrixXd& getCovariance() const { return covariance; }
    
    // robot pose extraction from state
    Eigen::Vector3d getRobotPose() const;
    sf::Vector2f getRobotPosition() const;
    float getRobotDirection() const;
    
    // landmark extraction from state
    std::vector<Eigen::Vector2d> getLandmarkPositions() const;
    Eigen::Vector2d getLandmark(int landmarkIndex) const;
    
    // state vector size information
    int getStateSize() const { return state.size(); }
    int getNumLandmarks() const { return numLandmarks; }
    int getRobotStateSize() const { return robotStateSize; }
    
    // utility functions
    void printState() const;
    void reset();
    
    // state transition model (motion model) - for ekf to use
    Eigen::VectorXd stateTransitionModel(const Eigen::VectorXd& currentState, 
                                        const Eigen::Vector3d& controlInput, 
                                        float deltaTime) const;
    
    // observation model (measurement model)
    Eigen::VectorXd observationModel(const Eigen::VectorXd& state, int landmarkIndex) const;
    
    // universal jacobian calculator, can compute any jacobian numerically
    template<typename Function>
    Eigen::MatrixXd calculateJacobian(Function func, 
                                     const Eigen::VectorXd& input, 
                                     double epsilon = 1e-6) const;
    
    // convenience wrapper for specific slam jacobians (backward compatibility)
    Eigen::MatrixXd calculateJacobian(JacobianType type, 
                                     const Eigen::VectorXd& state,
                                     const Eigen::VectorXd& parameters = Eigen::VectorXd(),
                                     int landmarkIndex = -1) const;

private:
    
};