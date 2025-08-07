#pragma once
#include <Eigen/Dense>
#include <SFML/Graphics.hpp>
#include <vector>
#include <random>
#include <chrono>
#include "sensors.hpp"

// structure to represent a landmark with uncertainty in the particle filter
struct LandmarkEstimate {
    Eigen::Vector2d mean;           // landmark position estimate
    Eigen::Matrix2d covariance;     // landmark position uncertainty
    int observationCount;           // number of times this landmark has been observed
    
    LandmarkEstimate() : mean(Eigen::Vector2d::Zero()), 
                        covariance(Eigen::Matrix2d::Identity() * 1000.0), 
                        observationCount(0) {}
    
    LandmarkEstimate(const Eigen::Vector2d& pos) : mean(pos), 
                                                  covariance(Eigen::Matrix2d::Identity() * 0.1),
                                                  observationCount(1) {}
};

// particle structure for ufastslam implementation
struct Particle {
    Eigen::Vector3d pose;                                    // robot pose [x, y, theta]
    Eigen::Vector3d velocity;                               // robot velocity [vx, vy, omega]
    std::vector<LandmarkEstimate> landmarks;                // estimated landmark positions
    double weight;                                          // particle importance weight
    
    Particle() : pose(Eigen::Vector3d::Zero()), 
                velocity(Eigen::Vector3d::Zero()),
                weight(1.0) {}
};

// unscented transform parameters for sigma point generation
struct UnscentedParams {
    double alpha;       // sigma point spread parameter (0.001 <= alpha <= 1)
                        // controls how far sigma points spread from mean
                        // smaller values = points closer to mean, more robust but less accurate
                        // larger values = points further from mean, captures nonlinearities better
    double beta;        // prior knowledge of distribution parameter (beta = 2 optimal for gaussian)
                        // incorporates higher-order moments of distribution  
                        // beta = 2 is optimal for gaussian distributions
    double kappa;       // secondary scaling parameter (typically 0 or 3-n where n=state_dimension)
                        // additional tuning parameter for sigma point placement
                        // kappa = 0 is common choice, kappa = 3-n ensures positive semi-definite covariance
    
    UnscentedParams() : alpha(0.001), beta(2.0), kappa(0.0) {}  // conservative default values for stability
};

// ufastslam implementation using particle filter with unscented transform
class UFastSLAM {
private:
    std::vector<Particle> particles;               // particle set for robot trajectory
    int numParticles;                              // number of particles in filter
    std::mt19937 randomGenerator;                  // random number generator for sampling
    std::normal_distribution<double> normalDist;   // gaussian noise distribution
    UnscentedParams utParams;                      // unscented transform parameters
    
    // algorithm parameters
    double motionNoiseStd;                         // motion model noise standard deviation
    double measurementNoiseStd;                    // measurement model noise standard deviation
    double resamplingThreshold;                    // effective sample size threshold for resampling
    
    // computational performance tracking
    mutable std::chrono::high_resolution_clock::time_point algorithmStartTime;
    mutable std::chrono::microseconds totalPredictTime{0};
    mutable std::chrono::microseconds totalUpdateTime{0};
    mutable int updateCount{0};
    
    // motion model for particle prediction using unscented transform
    Eigen::Vector3d motionModel(const Eigen::Vector3d& pose, 
                               const Eigen::Vector3d& velocity,
                               const Eigen::Vector3d& controlInput, 
                               float deltaTime) const;
    
    // observation model for range-bearing measurements
    Eigen::Vector2d observationModel(const Eigen::Vector3d& robotPose, 
                                   const Eigen::Vector2d& landmarkPos) const;
    
    // unscented transform functions for nonlinear uncertainty propagation
    std::vector<Eigen::VectorXd> generateSigmaPoints(const Eigen::VectorXd& mean, 
                                                     const Eigen::MatrixXd& covariance) const;
    
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> unscentedTransform(
        const std::vector<Eigen::VectorXd>& sigmaPoints,
        const std::vector<double>& weights) const;
    
    // particle filter operations
    void predictParticles(const Eigen::Vector3d& controlInput, float deltaTime);
    void updateParticleWeights(const std::vector<std::pair<Eigen::VectorXd, int>>& observations);
    void resampleParticles();
    double calculateEffectiveSampleSize() const;
    
    // landmark management for each particle
    void updateLandmarkEstimate(Particle& particle, int landmarkIndex, 
                               const Eigen::Vector2d& observation);
    void addNewLandmark(Particle& particle, const Eigen::Vector2d& observation);
    
    // data association for landmark observations
    int associateLandmark(const Particle& particle, const Eigen::Vector2d& observation) const;
    double mahalanobisDistance(const Eigen::Vector2d& innovation, 
                              const Eigen::Matrix2d& covariance) const;

public:
    // constructor and initialization
    UFastSLAM(int numParticles = 100);
    
    // initialize particles with robot's starting pose
    void initializeParticles(sf::Vector2f initialPosition, float initialDirection);
    void initializeParticles(double x, double y, double theta);
    
    // main ufastslam algorithm steps
    void predict(const Eigen::Vector3d& controlInput, float deltaTime);
    void update(const std::vector<std::pair<Eigen::VectorXd, int>>& observations);
    
    // state estimation from particle set
    Eigen::Vector3d getRobotPose() const;
    sf::Vector2f getRobotPosition() const;
    float getRobotDirection() const;
    std::vector<Eigen::Vector2d> getLandmarkPositions() const;
    
    // particle filter status and diagnostics
    int getNumParticles() const { return numParticles; }
    int getNumLandmarks() const;
    double getParticleSpread() const;
    
    // computational performance analysis
    void startAlgorithmRound() const;
    void endAlgorithmRound() const;
    void printComputationalLoad(const std::chrono::microseconds& totalRoundTime) const;
    
    // debugging and diagnostic methods
    void printDetailedParticleInfo(int particleIndex = 0) const;
    void printLandmarkInfo() const;
    void printConvergenceMetrics() const;
    void debugMotionModel(const Eigen::Vector3d& controlInput, float deltaTime) const;
    void debugObservationModel(int particleIndex, int landmarkIndex) const;
    
    // algorithm parameter tuning
    void setMotionNoise(double noise) { motionNoiseStd = noise; }
    void setMeasurementNoise(double noise) { measurementNoiseStd = noise; }
    void setResamplingThreshold(double threshold) { resamplingThreshold = threshold; }
    
    // utility functions
    void printParticleStats() const;
    void reset();
};
