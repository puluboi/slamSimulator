#pragma once

#include <SFML/Graphics.hpp>
#include <random>
#include "sensors.hpp"

class Odometry {
public:
    Odometry();
    
    void init(sf::Vector2f startPosition, float startDirection);
    void update(float deltaTime, sf::Vector2f actualPosition, sf::Vector2f previousPosition,
                float actualDirection, const std::vector<GyroscopeData>& gyroData,
                const std::vector<AccelerometerData>& accelData);
    
    // Getters
    sf::Vector2f getPosition() const { return position; }
    float getDirection() const { return direction; }
    sf::Vector2f getVelocity() const { return velocity; }
    
private:
    sf::Vector2f position;
    float direction;
    sf::Vector2f velocity;
    float angVelocity;
    sf::Vector2f acceleration;  // Current acceleration estimate
    
    // Integration state
    sf::Vector2f integratedVelocity;
    sf::Vector2f lastAccelReading;
    float lastUpdateTime;
    
    // Random number generation for wheel noise
    mutable std::mt19937 rng;
    mutable std::normal_distribution<float> wheelNoiseDistribution;
    mutable std::normal_distribution<float> noiseDistribution;
};
