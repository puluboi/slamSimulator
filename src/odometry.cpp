#include "odometry.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

Odometry::Odometry() 
    : rng(std::random_device{}()),
      noiseDistribution(0.0f, 0.2f)       // Higher noise to see bigger EKF improvements
{
    position = sf::Vector2f(0.0f, 0.0f);
    direction = 0.0f;
    velocity = sf::Vector2f(0.0f, 0.0f);
    acceleration = sf::Vector2f(0.0f, 0.0f);
    integratedVelocity = sf::Vector2f(0.0f, 0.0f);
    lastAccelReading = sf::Vector2f(0.0f, 0.0f);
    lastUpdateTime = 0.0f;
}

void Odometry::init(sf::Vector2f startPosition, float startDirection) {
    position = startPosition;
    direction = startDirection;
    velocity = sf::Vector2f(0.0f, 0.0f);
    acceleration = sf::Vector2f(0.0f, 0.0f);
    integratedVelocity = sf::Vector2f(0.0f, 0.0f);
    lastAccelReading = sf::Vector2f(0.0f, 0.0f);
    lastUpdateTime = 0.0f;
}

void Odometry::update(float deltaTime, sf::Vector2f actualPosition, sf::Vector2f previousPosition,
                     float actualDirection, const std::vector<GyroscopeData>& gyroData,
                     const std::vector<AccelerometerData>& accelData) {
    
    if(!accelData.empty()){ // Calculate positional odometry
        auto& latestAccelReading = accelData.back();
        velocity += sf::Vector2f(latestAccelReading.x, latestAccelReading.y) * deltaTime;
        //std::cout<<std::fixed << std::setprecision(3)<<latestAccelReading.x<<": "<<latestAccelReading.y<< std::endl;
        position += velocity * deltaTime;
    }
    if(!gyroData.empty()){
        auto& latestGyroReading = gyroData.back();
        direction += latestGyroReading.z * deltaTime*180/M_PI;
        std::cout<<std::fixed << std::setprecision(3)<<angVelocity<<std::endl;
        
    }
}
