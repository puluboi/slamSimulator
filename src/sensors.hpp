#pragma once

#include <SFML/Graphics.hpp>
#include "landmark.hpp"
#include <vector>
#include <random>

struct AccelerometerData {
    float x, y;  // acceleration in m/s²
    float timestamp;
};

struct GyroscopeData {
    float z;  // angular velocity around the z-axis in rad/s for 2d simulation
    float timestamp;
};

struct GPSData {
    sf::Vector2f position;  // gps position with noise
    float accuracy;         // gps accuracy estimate in meters
    float timestamp;
    bool valid;            // whether gps fix is valid
};

class Sensors {
public:
    Sensors();
    
    void update(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition, 
                sf::Vector2f previousVelocity, float direction, float previousDirection, 
                float gameTime, float currentAcceleration);

    void updateLineTracers(sf::Vector2f agentPosition, sf::Vector2f agentSize,
                           float direction, float tracerLength,
                           sf::Vector2u windowSize, sf::Vector2f position);

    
 
    
    
    
    // getter methods for sensor data
    const std::vector<sf::Vertex>& getLineTracers() const { return lineTracers; }
    const std::vector<AccelerometerData>& getAccelerometerData() const { return accelerometerData; }
    const std::vector<GyroscopeData>& getGyroscopeData() const { return gyroscopeData; }
    const std::vector<GPSData>& getGPSData() const { return gpsData; }
    const std::vector<Landmark>& getdetectedLandmarks() const { return detectedLandmarks; }
    sf::Vector2f getCurrentGPSPosition() const { 
        return gpsEnabled ? currentGPSPosition : sf::Vector2f(0, 0); 
    }
    
    // get average landmark position error
    float getAverageLandmarkError() const { return averageLandmarkError; }
    
    // get individual landmark errors
    const std::vector<float>& getLandmarkErrors() const { return landmarkErrors; }
    
    // get total average error across all detected landmarks
    float getTotalAverageError() const;
    
    // calculate average error across all detected landmarks
    float calculateAverageDetectedLandmarkError() const;
    
    void setLandmarks(const std::vector<Landmark>& newLandmarks){landmarks = newLandmarks;};
    
    // gps control methods
    void setGPSEnabled(bool enabled) { gpsEnabled = enabled; }
    bool isGPSEnabled() const { return gpsEnabled; }
    
private:
    void collectSensorData(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition,
                          sf::Vector2f previousVelocity, float direction, float previousDirection,
                          float gameTime, float currentAcceleration);
    void calculateAcceleration(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition,
                              sf::Vector2f previousVelocity, float gameTime, float currentAcceleration);
    void calculateAngularVelocity(float deltaTime, float direction, float previousDirection,
                                 float gameTime);
    void updateGPS(float deltaTime, sf::Vector2f actualPosition, float gameTime);
    
    // sensor data collections
    std::vector<AccelerometerData> accelerometerData;
    std::vector<GyroscopeData> gyroscopeData;
    std::vector<GPSData> gpsData;
    std::vector<sf::Vertex> lineTracers;
    std::vector<Landmark> detectedLandmarks;
    std::vector<Landmark> landmarks;

    // gps state variables
    sf::Vector2f currentGPSPosition;
    float lastGPSUpdateTime;
    bool gpsEnabled = false; // gps enabled by default
    
    // error tracking for landmarks
    float averageLandmarkError = 0.0f;
    std::vector<float> landmarkErrors; // store individual error lengths
    
    // random number generation for sensor noise
    mutable std::mt19937 rng;
    mutable std::normal_distribution<float> noiseDistribution; // for position
    mutable std::normal_distribution<float> LiDARnoiseDistribution; // for lidar
    mutable std::normal_distribution<float> accelNoiseDistribution; // for accelerometer
    mutable std::normal_distribution<float> gyroNoiseDistribution; // for gyroscope
    mutable std::normal_distribution<float> gpsNoiseDistribution; // for gps
};
