#pragma once

#include <SFML/Graphics.hpp>
#include "landmark.hpp"
#include <vector>
#include <random>

struct AccelerometerData {
    float x, y;  // Acceleration in m/s²
    float timestamp;
};

struct GyroscopeData {
    float z;  // Angular velocity around the Z-axis in rad/s (for 2D simulation)
    float timestamp;
};

struct GPSData {
    sf::Vector2f position;  // GPS position with noise
    float accuracy;         // GPS accuracy estimate in meters
    float timestamp;
    bool valid;            // Whether GPS fix is valid
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

    
 
    
    
    
    // Getters
    const std::vector<sf::Vertex>& getLineTracers() const { return lineTracers; }
    const std::vector<AccelerometerData>& getAccelerometerData() const { return accelerometerData; }
    const std::vector<GyroscopeData>& getGyroscopeData() const { return gyroscopeData; }
    const std::vector<GPSData>& getGPSData() const { return gpsData; }
    const std::vector<Landmark>& getdetectedLandmarks() const { return detectedLandmarks; }
    sf::Vector2f getCurrentGPSPosition() const { 
        return gpsEnabled ? currentGPSPosition : sf::Vector2f(0, 0); 
    }
    
    // Get average landmark position error
    float getAverageLandmarkError() const { return averageLandmarkError; }
    
    // Get individual landmark errors
    const std::vector<float>& getLandmarkErrors() const { return landmarkErrors; }
    
    // Get total average error across all detected landmarks
    float getTotalAverageError() const;
    
    // Calculate average error across all detected landmarks
    float calculateAverageDetectedLandmarkError() const;
    
    void setLandmarks(const std::vector<Landmark>& newLandmarks){landmarks = newLandmarks;};
    
    // GPS control
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
    
    // Sensor data
    std::vector<AccelerometerData> accelerometerData;
    std::vector<GyroscopeData> gyroscopeData;
    std::vector<GPSData> gpsData;
    std::vector<sf::Vertex> lineTracers;
    std::vector<Landmark> detectedLandmarks;
    std::vector<Landmark> landmarks;

    // GPS state
    sf::Vector2f currentGPSPosition;
    float lastGPSUpdateTime;
    bool gpsEnabled = false; // GPS enabled by default
    
    // Error tracking
    float averageLandmarkError = 0.0f;
    std::vector<float> landmarkErrors; // Store individual error lengths
    
    // Random number generation for noise
    mutable std::mt19937 rng;
    mutable std::normal_distribution<float> noiseDistribution; // For position
    mutable std::normal_distribution<float> LiDARnoiseDistribution; // For LiDAR
    mutable std::normal_distribution<float> accelNoiseDistribution; // For accelerometer
    mutable std::normal_distribution<float> gyroNoiseDistribution; // For gyroscope
    mutable std::normal_distribution<float> gpsNoiseDistribution; // For GPS
};
