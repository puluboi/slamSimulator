#include "sensors.hpp"
#include <cmath>
#include <random>
#include <iostream>

Sensors::Sensors() 
    : rng(std::random_device{}()),
      noiseDistribution(0.0f, 0.f), 
      LiDARnoiseDistribution(0.0f, 0.1f), // lidar noise: ~1.5cm std dev
      accelNoiseDistribution(0.0f, 0.f), // accelerometer noise: ~0.01 m/s² std dev
      gyroNoiseDistribution(0.0f, 0.1f), // gyroscope noise: ~0.0001 rad/s std dev
      gpsNoiseDistribution(0.0f, 3.0f) // gps noise: ~3m std dev (typical consumer gps)
{
    lineTracers.resize(6); // 10 vertices for 5 lines
    lastGPSUpdateTime = 0.0f;
    currentGPSPosition = sf::Vector2f(0.0f, 0.0f);
}

void Sensors::update(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition, 
                    sf::Vector2f previousVelocity, float direction, float previousDirection, 
                    float gameTime, float currentAcceleration) {
    collectSensorData(deltaTime, position, previousPosition, previousVelocity, 
                     direction, previousDirection, gameTime, currentAcceleration);
    
    // only update gps if enabled
    if (gpsEnabled) {
        updateGPS(deltaTime, position, gameTime);
    }
    
}

void Sensors::updateLineTracers(sf::Vector2f agentPosition, sf::Vector2f agentSize, 
                               float direction, float tracerLength, sf::Vector2u windowSize, sf::Vector2f position) {
    // calculate agent center
    sf::Vector2f agentCenter = agentPosition + sf::Vector2f(agentSize.x / 2, agentSize.y / 2);
    
    // reset error calculation for this frame
    float totalError = 0.0f;
    int errorCount = 0;
    landmarkErrors.clear(); // clear previous frame's errors
    
    // create 10 lines spreading from -30 to +30 degrees from agent direction
    for (int i = 0; i < 3; i++) {
        float angleOffset = -30.0f + (i * 30.f); // -30, -15, 0, 15, 30 degrees
        float totalAngle = direction + angleOffset;
        
        // convert to radians
        float radians = totalAngle * 3.14159f / 180.0f;
        
        // calculate end point (no obstacles, so just extend to full range)
        sf::Vector2f endPoint;
        endPoint.x = agentCenter.x + cos(radians) * tracerLength;
        endPoint.y = agentCenter.y + sin(radians) * tracerLength;
        
        // Add noise to simulate LiDAR readings
        float noise = noiseDistribution(rng);
        sf::Vector2f direction_vec = endPoint - agentCenter;
        float length = sqrt(direction_vec.x * direction_vec.x + direction_vec.y * direction_vec.y);
        if (length > 0) {
            direction_vec /= length;
            endPoint = agentCenter + direction_vec * (length + noise);
        }
        
        // Ray casting for landmark detection
        sf::Vector2f rayStart = agentCenter;
        sf::Vector2f rayDir = endPoint - rayStart;
        float rayLen = std::sqrt(rayDir.x*rayDir.x + rayDir.y*rayDir.y);
        if(rayLen > 0) rayDir /= rayLen;

        float min_dist = rayLen;
        bool hit = false;
        Landmark* closestLandmark = nullptr;
        sf::Vector2f closestHitPoint;

        for (auto& landmark : landmarks) {
            sf::FloatRect lb = landmark.getShape().getGlobalBounds();
            float tmin = 0, tmax = min_dist;

            // Intersect with vertical slabs
            if (std::abs(rayDir.x) < 1e-6) {
                if (rayStart.x < lb.left || rayStart.x > lb.left + lb.width) continue;
            } else {
                float t1 = (lb.left - rayStart.x) / rayDir.x;
                float t2 = (lb.left + lb.width - rayStart.x) / rayDir.x;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax) continue;
            }

            // Intersect with horizontal slabs
            if (std::abs(rayDir.y) < 1e-6) {
                if (rayStart.y < lb.top || rayStart.y > lb.top + lb.height) continue;
            } else {
                float t1 = (lb.top - rayStart.y) / rayDir.y;
                float t2 = (lb.top + lb.height - rayStart.y) / rayDir.y;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax) continue;
            }

            // Check if this landmark is closer than any previous hit
            if (tmin < min_dist) {
                min_dist = tmin;
                closestHitPoint = rayStart + rayDir * min_dist;
                closestLandmark = &landmark;
                hit = true;
            }
        }
        
        // Only process the closest landmark hit (if any)
        if (hit && closestLandmark) {
            endPoint = closestHitPoint;
            
            sf::FloatRect lb = closestLandmark->getShape().getGlobalBounds();
            sf::Vector2f landmarkPosition = sf::Vector2f(lb.left, lb.top);
            sf::Vector2f odometryOffset = closestLandmark->getShape().getPosition() - agentPosition + sf::Vector2f(LiDARnoiseDistribution(rng), LiDARnoiseDistribution(rng));
            landmarkPosition = position + odometryOffset;

            auto it = std::find_if(detectedLandmarks.begin(), detectedLandmarks.end(),
                                   [&closestLandmark](const Landmark& detected) {
                                       return detected.getId() == closestLandmark->getId();
                                   });
            float errorLength = std::sqrt(
                std::pow(landmarkPosition.x - closestLandmark->getShape().getPosition().x, 2) +
                std::pow(landmarkPosition.y - closestLandmark->getShape().getPosition().y, 2)
            );
            
            // Store individual error length
            landmarkErrors.push_back(errorLength);
            
            // Accumulate error for average calculation
            totalError += errorLength;
            errorCount++;
            
            if (it != detectedLandmarks.end()) {
                it->setObservedPos(landmarkPosition);
            } else {
                closestLandmark->setObservedPos(landmarkPosition);
                detectedLandmarks.push_back(*closestLandmark);
            }
        }
        
        // Set line tracer vertices
        int vertexIndex = i * 2;
        lineTracers[vertexIndex].position = agentCenter;
        lineTracers[vertexIndex].color = sf::Color::Red;
        lineTracers[vertexIndex + 1].position = endPoint;
        lineTracers[vertexIndex + 1].color = sf::Color::Red;
    }
    
    // Calculate average error for this frame
    if (errorCount > 0) {
        averageLandmarkError = totalError / errorCount;
    }
}


void Sensors::collectSensorData(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition,
                               sf::Vector2f previousVelocity, float direction, float previousDirection,
                               float gameTime, float currentAcceleration) {
    calculateAcceleration(deltaTime, position, previousPosition, previousVelocity, gameTime, currentAcceleration);
    calculateAngularVelocity(deltaTime, direction, previousDirection, gameTime);
}

void Sensors::calculateAcceleration(float deltaTime, sf::Vector2f position, sf::Vector2f previousPosition,
                                   sf::Vector2f previousVelocity, float gameTime, float currentAcceleration) {
    // Calculate true acceleration based on velocity change (what an IMU would actually measure)
    sf::Vector2f currentVelocity = (position - previousPosition) / deltaTime;
    sf::Vector2f acceleration = (currentVelocity - previousVelocity) / deltaTime;
    
   
    
    // Store accelerometer data
    AccelerometerData accelData;
    accelData.x = acceleration.x + accelNoiseDistribution(rng);
    accelData.y = acceleration.y+ accelNoiseDistribution(rng);
    accelData.timestamp = gameTime;
    
    accelerometerData.push_back(accelData);
    //std::cout << "accSize: "<< accelerometerData.size()<<std::endl;
    // Keep the accelerometerData small
    if(accelerometerData.size() > 10) {
        accelerometerData.erase(accelerometerData.begin());
    }
}

void Sensors::calculateAngularVelocity(float deltaTime, float direction, float previousDirection,
                                      float gameTime) {
    // Calculate angular velocity
    float angularVelocity = (direction - previousDirection) / deltaTime;
    
    // Convert to radians per second
    angularVelocity = angularVelocity * 3.14159f / 180.0f;
    
    // Add noise
    angularVelocity += gyroNoiseDistribution(rng);
    
    // Store gyroscope data
    GyroscopeData gyroData;
    gyroData.z = angularVelocity; // Yaw
    gyroData.timestamp = gameTime;
    
    gyroscopeData.push_back(gyroData);
    
    // Keep only recent data (last 5 seconds)
    while (!gyroscopeData.empty() && gyroscopeData.front().timestamp < gameTime - 5.0f) {
        gyroscopeData.erase(gyroscopeData.begin());
    }
}

void Sensors::updateGPS(float deltaTime, sf::Vector2f actualPosition, float gameTime) {
    lastGPSUpdateTime += deltaTime;
    
    // GPS updates every 1 second (typical for consumer GPS)
    if (lastGPSUpdateTime >= 1.0f) {
        // Add noise to GPS reading
        sf::Vector2f noisyPosition = actualPosition;
        noisyPosition.x += gpsNoiseDistribution(rng);
        noisyPosition.y += gpsNoiseDistribution(rng);
        
        // Calculate accuracy estimate (simulate varying GPS accuracy)
        float accuracy = 3.0f + abs(gpsNoiseDistribution(rng)); // 3-6m typical accuracy
        
        // Store GPS data
        GPSData gpsData;
        gpsData.position = noisyPosition;
        gpsData.accuracy = accuracy;
        gpsData.timestamp = gameTime;
        gpsData.valid = true; // Assume GPS is always valid for simplicity
        
        this->gpsData.push_back(gpsData);
        currentGPSPosition = noisyPosition;
        
        // Keep only recent data (last 30 seconds)
        while (!this->gpsData.empty() && this->gpsData.front().timestamp < gameTime - 30.0f) {
            this->gpsData.erase(this->gpsData.begin());
        }
        
        lastGPSUpdateTime = 0.0f;
    }
}

float Sensors::getTotalAverageError() const {
    if (detectedLandmarks.empty() || landmarks.empty()) {
        return 0.0f;
    }
    
    float totalError = 0.0f;
    int validComparisons = 0;
    
    // Compare each detected landmark with its corresponding actual landmark
    for (const auto& detected : detectedLandmarks) {
        // Find the corresponding actual landmark by ID
        for (const auto& actual : landmarks) {
            if (detected.getId() == actual.getId()) {
                sf::Vector2f detectedPos = detected.getObservedPos();
                sf::Vector2f actualPos = actual.getShape().getPosition();
                
                float error = std::sqrt(
                    std::pow(detectedPos.x - actualPos.x, 2) +
                    std::pow(detectedPos.y - actualPos.y, 2)
                );
                
                totalError += error;
                validComparisons++;
                break; // Found the matching landmark, move to next detected landmark
            }
        }
    }
    
    return validComparisons > 0 ? totalError / validComparisons : 0.0f;
}
