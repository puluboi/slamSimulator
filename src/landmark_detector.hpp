#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

// Landmark types for navigation
enum class LandmarkType {
    POINT,      // Single point obstacle
    LINE,       // Linear wall/barrier
    CORNER,     // Corner/edge feature
    CIRCLE,     // Circular obstacle
    POLYGON     // Complex polygon shape
};

// Landmark structure for SLAM and navigation
struct Landmark {
    sf::Vector2f position;              // Center position in odometry frame
    LandmarkType type;                  // Type of landmark
    float confidence;                   // Detection confidence (0-1)
    sf::Vector2f size;                  // Size (width, height) for navigation
    float orientation;                  // Orientation in degrees
    std::vector<sf::Vector2f> shape;    // Detailed shape points for complex landmarks
    int observationCount;               // Number of times observed
    float lastSeenTime;                 // Last observation timestamp
    sf::CircleShape visualMarker;       // For rendering
    
    // Additional fields for landmark creator
    float clearanceRadius;              // Safe distance for navigation
    bool isNavigable;                   // Whether robots can pass through/around
    
    // Constructor
    Landmark() : position(0, 0), type(LandmarkType::POINT), confidence(0.5f), 
                 size(10, 10), orientation(0), observationCount(1), lastSeenTime(0),
                 clearanceRadius(5.0f), isNavigable(true) {
        visualMarker.setRadius(5.0f);
        visualMarker.setFillColor(sf::Color::Yellow);
    }
};

class LandmarkDetector {
public:
    LandmarkDetector();
    
    // Simplified update function (no hit markers needed)
    void update(sf::Vector2f odometryPosition, float gameTime);
    
    // Feature detection algorithms
    std::vector<Landmark> detectPointFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectLineFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectCornerFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectCircularFeatures(const std::vector<sf::Vector2f>& points);
    
    // Shape analysis
    void analyzeShapeProperties(Landmark& landmark, const std::vector<sf::Vector2f>& points);
    void updateNavigationProperties(Landmark& landmark);
    
    // Utility functions
    std::vector<std::vector<sf::Vector2f>> clusterPoints(const std::vector<sf::Vector2f>& points);
    float calculateDistance(sf::Vector2f a, sf::Vector2f b);
    bool arePointsCollinear(const std::vector<sf::Vector2f>& points, float tolerance = 5.0f);
    sf::Vector2f calculateCentroid(const std::vector<sf::Vector2f>& points);
    
    // Getters
    const std::vector<Landmark>& getLandmarks() const { return landmarks; }
    
private:
    std::vector<Landmark> landmarks;
    
    // Detection parameters
    int minPointsForFeature;
    float maxClusterDistance;
    float minConfidence;
    float associationDistance;
};
