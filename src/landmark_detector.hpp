#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

// landmark types for navigation
enum class LandmarkType {
    POINT,      // single point obstacle
    LINE,       // linear wall/barrier
    CORNER,     // corner/edge feature
    CIRCLE,     // circular obstacle
    POLYGON     // complex polygon shape
};

// landmark structure for slam and navigation
struct Landmark {
    sf::Vector2f position;              // center position in odometry frame
    LandmarkType type;                  // type of landmark
    float confidence;                   // detection confidence (0-1)
    sf::Vector2f size;                  // size (width, height) for navigation
    float orientation;                  // orientation in degrees
    std::vector<sf::Vector2f> shape;    // detailed shape points for complex landmarks
    int observationCount;               // number of times observed
    float lastSeenTime;                 // last observation timestamp
    sf::CircleShape visualMarker;       // for rendering
    
    // additional fields for landmark creator
    float clearanceRadius;              // safe distance for navigation
    bool isNavigable;                   // whether robots can pass through/around
    
    // constructor
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
    
    // simplified update function (no hit markers needed)
    void update(sf::Vector2f odometryPosition, float gameTime);
    
    // feature detection algorithms
    std::vector<Landmark> detectPointFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectLineFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectCornerFeatures(const std::vector<sf::Vector2f>& points);
    std::vector<Landmark> detectCircularFeatures(const std::vector<sf::Vector2f>& points);
    
    // shape analysis functions
    void analyzeShapeProperties(Landmark& landmark, const std::vector<sf::Vector2f>& points);
    void updateNavigationProperties(Landmark& landmark);
    
    // utility functions for landmark detection
    std::vector<std::vector<sf::Vector2f>> clusterPoints(const std::vector<sf::Vector2f>& points);
    float calculateDistance(sf::Vector2f a, sf::Vector2f b);
    bool arePointsCollinear(const std::vector<sf::Vector2f>& points, float tolerance = 5.0f);
    sf::Vector2f calculateCentroid(const std::vector<sf::Vector2f>& points);
    
    // getter methods
    const std::vector<Landmark>& getLandmarks() const { return landmarks; }
    
private:
    std::vector<Landmark> landmarks;
    
    // detection parameters
    int minPointsForFeature;
    float maxClusterDistance;
    float minConfidence;
    float associationDistance;
};
