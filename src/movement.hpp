#pragma once

#include <SFML/Graphics.hpp>
#include "landmark.hpp" 
class Movement {
public:
    Movement();
    
    void init(sf::Vector2f startPosition);
    void update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                const std::vector<Landmark>& landmarks = {});
    
    // Getters
    sf::Vector2f getPosition() const { return position; }
    float getVelocity() const { return currentSpeed; }
    float getDirection() const { return direction; }
    sf::RectangleShape getShape() const { return shape; }
    float getCurrentAcceleration() const { return currentAcceleration; }
    
private:
    void updateMovement(float deltaTime, bool keys[4], sf::Vector2u windowSize);
    void updateRotation(float deltaTime);
    bool checkCollisionWithLandmarks(sf::Vector2f newPosition, const std::vector<Landmark>& landmarks);
    float angleDifference(float angle1, float angle2);
    
    sf::RectangleShape shape;
    sf::Vector2f position;
    float speed;
    float acceleration;
    float deacceleration;
    float currentAcceleration;
    float currentSpeed;
    float direction; // Current facing direction in degrees
    float targetDirection; // Target direction to rotate towards
    float rotationSpeed; // Rotation speed in degrees per second
};
