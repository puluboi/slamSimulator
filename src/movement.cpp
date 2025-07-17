#include "movement.hpp"
#include <cmath>


Movement::Movement() {
    speed = 20.0f;
    acceleration =8.f;
    deacceleration = 0.1f;
    currentSpeed = 0.0f;
    currentAcceleration = 0.0f;
    direction = 0.0f;        // Facing right initially
    targetDirection = 0.0f;  // Start with same target
    rotationSpeed = 90.0f;   // 90 degrees per second
}

void Movement::init(sf::Vector2f startPosition) {
    position = startPosition;
    
    // Setup agent shape
    shape.setSize(sf::Vector2f(10.0f, 10.0f));
    shape.setFillColor(sf::Color::Blue);
    shape.setPosition(position);
}

void Movement::update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                     const std::vector<Landmark>& landmarks) {
    // Store current position for collision rollback
    sf::Vector2f oldPosition = position;
    
    // Update movement and rotation
    updateMovement(deltaTime, keys, windowSize);
    updateRotation(deltaTime);
    
    // Check for collisions with landmarks
    if (checkCollisionWithLandmarks(position, landmarks)) {
        // Collision detected, revert to old position
        position = oldPosition;
    }
    
    // Update agent shape position
    shape.setPosition(position);
}

void Movement::updateMovement(float deltaTime, bool keys[4], sf::Vector2u windowSize) {
    sf::Vector2f addedMov(0, 0);
    // Handle movement input
    // If no keyboard input, the robot deaccelerates abruptly
    if (!keys[0] && !keys[1] && !keys[2] && !keys[3]) {
        currentAcceleration = -deacceleration;
        currentSpeed = std::max(0.0f, currentSpeed+currentAcceleration);
        if(currentSpeed == 0.0f || currentSpeed >= speed){
            currentAcceleration = 0.0f;
        }
    } else {
        currentAcceleration = acceleration;
        currentSpeed = std::min(currentSpeed + currentAcceleration * deltaTime, speed); // cap the speed
        
    }
    if (keys[0]) { // W key - move up
        addedMov.y = -currentSpeed * deltaTime;
    }
    if (keys[1]) { // A key - move left
        addedMov.x = -currentSpeed * deltaTime;
    }
    if (keys[2]) { // S key - move down
        addedMov.y = currentSpeed * deltaTime;
    }
    if (keys[3]) { // D key - move right
        addedMov.x = currentSpeed * deltaTime;
    }
    // If no keys are pressed, drift in the last controlled direction
    if (!keys[0] && !keys[1] && !keys[2] && !keys[3] && currentSpeed > 0.0f) {
        addedMov.x += std::cos(direction * 3.14159f / 180.0f) * currentSpeed * deltaTime;
        addedMov.y += std::sin(direction * 3.14159f / 180.0f) * currentSpeed * deltaTime;
    }
    position += addedMov;
    
    // Only update target direction if we're actually moving
    if (addedMov.x != 0.0f || addedMov.y != 0.0f) {
        targetDirection = std::atan2(addedMov.y, addedMov.x) * 180.0f / 3.14159f; // Convert to degrees
    }
    
    // Keep agent within window bounds
    if (position.x < 0) position.x = 0;
    if (position.y < 0) position.y = 0;
    if (position.x > windowSize.x - shape.getSize().x) 
        position.x = windowSize.x - shape.getSize().x;
    if (position.y > windowSize.y - shape.getSize().y) 
        position.y = windowSize.y - shape.getSize().y;
}

void Movement::updateRotation(float deltaTime) {
    // Calculate the shortest angle difference
    float diff = angleDifference(targetDirection, direction);
    
    // If the difference is small enough, snap to target
    if (abs(diff) < rotationSpeed * deltaTime) {
        direction = targetDirection;
    } else {
        // Rotate towards target direction
        if (diff > 0) {
            direction += rotationSpeed * deltaTime;
        } else {
            direction -= rotationSpeed * deltaTime;
        }
        
        // Keep angle in 0-360 range
        if (direction < 0) direction += 360.0f;
        if (direction >= 360.0f) direction -= 360.0f;
    }
}

float Movement::angleDifference(float angle1, float angle2) {
    // Calculate the shortest angular difference between two angles
    float diff = angle1 - angle2;
    
    // Normalize to [-180, 180] range
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    return diff;
}

bool Movement::checkCollisionWithLandmarks(sf::Vector2f newPosition, const std::vector<Landmark>& landmarks) {
    // Create a temporary rectangle at the new position to check for collisions
    sf::RectangleShape tempShape = shape;
    tempShape.setPosition(newPosition);
    
    // Check collision with each landmark
    for (const auto& landmark : landmarks) {
        if (tempShape.getGlobalBounds().intersects(landmark.getShape().getGlobalBounds())) {
            return true; // Collision detected
        }
    }
    
    return false; // No collision
}
