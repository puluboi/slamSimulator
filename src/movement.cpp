#include "movement.hpp"
#include <cmath>


Movement::Movement() {
    speed = 200.0f;
    acceleration =80.f;
    deacceleration = 10.f;
    currentSpeed = 0.0f;
    currentAcceleration = 0.0f;
    direction = 0.0f;        // facing right initially
    targetDirection = 0.0f;  // start with same target
    rotationSpeed = 900.0f;   // 90 degrees per second
}

void Movement::init(sf::Vector2f startPosition) {
    position = startPosition;
    
    // setup agent shape
    shape.setSize(sf::Vector2f(10.0f, 10.0f));
    shape.setFillColor(sf::Color::Blue);
    shape.setPosition(position);
}

void Movement::update(float deltaTime, bool keys[4], sf::Vector2u windowSize, 
                     const std::vector<Landmark>& landmarks) {
    // store current position for collision rollback
    sf::Vector2f oldPosition = position;
    
    // update movement and rotation
    updateMovement(deltaTime, keys, windowSize);
    updateRotation(deltaTime);
    
    // check for collisions with landmarks
    if (checkCollisionWithLandmarks(position, landmarks)) {
        // collision detected, revert to old position
        position = oldPosition;
    }
    
    // update agent shape position
    shape.setPosition(position);
}

void Movement::updateMovement(float deltaTime, bool keys[4], sf::Vector2u windowSize) {
    sf::Vector2f addedMov(0, 0);
    // handle movement input
    // if no keyboard input, the robot deaccelerates abruptly
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
    if (keys[0]) { // w key - move up
        addedMov.y = -currentSpeed * deltaTime;
    }
    if (keys[1]) { // a key - move left
        addedMov.x = -currentSpeed * deltaTime;
    }
    if (keys[2]) { // s key - move down
        addedMov.y = currentSpeed * deltaTime;
    }
    if (keys[3]) { // d key - move right
        addedMov.x = currentSpeed * deltaTime;
    }
    // if no keys are pressed, drift in the last controlled direction
    if (!keys[0] && !keys[1] && !keys[2] && !keys[3] && currentSpeed > 0.0f) {
        addedMov.x += std::cos(direction * 3.14159f / 180.0f) * currentSpeed * deltaTime;
        addedMov.y += std::sin(direction * 3.14159f / 180.0f) * currentSpeed * deltaTime;
    }
    position += addedMov;
    
    // only update target direction if we're actually moving
    if (addedMov.x != 0.0f || addedMov.y != 0.0f) {
        targetDirection = std::atan2(addedMov.y, addedMov.x) * 180.0f / 3.14159f; // convert to degrees
    }
    
    // keep agent within window bounds
    if (position.x < 0) position.x = 0;
    if (position.y < 0) position.y = 0;
    if (position.x > windowSize.x - shape.getSize().x) 
        position.x = windowSize.x - shape.getSize().x;
    if (position.y > windowSize.y - shape.getSize().y) 
        position.y = windowSize.y - shape.getSize().y;
}

void Movement::updateRotation(float deltaTime) {
    // calculate the shortest angle difference
    float diff = angleDifference(targetDirection, direction);
    
    // if the difference is small enough, snap to target
    if (abs(diff) < rotationSpeed * deltaTime) {
        direction = targetDirection;
    } else {
        // rotate towards target direction
        if (diff > 0) {
            direction += rotationSpeed * deltaTime;
        } else {
            direction -= rotationSpeed * deltaTime;
        }
        
        // keep angle in 0-360 range
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
