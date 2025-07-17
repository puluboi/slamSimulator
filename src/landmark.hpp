#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <SFML/Graphics.hpp>

class Landmark{
    public:
    Landmark(sf::RectangleShape shape_, int id_)
        : id(id_), shape_(shape_) {}
    int getId() const {
        return id;
    }

    const sf::RectangleShape& getShape() const {
        return shape_;
    }
    void setObservedPos(const sf::Vector2f& pos) {
        observedPos = pos;
    }

    const sf::Vector2f& getObservedPos() const {
        return observedPos;
    }
    private:
    sf::Vector2f observedPos;
    int id;
    sf::RectangleShape shape_;
};

#endif // LANDMARK_HPP