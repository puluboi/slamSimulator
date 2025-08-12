#pragma once

#include <SFML/Graphics.hpp>
#include "sensors.hpp"
#include "slam.hpp"  // Include for SLAMAlgorithm enum and UFastSLAM access

class Renderer {
public:
    static void renderAgent(sf::RenderWindow& window, const sf::RectangleShape& shape,
                           const std::vector<sf::Vertex>& lineTracers);
    
    static void renderMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, 
                            sf::Vector2f minimapSize, sf::Vector2u worldSize,
                            sf::Vector2f agentPosition, sf::Vector2f agentSize,
                            sf::Vector2f odometryPosition, sf::Vector2f gpsPosition,
                            float direction, float odometryDirection, 
                            const std::vector<GPSData>& gpsData, bool gpsEnabled,
                            const std::vector<Landmark>& dlandmarks,
                            const std::vector<sf::Vector2f>& currentPath,
                            sf::Vector2f currentTarget,
                            size_t currentPathIndex,
                            bool explorationMode,
                            const SLAM* slam = nullptr);  // Add SLAM pointer for particle access
};
