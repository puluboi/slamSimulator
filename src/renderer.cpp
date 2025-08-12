#include "renderer.hpp"
#include <cmath>
#include <set>
#include <algorithm>  // for std::sort
#include <iostream>   // for potential debugging

void Renderer::renderAgent(sf::RenderWindow& window, const sf::RectangleShape& shape,
                          const std::vector<sf::Vertex>& lineTracers) {
    // Draw line tracers
    if (lineTracers.size() > 0) {
        window.draw(&lineTracers[0], lineTracers.size(), sf::Lines);
    }
    
    // Draw agent
    window.draw(shape);
}


void Renderer::renderMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, 
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
                            const SLAM* slam) {
    // Draw minimap background
    sf::RectangleShape minimapBackground;
    minimapBackground.setPosition(minimapPosition);
    minimapBackground.setSize(minimapSize);
    minimapBackground.setFillColor(sf::Color(50, 50, 50, 200));
    minimapBackground.setOutlineThickness(2.0f);
    minimapBackground.setOutlineColor(sf::Color::White);
    window.draw(minimapBackground);
    
    // Calculate scale factors
    float scaleX = minimapSize.x / worldSize.x;
    float scaleY = minimapSize.y / worldSize.y;
    
    // Draw optimized occupancy grid using vertex arrays
    const int gridSize = 20; // Grid cell size in world coordinates
    static sf::VertexArray occupancyGrid(sf::Quads);
    occupancyGrid.clear();
    
    // Use a set to avoid duplicate grid cells
    std::set<std::pair<int, int>> occupiedCells;
    
    // Mark grid cells that contain landmarks
    for (const auto& landmark : dlandmarks) {
        sf::Vector2f landmarkPos = landmark.getObservedPos();
        sf::Vector2f landmarkSize = landmark.getShape().getSize();
        
        // Calculate which grid cells this landmark occupies
        int minGridX = std::max(0, static_cast<int>(landmarkPos.x / gridSize));
        int maxGridX = std::min(static_cast<int>(worldSize.x / gridSize) - 1, 
                               static_cast<int>((landmarkPos.x + landmarkSize.x) / gridSize));
        int minGridY = std::max(0, static_cast<int>(landmarkPos.y / gridSize));
        int maxGridY = std::min(static_cast<int>(worldSize.y / gridSize) - 1, 
                               static_cast<int>((landmarkPos.y + landmarkSize.y) / gridSize));
        
        for (int gridY = minGridY; gridY <= maxGridY; gridY++) {
            for (int gridX = minGridX; gridX <= maxGridX; gridX++) {
                occupiedCells.insert({gridX, gridY});
            }
        }
    }
    
    // Create vertices for occupied cells (4 vertices per quad)
    occupancyGrid.resize(occupiedCells.size() * 4);
    size_t vertexIndex = 0;
    
    for (const auto& cell : occupiedCells) {
        float cellX = minimapPosition.x + cell.first * gridSize * scaleX;
        float cellY = minimapPosition.y + cell.second * gridSize * scaleY;
        float cellWidth = gridSize * scaleX;
        float cellHeight = gridSize * scaleY;
        
        // Create quad vertices (top-left, top-right, bottom-right, bottom-left)
        occupancyGrid[vertexIndex].position = sf::Vector2f(cellX, cellY);
        occupancyGrid[vertexIndex].color = sf::Color(255, 0, 0, 100);
        vertexIndex++;
        
        occupancyGrid[vertexIndex].position = sf::Vector2f(cellX + cellWidth, cellY);
        occupancyGrid[vertexIndex].color = sf::Color(255, 0, 0, 100);
        vertexIndex++;
        
        occupancyGrid[vertexIndex].position = sf::Vector2f(cellX + cellWidth, cellY + cellHeight);
        occupancyGrid[vertexIndex].color = sf::Color(255, 0, 0, 100);
        vertexIndex++;
        
        occupancyGrid[vertexIndex].position = sf::Vector2f(cellX, cellY + cellHeight);
        occupancyGrid[vertexIndex].color = sf::Color(255, 0, 0, 100);
        vertexIndex++;
    }
    
    // Draw all occupied cells in a single draw call
    if (!occupancyGrid.getVertexCount() == 0) {
        window.draw(occupancyGrid);
    }
    
    // Draw agent position (actual position in blue)
    sf::CircleShape agentDot(3.0f);
    sf::Vector2f agentMinimapPos = minimapPosition + sf::Vector2f(
        agentPosition.x * scaleX+3.f,
        agentPosition.y * scaleY+3.f
    );
    agentDot.setPosition(agentMinimapPos - sf::Vector2f(3.0f, 3.0f));
    agentDot.setFillColor(sf::Color::Blue);
    window.draw(agentDot);
    
    // Draw agent direction
    float arrowLength = 8.0f;
    float radians = direction * 3.14159f / 180.0f;
    sf::Vector2f arrowEnd = agentMinimapPos + sf::Vector2f(
        cos(radians) * arrowLength,
        sin(radians) * arrowLength
    );
    
    sf::Vertex directionLine[] = {
        sf::Vertex(agentMinimapPos, sf::Color::Blue),
        sf::Vertex(arrowEnd, sf::Color::Blue)
    };
    window.draw(directionLine, 2, sf::Lines);
    
    // Draw odometry position (estimated position in red)
    sf::CircleShape odometryDot(3.0f);
    sf::Vector2f odometryMinimapPos = minimapPosition + sf::Vector2f(
        odometryPosition.x * scaleX+3.f,
        odometryPosition.y * scaleY+3.f
    );
    odometryDot.setPosition(odometryMinimapPos - sf::Vector2f(3.0f, 3.0f));
    odometryDot.setFillColor(sf::Color::Red);
    window.draw(odometryDot);
    
    // Draw odometry direction
    float odometryRadians = odometryDirection * 3.14159f / 180.0f;
    sf::Vector2f odometryArrowEnd = odometryMinimapPos + sf::Vector2f(
        cos(odometryRadians) * arrowLength,
        sin(odometryRadians) * arrowLength
    );
    
    sf::Vertex odometryDirectionLine[] = {
        sf::Vertex(odometryMinimapPos, sf::Color::Red),
        sf::Vertex(odometryArrowEnd, sf::Color::Red)
    };
    window.draw(odometryDirectionLine, 2, sf::Lines);
    
    // Draw GPS position (if GPS is enabled and available, in green)
    if (gpsEnabled) {
        sf::CircleShape gpsDot(3.0f);
        sf::Vector2f gpsMinimapPos = minimapPosition + sf::Vector2f(
            gpsPosition.x * scaleX,
            gpsPosition.y * scaleY
        );
        gpsDot.setPosition(gpsMinimapPos - sf::Vector2f(3.0f, 3.0f));
        gpsDot.setFillColor(sf::Color::Green);
        window.draw(gpsDot);
        
        // Draw recent GPS trail
        for (size_t i = 1; i < gpsData.size() && i < 20; ++i) {
            if (gpsData[i].valid && gpsData[i-1].valid) {
                sf::Vector2f pos1 = minimapPosition + sf::Vector2f(
                    gpsData[i-1].position.x * scaleX,
                    gpsData[i-1].position.y * scaleY
                );
                sf::Vector2f pos2 = minimapPosition + sf::Vector2f(
                    gpsData[i].position.x * scaleX,
                    gpsData[i].position.y * scaleY
                );
                
                sf::Vertex gpsTrail[] = {
                    sf::Vertex(pos1, sf::Color(0, 255, 0, 128)),
                    sf::Vertex(pos2, sf::Color(0, 255, 0, 128))
                };
                window.draw(gpsTrail, 2, sf::Lines);
            }
        }
    }
    
    // Draw top 10 particles when in uFastSLAM mode
    if (slam && slam->getCurrentAlgorithm() == SLAMAlgorithm::UFASTSLAM) {
        const auto& ufastslam = slam->getUFastSLAM();
        const auto& particles = ufastslam.getParticles();
        
        // Get particles sorted by weight
        std::vector<std::pair<double, int>> particleWeights;
        for (int i = 0; i < particles.size(); i++) {
            particleWeights.push_back({particles[i].weight, i});
        }
        
        // Sort by weight (highest first)
        std::sort(particleWeights.begin(), particleWeights.end(), 
                  std::greater<std::pair<double, int>>());
        
        // Draw top 10 particles
        int numParticlesToDraw = std::min(10, static_cast<int>(particleWeights.size()));
        
        for (int i = 0; i < numParticlesToDraw; i++) {
            int particleIndex = particleWeights[i].second;
            const auto& particle = particles[particleIndex];
            
            // Convert world coordinates to minimap coordinates
            sf::Vector2f particleMinimapPos = minimapPosition + sf::Vector2f(
                particle.pose(0) * scaleX,
                particle.pose(1) * scaleY
            );
            
            // Set transparency based on rank (best particle is most opaque)
            sf::Uint8 alpha = static_cast<sf::Uint8>(150 - (i * 10)); // Fade from 150 to 60
            
            // Draw particle position as small yellow circle
            sf::CircleShape particleShape(1.5f);
            particleShape.setFillColor(sf::Color(255, 255, 0, alpha));
            particleShape.setOrigin(1.5f, 1.5f);
            particleShape.setPosition(particleMinimapPos);
            window.draw(particleShape);
            
            // Draw particle orientation as small yellow line
            float arrowLength = 6.0f;
            float radians = particle.pose(2);
            sf::Vector2f arrowEnd = particleMinimapPos + sf::Vector2f(
                cos(radians) * arrowLength,
                sin(radians) * arrowLength
            );
            
            sf::Vertex directionLine[] = {
                sf::Vertex(particleMinimapPos, sf::Color(255, 255, 0, alpha)),
                sf::Vertex(arrowEnd, sf::Color(255, 255, 0, alpha))
            };
            window.draw(directionLine, 2, sf::Lines);
        }
    }
    
    // Draw landmarks using vertex arrays for better performance
    static sf::VertexArray landmarkVertices(sf::Quads);
    landmarkVertices.clear();
    landmarkVertices.resize(dlandmarks.size() * 4);
    
    size_t landmarkVertexIndex = 0;
    for (const auto& landmark : dlandmarks) {
        sf::Vector2f landmarkPosition = landmark.getObservedPos();
        sf::Vector2f landmarkSize = landmark.getShape().getSize();
        
        float landmarkX = minimapPosition.x + landmarkPosition.x * scaleX;
        float landmarkY = minimapPosition.y + landmarkPosition.y * scaleY;
        float landmarkWidth = landmarkSize.x * scaleX;
        float landmarkHeight = landmarkSize.y * scaleY;
        
        // Create quad vertices for landmark
        landmarkVertices[landmarkVertexIndex].position = sf::Vector2f(landmarkX, landmarkY);
        landmarkVertices[landmarkVertexIndex].color = sf::Color::Black;
        landmarkVertexIndex++;
        
        landmarkVertices[landmarkVertexIndex].position = sf::Vector2f(landmarkX + landmarkWidth, landmarkY);
        landmarkVertices[landmarkVertexIndex].color = sf::Color::Black;
        landmarkVertexIndex++;
        
        landmarkVertices[landmarkVertexIndex].position = sf::Vector2f(landmarkX + landmarkWidth, landmarkY + landmarkHeight);
        landmarkVertices[landmarkVertexIndex].color = sf::Color::Black;
        landmarkVertexIndex++;
        
        landmarkVertices[landmarkVertexIndex].position = sf::Vector2f(landmarkX, landmarkY + landmarkHeight);
        landmarkVertices[landmarkVertexIndex].color = sf::Color::Black;
        landmarkVertexIndex++;
    }
    
    // Draw all landmarks in a single draw call
    if (landmarkVertices.getVertexCount() > 0) {
        window.draw(landmarkVertices);
    }
    
    // Draw legend
    sf::Vector2f legendPos = minimapPosition + sf::Vector2f(5.0f, minimapSize.y - 40.0f);
    
    
    // GPS (green) - only show if GPS is enabled
    if (gpsEnabled) {
        sf::CircleShape legendGPS(2.0f);
        legendGPS.setPosition(legendPos + sf::Vector2f(0.0f, 16.0f));
        legendGPS.setFillColor(sf::Color::Green);
        window.draw(legendGPS);
    }
    
    // Draw current target (red circle) if in exploration mode
    if (explorationMode && (currentTarget.x != 0.0f || currentTarget.y != 0.0f)) {
        float targetX = minimapPosition.x + currentTarget.x * scaleX;
        float targetY = minimapPosition.y + currentTarget.y * scaleY;
        
        // Draw target circle
        sf::CircleShape targetCircle(2.0f);
        targetCircle.setOrigin(2.f, 2.f);
        targetCircle.setPosition(targetX, targetY);
        targetCircle.setFillColor(sf::Color::Red);
        targetCircle.setOutlineColor(sf::Color::White);
        targetCircle.setOutlineThickness(1.0f);
        window.draw(targetCircle);
        
        
    }
    
    // Draw current path (blue line) if in exploration mode
    if (explorationMode && !currentPath.empty()) {
        static sf::VertexArray pathLine(sf::LineStrip);
        pathLine.clear();
        
        // Add agent position as starting point
        float agentX = minimapPosition.x + agentPosition.x * scaleX+10.f;
        float agentY = minimapPosition.y + agentPosition.y * scaleY+10.f;
        pathLine.append(sf::Vertex(sf::Vector2f(agentX, agentY), sf::Color::Blue));
        
        // Add all path points
        for (size_t i = currentPathIndex; i < currentPath.size(); ++i) {
            float pathX = minimapPosition.x + currentPath[i].x * scaleX;
            float pathY = minimapPosition.y + currentPath[i].y * scaleY;
            
            // Use different colors for completed and remaining path
            sf::Color pathColor = (i <= currentPathIndex) ? sf::Color(100, 100, 255) : sf::Color::Blue;
            pathLine.append(sf::Vertex(sf::Vector2f(pathX, pathY), pathColor));
        }
        
        // Draw the path
        if (pathLine.getVertexCount() > 1) {
            window.draw(pathLine);
        }
        
        // Draw path waypoints as small circles
        for (size_t i = currentPathIndex; i < currentPath.size(); ++i) {
            float pathX = minimapPosition.x + currentPath[i].x * scaleX;
            float pathY = minimapPosition.y + currentPath[i].y * scaleY;
            
            sf::CircleShape waypoint(2.0f);
            waypoint.setOrigin(2.0f, 2.0f);
            waypoint.setPosition(pathX, pathY);
            
            // Current waypoint is highlighted
            if (i == currentPathIndex) {
                waypoint.setFillColor(sf::Color::Yellow);
                waypoint.setOutlineColor(sf::Color::White);
                waypoint.setOutlineThickness(1.0f);
            } else {
                waypoint.setFillColor(sf::Color::Blue);
            }
            
            window.draw(waypoint);
        }
    }
    
    // Add legend entries for path and target
    if (explorationMode) {
        sf::Vector2f pathLegendPos = legendPos + sf::Vector2f(40.0f, 0.0f);
        
       
    }
}