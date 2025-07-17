#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include "landmark.hpp"

struct GridCell {
    int x, y;
    bool occupied = false;
    bool explored = false;
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct PathNode {
    int x, y;
    float gCost, hCost, fCost;
    PathNode* parent;
    
    PathNode(int x, int y) : x(x), y(y), gCost(0), hCost(0), fCost(0), parent(nullptr) {}
    
    bool operator>(const PathNode& other) const {
        return fCost > other.fCost;
    }
};

class Pathfinding {
public:
    Pathfinding(int worldWidth, int worldHeight, int gridSize = 20);
    
    // Update occupancy grid with detected landmarks
    void updateOccupancyGrid(const std::vector<Landmark>& detectedLandmarks, sf::Vector2f agentPos);
    
    // Find path using A* algorithm
    std::vector<sf::Vector2f> findPath(sf::Vector2f start, sf::Vector2f goal);
    
    // Find nearest unexplored cell for exploration
    sf::Vector2f findNearestUnexplored(sf::Vector2f agentPos);
    
    // Get the occupancy grid for visualization
    const std::vector<std::vector<GridCell>>& getGrid() const { return grid; }
    
    // Check if exploration is complete
    bool isExplorationComplete() const;
    
    // Get grid size
    int getGridSize() const { return gridSize; }
    
private:
    std::vector<std::vector<GridCell>> grid;
    int worldWidth, worldHeight, gridSize;
    int gridCols, gridRows;
    
    // A* helper functions
    float calculateHeuristic(int x1, int y1, int x2, int y2);
    std::vector<sf::Vector2i> getNeighbors(int x, int y);
    bool isValidCell(int x, int y);
    sf::Vector2i worldToGrid(sf::Vector2f worldPos);
    sf::Vector2f gridToWorld(sf::Vector2i gridPos);
};

#endif // PATHFINDING_HPP
