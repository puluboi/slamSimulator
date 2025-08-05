#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

// Forward declarations
struct Landmark;

// Grid cell states for minimap visualization
enum class GridCellState {
    UNKNOWN = 0,    // Unexplored cell (invisible on minimap)
    EXPLORED = 1,   // Explored free space (green on minimap)
    OCCUPIED = 2    // Contains landmark (red on minimap)
};

struct MinimapGridCell {
    GridCellState state;
    bool hasLandmark;           // True if this cell contains a landmark
    int landmarkCount;          // Number of landmarks in this cell
    float confidence;           // Confidence level (0.0 - 1.0)
    
    MinimapGridCell() : state(GridCellState::UNKNOWN), hasLandmark(false), 
                        landmarkCount(0), confidence(0.0f) {}
};

class MinimapGrid {
public:
    MinimapGrid(float cellSize = 40.0f, int width = 20, int height = 15);
    
    // Grid operations based on landmark data
    void updateFromLandmarks(const std::vector<Landmark>& landmarks);
    void markCellOccupied(sf::Vector2f worldPosition, float confidence = 1.0f);
    void markCellExplored(sf::Vector2f worldPosition, float confidence = 1.0f);
    void clearGrid();
    
    // Grid coordinate conversion
    sf::Vector2i worldToGrid(sf::Vector2f worldPosition) const;
    sf::Vector2f gridToWorld(sf::Vector2i gridPosition) const;
    bool isValidGridCoordinate(sf::Vector2i gridPosition) const;
    
    // Visualization on minimap
    void renderOnMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, 
                        sf::Vector2f minimapSize, sf::Vector2u worldSize) const;
    
    // Grid properties
    float getCellSize() const { return cellSize; }
    sf::Vector2i getGridSize() const { return sf::Vector2i(gridWidth, gridHeight); }
    void setOrigin(sf::Vector2f origin) { gridOrigin = origin; }
    sf::Vector2f getOrigin() const { return gridOrigin; }
    
    // Grid state queries
    GridCellState getCellState(sf::Vector2f worldPosition) const;
    bool isOccupied(sf::Vector2f worldPosition) const;
    
private:
    float cellSize;                         // Size of each grid cell in world units
    int gridWidth, gridHeight;              // Grid dimensions in cells
    sf::Vector2f gridOrigin;                // World position of grid origin (center)
    std::vector<std::vector<MinimapGridCell>> grid;  // 2D grid of cells
    
    // Helper functions
    void updateCell(sf::Vector2i gridPos, bool occupied, float confidence);
    sf::Color getCellColor(const MinimapGridCell& cell) const;
    void markLandmarkArea(const Landmark& landmark);
};
