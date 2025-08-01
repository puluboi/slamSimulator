#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

// forward declarations
struct Landmark;

// grid cell states for minimap visualization
enum class GridCellState {
    UNKNOWN = 0,    // unexplored cell (invisible on minimap)
    EXPLORED = 1,   // explored free space (green on minimap)
    OCCUPIED = 2    // contains landmark (red on minimap)
};

struct MinimapGridCell {
    GridCellState state;
    bool hasLandmark;           // true if this cell contains a landmark
    int landmarkCount;          // number of landmarks in this cell
    float confidence;           // confidence level (0.0 - 1.0)
    
    MinimapGridCell() : state(GridCellState::UNKNOWN), hasLandmark(false), 
                        landmarkCount(0), confidence(0.0f) {}
};

class MinimapGrid {
public:
    MinimapGrid(float cellSize = 40.0f, int width = 20, int height = 15);
    
    // grid operations based on landmark data
    void updateFromLandmarks(const std::vector<Landmark>& landmarks);
    void markCellOccupied(sf::Vector2f worldPosition, float confidence = 1.0f);
    void markCellExplored(sf::Vector2f worldPosition, float confidence = 1.0f);
    void clearGrid();
    
    // grid coordinate conversion
    sf::Vector2i worldToGrid(sf::Vector2f worldPosition) const;
    sf::Vector2f gridToWorld(sf::Vector2i gridPosition) const;
    bool isValidGridCoordinate(sf::Vector2i gridPosition) const;
    
    // visualization on minimap
    void renderOnMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, 
                        sf::Vector2f minimapSize, sf::Vector2u worldSize) const;
    
    // grid properties
    float getCellSize() const { return cellSize; }
    sf::Vector2i getGridSize() const { return sf::Vector2i(gridWidth, gridHeight); }
    void setOrigin(sf::Vector2f origin) { gridOrigin = origin; }
    sf::Vector2f getOrigin() const { return gridOrigin; }
    
    // grid state queries
    GridCellState getCellState(sf::Vector2f worldPosition) const;
    bool isOccupied(sf::Vector2f worldPosition) const;
    
private:
    float cellSize;                         // size of each grid cell in world units
    int gridWidth, gridHeight;              // grid dimensions in cells
    sf::Vector2f gridOrigin;                // world position of grid origin (center)
    std::vector<std::vector<MinimapGridCell>> grid;  // 2d grid of cells
    
    // helper functions
    void updateCell(sf::Vector2i gridPos, bool occupied, float confidence);
    sf::Color getCellColor(const MinimapGridCell& cell) const;
    void markLandmarkArea(const Landmark& landmark);
};
