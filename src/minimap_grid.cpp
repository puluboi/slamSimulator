#include "minimap_grid.hpp"
#include "landmark_detector.hpp"
#include <cmath>
#include <algorithm>

MinimapGrid::MinimapGrid(float cellSize, int width, int height) 
    : cellSize(cellSize), gridWidth(width), gridHeight(height), gridOrigin(0.0f, 0.0f) {
    // Initialize grid with unknown cells
    grid.resize(gridHeight);
    for (int y = 0; y < gridHeight; ++y) {
        grid[y].resize(gridWidth);
    }
}

void MinimapGrid::updateFromLandmarks(const std::vector<Landmark>& landmarks) {
    // Clear previous landmark data but keep the grid structure
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            // Only reset landmark-specific data, preserve exploration state
            if (grid[y][x].state != GridCellState::EXPLORED) {
                grid[y][x].state = GridCellState::UNKNOWN;
            }
            grid[y][x].hasLandmark = false;
            grid[y][x].landmarkCount = 0;
            // Keep confidence if cell is explored
            if (grid[y][x].state == GridCellState::UNKNOWN) {
                grid[y][x].confidence = 0.0f;
            }
        }
    }
    
    // Mark cells containing landmarks
    for (const auto& landmark : landmarks) {
        markLandmarkArea(landmark);
    }
}

void MinimapGrid::markLandmarkArea(const Landmark& landmark) {
    switch (landmark.type) {
        case LandmarkType::POINT:
            // Mark a single cell for point landmarks
            markCellOccupied(landmark.position, landmark.confidence);
            break;
            
        case LandmarkType::LINE:
            // Mark cells along the line
            if (landmark.shape.size() >= 2) {
                sf::Vector2f start = landmark.shape[0];
                sf::Vector2f end = landmark.shape[1];
                
                // Use Bresenham's line algorithm to mark cells along the line
                sf::Vector2i startGrid = worldToGrid(start);
                sf::Vector2i endGrid = worldToGrid(end);
                
                int dx = abs(endGrid.x - startGrid.x);
                int dy = abs(endGrid.y - startGrid.y);
                int x = startGrid.x;
                int y = startGrid.y;
                int x_inc = (startGrid.x < endGrid.x) ? 1 : -1;
                int y_inc = (startGrid.y < endGrid.y) ? 1 : -1;
                int error = dx - dy;
                
                while (true) {
                    if (isValidGridCoordinate(sf::Vector2i(x, y))) {
                        updateCell(sf::Vector2i(x, y), true, landmark.confidence);
                    }
                    
                    if (x == endGrid.x && y == endGrid.y) break;
                    
                    int error2 = 2 * error;
                    if (error2 > -dy) {
                        error -= dy;
                        x += x_inc;
                    }
                    if (error2 < dx) {
                        error += dx;
                        y += y_inc;
                    }
                }
            } else {
                // Fallback to marking center position
                markCellOccupied(landmark.position, landmark.confidence);
            }
            break;
            
        case LandmarkType::CIRCLE:
            // Mark cells within the circular area
            {
                float radius = std::max(landmark.size.x, landmark.size.y) * 0.5f;
                sf::Vector2i centerGrid = worldToGrid(landmark.position);
                int gridRadius = static_cast<int>(std::ceil(radius / cellSize));
                
                for (int dy = -gridRadius; dy <= gridRadius; ++dy) {
                    for (int dx = -gridRadius; dx <= gridRadius; ++dx) {
                        sf::Vector2i gridPos(centerGrid.x + dx, centerGrid.y + dy);
                        if (isValidGridCoordinate(gridPos)) {
                            sf::Vector2f cellCenter = gridToWorld(gridPos);
                            float distance = std::sqrt(std::pow(cellCenter.x - landmark.position.x, 2) + 
                                                     std::pow(cellCenter.y - landmark.position.y, 2));
                            if (distance <= radius) {
                                updateCell(gridPos, true, landmark.confidence);
                            }
                        }
                    }
                }
            }
            break;
            
        case LandmarkType::CORNER:
        case LandmarkType::POLYGON:
        default:
            // For complex shapes, mark a rectangular area around the landmark
            {
                sf::Vector2f halfSize = landmark.size * 0.5f;
                sf::Vector2f topLeft = landmark.position - halfSize;
                sf::Vector2f bottomRight = landmark.position + halfSize;
                
                sf::Vector2i startGrid = worldToGrid(topLeft);
                sf::Vector2i endGrid = worldToGrid(bottomRight);
                
                for (int y = startGrid.y; y <= endGrid.y; ++y) {
                    for (int x = startGrid.x; x <= endGrid.x; ++x) {
                        sf::Vector2i gridPos(x, y);
                        if (isValidGridCoordinate(gridPos)) {
                            updateCell(gridPos, true, landmark.confidence);
                        }
                    }
                }
            }
            break;
    }
}

void MinimapGrid::markCellOccupied(sf::Vector2f worldPosition, float confidence) {
    // Only mark cells that are within reasonable world bounds (0 to 800x600)
    if (worldPosition.x < 0 || worldPosition.x > 800 || 
        worldPosition.y < 0 || worldPosition.y > 600) {
        return; // Skip out-of-bounds positions
    }
    
    sf::Vector2i gridPos = worldToGrid(worldPosition);
    if (isValidGridCoordinate(gridPos)) {
        updateCell(gridPos, true, confidence);
    }
}

void MinimapGrid::clearGrid() {
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            grid[y][x] = MinimapGridCell(); // Reset to default state
        }
    }
}

sf::Vector2i MinimapGrid::worldToGrid(sf::Vector2f worldPosition) const {
    // Convert world coordinates to grid coordinates
    sf::Vector2f relative = worldPosition - gridOrigin;
    sf::Vector2f gridCenter(gridWidth * cellSize * 0.5f, gridHeight * cellSize * 0.5f);
    sf::Vector2f adjusted = relative + gridCenter;
    
    int gridX = static_cast<int>(std::floor(adjusted.x / cellSize));
    int gridY = static_cast<int>(std::floor(adjusted.y / cellSize));
    
    // Clamp to grid bounds to prevent out-of-bounds access
    gridX = std::max(0, std::min(gridX, gridWidth - 1));
    gridY = std::max(0, std::min(gridY, gridHeight - 1));
    
    return sf::Vector2i(gridX, gridY);
}

sf::Vector2f MinimapGrid::gridToWorld(sf::Vector2i gridPosition) const {
    // Convert grid coordinates to world coordinates (cell center)
    sf::Vector2f gridCenter(gridWidth * cellSize * 0.5f, gridHeight * cellSize * 0.5f);
    sf::Vector2f worldPos;
    worldPos.x = (gridPosition.x + 0.5f) * cellSize - gridCenter.x + gridOrigin.x;
    worldPos.y = (gridPosition.y + 0.5f) * cellSize - gridCenter.y + gridOrigin.y;
    return worldPos;
}

bool MinimapGrid::isValidGridCoordinate(sf::Vector2i gridPosition) const {
    return gridPosition.x >= 0 && gridPosition.x < gridWidth &&
           gridPosition.y >= 0 && gridPosition.y < gridHeight;
}

void MinimapGrid::renderOnMinimap(sf::RenderWindow& window, sf::Vector2f minimapPosition, 
                                 sf::Vector2f minimapSize, sf::Vector2u worldSize) const {
    // Calculate scale factors from world to minimap
    float scaleX = minimapSize.x / worldSize.x;
    float scaleY = minimapSize.y / worldSize.y;
    
    // Draw cells first (red for occupied, green for explored)
    sf::RectangleShape cellShape;
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            const MinimapGridCell& cell = grid[y][x];
            
            // Draw occupied cells (red) and explored cells (green)
            if ((cell.state == GridCellState::OCCUPIED && cell.hasLandmark) || 
                cell.state == GridCellState::EXPLORED) {
                sf::Vector2f worldCellPos = gridToWorld(sf::Vector2i(x, y));
                
                // Scale cell position and size to minimap coordinates
                float scaledX = minimapPosition.x + (worldCellPos.x - cellSize * 0.5f) * scaleX;
                float scaledY = minimapPosition.y + (worldCellPos.y - cellSize * 0.5f) * scaleY;
                float scaledWidth = cellSize * scaleX;
                float scaledHeight = cellSize * scaleY;
                
                cellShape.setPosition(scaledX, scaledY);
                cellShape.setSize(sf::Vector2f(scaledWidth, scaledHeight));
                cellShape.setFillColor(getCellColor(cell));
                
                // Only draw cells that are within the minimap bounds
                if (scaledX < minimapPosition.x + minimapSize.x && scaledX + scaledWidth > minimapPosition.x &&
                    scaledY < minimapPosition.y + minimapSize.y && scaledY + scaledHeight > minimapPosition.y) {
                    window.draw(cellShape);
                }
            }
        }
    }
    
    // Draw reduced grid lines (only major grid lines for performance)
    sf::RectangleShape gridLine;
    gridLine.setFillColor(sf::Color(0, 0, 0, 100)); // Semi-transparent black
    
    // Draw only every 3rd vertical line for performance
    for (int x = 0; x <= gridWidth; x += 3) {
        sf::Vector2f worldLinePos = gridToWorld(sf::Vector2i(x, 0));
        worldLinePos.x -= cellSize * 0.5f;
        
        float scaledX = minimapPosition.x + (worldLinePos.x * scaleX);
        gridLine.setSize(sf::Vector2f(1.0f, minimapSize.y)); // Thinner lines for performance
        gridLine.setPosition(scaledX, minimapPosition.y);
        
        if (scaledX >= minimapPosition.x && scaledX <= minimapPosition.x + minimapSize.x) {
            window.draw(gridLine);
        }
    }
    
    // Draw only every 3rd horizontal line for performance
    for (int y = 0; y <= gridHeight; y += 3) {
        sf::Vector2f worldLinePos = gridToWorld(sf::Vector2i(0, y));
        worldLinePos.y -= cellSize * 0.5f;
        
        float scaledY = minimapPosition.y + (worldLinePos.y * scaleY);
        gridLine.setSize(sf::Vector2f(minimapSize.x, 1.0f)); // Thinner lines for performance
        gridLine.setPosition(minimapPosition.x, scaledY);
        
        if (scaledY >= minimapPosition.y && scaledY <= minimapPosition.y + minimapSize.y) {
            window.draw(gridLine);
        }
    }
}

GridCellState MinimapGrid::getCellState(sf::Vector2f worldPosition) const {
    sf::Vector2i gridPos = worldToGrid(worldPosition);
    if (isValidGridCoordinate(gridPos)) {
        return grid[gridPos.y][gridPos.x].state;
    }
    return GridCellState::UNKNOWN;
}

bool MinimapGrid::isOccupied(sf::Vector2f worldPosition) const {
    return getCellState(worldPosition) == GridCellState::OCCUPIED;
}

void MinimapGrid::updateCell(sf::Vector2i gridPos, bool occupied, float confidence) {
    MinimapGridCell& cell = grid[gridPos.y][gridPos.x];
    
    if (occupied) {
        cell.state = GridCellState::OCCUPIED;
        cell.hasLandmark = true;
        cell.landmarkCount++;
        cell.confidence = std::max(cell.confidence, confidence);
    } else {
        cell.state = GridCellState::EXPLORED;
        cell.hasLandmark = false;
        cell.confidence = confidence;
    }
}

sf::Color MinimapGrid::getCellColor(const MinimapGridCell& cell) const {
    if (cell.state == GridCellState::OCCUPIED && cell.hasLandmark) {
        // Bright red color with high opacity for better visibility
        sf::Uint8 alpha = static_cast<sf::Uint8>(std::min(255.0f, cell.confidence * 150 + 105)); // Higher base alpha
        
        // Use bright red for all occupied cells
        return sf::Color(255, 0, 0, alpha); // Pure bright red
    } else if (cell.state == GridCellState::EXPLORED) {
        // Green color for explored areas
        sf::Uint8 alpha = static_cast<sf::Uint8>(std::min(255.0f, cell.confidence * 100 + 80)); // Semi-transparent green
        return sf::Color(0, 255, 0, alpha); // Bright green
    }
    
    // Should not reach here for visible cells, but return transparent as fallback
    return sf::Color::Transparent;
}

void MinimapGrid::markCellExplored(sf::Vector2f worldPosition, float confidence) {
    sf::Vector2i gridPos = worldToGrid(worldPosition);
    if (isValidGridCoordinate(gridPos)) {
        MinimapGridCell& cell = grid[gridPos.y][gridPos.x];
        // Only mark as explored if it's not already occupied by a landmark
        if (cell.state != GridCellState::OCCUPIED) {
            cell.state = GridCellState::EXPLORED;
            cell.confidence = std::max(cell.confidence, confidence);
        }
    }
}
