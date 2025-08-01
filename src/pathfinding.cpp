#include "pathfinding.hpp"
#include <algorithm>
#include <unordered_set>
#include <iostream>

Pathfinding::Pathfinding(int worldWidth, int worldHeight, int gridSize) 
    : worldWidth(worldWidth), worldHeight(worldHeight), gridSize(gridSize) {
    gridCols = worldWidth / gridSize;
    gridRows = worldHeight / gridSize;
    
    // initialize grid
    grid.resize(gridRows);
    for (int y = 0; y < gridRows; y++) {
        grid[y].resize(gridCols);
        for (int x = 0; x < gridCols; x++) {
            grid[y][x].x = x;
            grid[y][x].y = y;
            grid[y][x].occupied = false;
            grid[y][x].explored = false;
        }
    }
}

void Pathfinding::updateOccupancyGrid(const std::vector<Landmark>& detectedLandmarks, sf::Vector2f agentPos) {
    // mark agent's current area as explored
    sf::Vector2i agentGrid = worldToGrid(agentPos);
    int exploreRadius = 3; // explore 3 cells around agent
    
    for (int dy = -exploreRadius; dy <= exploreRadius; dy++) {
        for (int dx = -exploreRadius; dx <= exploreRadius; dx++) {
            int gridX = agentGrid.x + dx;
            int gridY = agentGrid.y + dy;
            if (isValidCell(gridX, gridY)) {
                grid[gridY][gridX].explored = true;
            }
        }
    }
    
    // Mark landmark cells as occupied
    for (const auto& landmark : detectedLandmarks) {
        sf::Vector2f landmarkPos = landmark.getObservedPos();
        sf::Vector2f landmarkSize = landmark.getShape().getSize();
        
        int minGridX = std::max(0, static_cast<int>(landmarkPos.x / gridSize));
        int maxGridX = std::min(gridCols - 1, static_cast<int>((landmarkPos.x + landmarkSize.x) / gridSize));
        int minGridY = std::max(0, static_cast<int>(landmarkPos.y / gridSize));
        int maxGridY = std::min(gridRows - 1, static_cast<int>((landmarkPos.y + landmarkSize.y) / gridSize));
        
        for (int gridY = minGridY; gridY <= maxGridY; gridY++) {
            for (int gridX = minGridX; gridX <= maxGridX; gridX++) {
                grid[gridY][gridX].occupied = true;
                grid[gridY][gridX].explored = true; // Occupied cells are considered explored
            }
        }
    }
}

std::vector<sf::Vector2f> Pathfinding::findPath(sf::Vector2f start, sf::Vector2f goal) {
    sf::Vector2i startGrid = worldToGrid(start);
    sf::Vector2i goalGrid = worldToGrid(goal);
    
    std::vector<sf::Vector2f> path;
    
    // Check if start and goal are valid
    if (!isValidCell(startGrid.x, startGrid.y) || !isValidCell(goalGrid.x, goalGrid.y) ||
        grid[goalGrid.y][goalGrid.x].occupied) {
        return path; // Return empty path
    }
    
    // A* algorithm
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> openSet;
    std::unordered_set<int> closedSet;
    std::unordered_map<int, std::unique_ptr<PathNode>> allNodes;
    
    auto getNodeKey = [this](int x, int y) { return y * gridCols + x; };
    
    // Create start node
    auto startNode = std::make_unique<PathNode>(startGrid.x, startGrid.y);
    startNode->gCost = 0;
    startNode->hCost = calculateHeuristic(startGrid.x, startGrid.y, goalGrid.x, goalGrid.y);
    startNode->fCost = startNode->gCost + startNode->hCost;
    
    openSet.push(*startNode);
    allNodes[getNodeKey(startGrid.x, startGrid.y)] = std::move(startNode);
    
    while (!openSet.empty()) {
        PathNode current = openSet.top();
        openSet.pop();
        
        int currentKey = getNodeKey(current.x, current.y);
        
        if (closedSet.find(currentKey) != closedSet.end()) {
            continue;
        }
        
        closedSet.insert(currentKey);
        
        // Check if we reached the goal
        if (current.x == goalGrid.x && current.y == goalGrid.y) {
            // Reconstruct path
            PathNode* node = allNodes[currentKey].get();
            while (node != nullptr) {
                path.push_back(gridToWorld(sf::Vector2i(node->x, node->y)));
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }
        
        // Check neighbors
        std::vector<sf::Vector2i> neighbors = getNeighbors(current.x, current.y);
        for (const auto& neighbor : neighbors) {
            int neighborKey = getNodeKey(neighbor.x, neighbor.y);
            
            if (closedSet.find(neighborKey) != closedSet.end() || 
                grid[neighbor.y][neighbor.x].occupied) {
                continue;
            }
            
            float tentativeGCost = current.gCost + 1.0f; // Assuming uniform cost
            
            if (allNodes.find(neighborKey) == allNodes.end()) {
                auto neighborNode = std::make_unique<PathNode>(neighbor.x, neighbor.y);
                neighborNode->gCost = tentativeGCost;
                neighborNode->hCost = calculateHeuristic(neighbor.x, neighbor.y, goalGrid.x, goalGrid.y);
                neighborNode->fCost = neighborNode->gCost + neighborNode->hCost;
                neighborNode->parent = allNodes[currentKey].get();
                
                openSet.push(*neighborNode);
                allNodes[neighborKey] = std::move(neighborNode);
            } else if (tentativeGCost < allNodes[neighborKey]->gCost) {
                allNodes[neighborKey]->gCost = tentativeGCost;
                allNodes[neighborKey]->fCost = allNodes[neighborKey]->gCost + allNodes[neighborKey]->hCost;
                allNodes[neighborKey]->parent = allNodes[currentKey].get();
                
                openSet.push(*allNodes[neighborKey]);
            }
        }
    }
    
    return path;
}

sf::Vector2f Pathfinding::findNearestUnexplored(sf::Vector2f agentPos) {
    sf::Vector2i agentGrid = worldToGrid(agentPos);
    
    // BFS to find nearest unexplored cell
    std::queue<sf::Vector2i> queue;
    std::vector<std::vector<bool>> visited(gridRows, std::vector<bool>(gridCols, false));
    
    queue.push(agentGrid);
    visited[agentGrid.y][agentGrid.x] = true;
    
    while (!queue.empty()) {
        sf::Vector2i current = queue.front();
        queue.pop();
        
        // Check if this cell is unexplored and not occupied
        if (!grid[current.y][current.x].explored && !grid[current.y][current.x].occupied) {
            return gridToWorld(current);
        }
        
        // Add neighbors to queue
        std::vector<sf::Vector2i> neighbors = getNeighbors(current.x, current.y);
        for (const auto& neighbor : neighbors) {
            if (!visited[neighbor.y][neighbor.x]) {
                visited[neighbor.y][neighbor.x] = true;
                queue.push(neighbor);
            }
        }
    }
    
    // If no unexplored cell found, return agent position
    return agentPos;
}

bool Pathfinding::isExplorationComplete() const {
    for (int y = 0; y < gridRows; y++) {
        for (int x = 0; x < gridCols; x++) {
            if (!grid[y][x].explored && !grid[y][x].occupied) {
                return false;
            }
        }
    }
    std::cout<<"exploration complete"<< std::endl;
    return true;
}

float Pathfinding::calculateHeuristic(int x1, int y1, int x2, int y2) {
    // Manhattan distance
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<sf::Vector2i> Pathfinding::getNeighbors(int x, int y) {
    std::vector<sf::Vector2i> neighbors;
    
    // 4-directional movement (up, down, left, right)
    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};
    
    for (int i = 0; i < 4; i++) {
        int newX = x + dx[i];
        int newY = y + dy[i];
        
        if (isValidCell(newX, newY)) {
            neighbors.push_back(sf::Vector2i(newX, newY));
        }
    }
    
    return neighbors;
}

bool Pathfinding::isValidCell(int x, int y) {
    return x >= 0 && x < gridCols && y >= 0 && y < gridRows;
}

sf::Vector2i Pathfinding::worldToGrid(sf::Vector2f worldPos) {
    return sf::Vector2i(
        std::max(0, std::min(gridCols - 1, static_cast<int>(worldPos.x / gridSize))),
        std::max(0, std::min(gridRows - 1, static_cast<int>(worldPos.y / gridSize)))
    );
}

sf::Vector2f Pathfinding::gridToWorld(sf::Vector2i gridPos) {
    return sf::Vector2f(
        gridPos.x * gridSize + gridSize / 2.0f,
        gridPos.y * gridSize + gridSize / 2.0f
    );
}
