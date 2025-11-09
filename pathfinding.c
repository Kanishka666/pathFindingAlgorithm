#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#define GRID_SIZE 10
#define OBSTACLE 1
#define FREE 0
#define PATH 2

// Structure for a node in the grid
typedef struct Node {
    int x, y;
    float g, h, f;
    struct Node* parent;
} Node;

// Structure for priority queue (min-heap)
typedef struct {
    Node** nodes;
    int size;
    int capacity;
} PriorityQueue;

// Structure to store path information
typedef struct {
    int** coordinates;
    int length;
    float cost;
} PathInfo;

// Global variable for statistics
int nodesExplored = 0;

// Function prototypes
PriorityQueue* createPQ(int capacity);
void pushPQ(PriorityQueue* pq, Node* node);
Node* popPQ(PriorityQueue* pq);
bool isEmptyPQ(PriorityQueue* pq);
void freePQ(PriorityQueue* pq);
float heuristic(int x1, int y1, int x2, int y2);
bool isValid(int x, int y, int grid[GRID_SIZE][GRID_SIZE]);
Node* createNode(int x, int y);
void printGrid(int grid[GRID_SIZE][GRID_SIZE]);
bool astar(int grid[GRID_SIZE][GRID_SIZE], int startX, int startY, int endX, int endY, PathInfo* pathInfo);
void printPathInfo(PathInfo* pathInfo);
void freePathInfo(PathInfo* pathInfo);

// Main function
int main() {
    int grid[GRID_SIZE][GRID_SIZE] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    
    int startX, startY, endX, endY;
    
    printf("=== A* Pathfinding Algorithm ===\n\n");
    printf("Grid Legend:\n");
    printf("  . = Free space\n");
    printf("  # = Obstacle\n");
    printf("  * = Path\n");
    printf("  S = Start\n");
    printf("  E = End\n\n");
    
    printf("Original Grid:\n");
    printGrid(grid);
    
    printf("\nEnter starting point (x y): ");
    scanf("%d %d", &startX, &startY);
    
    printf("Enter ending point (x y): ");
    scanf("%d %d", &endX, &endY);
    
    // Validate input
    if (startX < 0 || startX >= GRID_SIZE || startY < 0 || startY >= GRID_SIZE) {
        printf("\nError: Starting point (%d,%d) is out of bounds!\n", startX, startY);
        printf("Valid range: 0-%d for both x and y\n", GRID_SIZE - 1);
        return 1;
    }
    
    if (endX < 0 || endX >= GRID_SIZE || endY < 0 || endY >= GRID_SIZE) {
        printf("\nError: Ending point (%d,%d) is out of bounds!\n", endX, endY);
        printf("Valid range: 0-%d for both x and y\n", GRID_SIZE - 1);
        return 1;
    }
    
    if (grid[startX][startY] == OBSTACLE) {
        printf("\nError: Starting point (%d,%d) is on an obstacle!\n", startX, startY);
        return 1;
    }
    
    if (grid[endX][endY] == OBSTACLE) {
        printf("\nError: Ending point (%d,%d) is on an obstacle!\n", endX, endY);
        return 1;
    }
    
    // Run A* Algorithm with timing
    printf("\n=== Running A* Algorithm ===\n");
    
    clock_t start = clock();
    PathInfo pathInfo;
    nodesExplored = 0;
    
    bool found = astar(grid, startX, startY, endX, endY, &pathInfo);
    
    clock_t end = clock();
    double timeTaken = ((double)(end - start)) / CLOCKS_PER_SEC * 1000.0;
    
    if (found) {
        printf("\n✓ Shortest path found!\n");
        printf("\n--- Statistics ---\n");
        printf("Execution time: %.4f ms\n", timeTaken);
        printf("Nodes explored: %d\n", nodesExplored);
        printf("Path length: %d steps\n", pathInfo.length);
        printf("Path cost: %.2f\n", pathInfo.cost);
        
        printf("\nGrid with Shortest Path:\n");
        grid[startX][startY] = 'S';
        grid[endX][endY] = 'E';
        printGrid(grid);
        
        printf("\nPath coordinates:\n");
        printPathInfo(&pathInfo);
        
        freePathInfo(&pathInfo);
    } else {
        printf("\n✗ No path found!\n");
        printf("Execution time: %.4f ms\n", timeTaken);
        printf("Nodes explored: %d\n", nodesExplored);
    }
    
    return 0;
}

// Create a priority queue
PriorityQueue* createPQ(int capacity) {
    PriorityQueue* pq = (PriorityQueue*)malloc(sizeof(PriorityQueue));
    pq->nodes = (Node**)malloc(capacity * sizeof(Node*));
    pq->size = 0;
    pq->capacity = capacity;
    return pq;
}

// Push node into priority queue
void pushPQ(PriorityQueue* pq, Node* node) {
    if (pq->size >= pq->capacity) {
        pq->capacity *= 2;
        pq->nodes = (Node**)realloc(pq->nodes, pq->capacity * sizeof(Node*));
    }
    
    int i = pq->size++;
    while (i > 0 && node->f < pq->nodes[(i - 1) / 2]->f) {
        pq->nodes[i] = pq->nodes[(i - 1) / 2];
        i = (i - 1) / 2;
    }
    pq->nodes[i] = node;
}

// Pop node with minimum f value
Node* popPQ(PriorityQueue* pq) {
    if (pq->size == 0) return NULL;
    
    Node* minNode = pq->nodes[0];
    Node* lastNode = pq->nodes[--pq->size];
    
    int i = 0;
    while (2 * i + 1 < pq->size) {
        int child = 2 * i + 1;
        if (child + 1 < pq->size && pq->nodes[child + 1]->f < pq->nodes[child]->f)
            child++;
        
        if (lastNode->f <= pq->nodes[child]->f) break;
        
        pq->nodes[i] = pq->nodes[child];
        i = child;
    }
    pq->nodes[i] = lastNode;
    
    return minNode;
}

// Check if priority queue is empty
bool isEmptyPQ(PriorityQueue* pq) {
    return pq->size == 0;
}

// Free priority queue
void freePQ(PriorityQueue* pq) {
    free(pq->nodes);
    free(pq);
}

// Calculate heuristic (Euclidean distance)
float heuristic(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Check if position is valid
bool isValid(int x, int y, int grid[GRID_SIZE][GRID_SIZE]) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && grid[x][y] != OBSTACLE;
}

// Create a new node
Node* createNode(int x, int y) {
    Node* node = (Node*)malloc(sizeof(Node));
    node->x = x;
    node->y = y;
    node->g = 0;
    node->h = 0;
    node->f = 0;
    node->parent = NULL;
    return node;
}

// Print the grid
void printGrid(int grid[GRID_SIZE][GRID_SIZE]) {
    printf("   ");
    for (int j = 0; j < GRID_SIZE; j++) {
        printf("%d ", j);
    }
    printf("\n");
    
    for (int i = 0; i < GRID_SIZE; i++) {
        printf("%d  ", i);
        for (int j = 0; j < GRID_SIZE; j++) {
            if (grid[i][j] == OBSTACLE)
                printf("# ");
            else if (grid[i][j] == PATH)
                printf("* ");
            else if (grid[i][j] == 'S')
                printf("S ");
            else if (grid[i][j] == 'E')
                printf("E ");
            else
                printf(". ");
        }
        printf("\n");
    }
}

// A* pathfinding algorithm
bool astar(int grid[GRID_SIZE][GRID_SIZE], int startX, int startY, int endX, int endY, PathInfo* pathInfo) {
    bool closedList[GRID_SIZE][GRID_SIZE] = {false};
    Node* nodeGrid[GRID_SIZE][GRID_SIZE] = {NULL};
    
    Node* startNode = createNode(startX, startY);
    startNode->g = 0;
    startNode->h = heuristic(startX, startY, endX, endY);
    startNode->f = startNode->g + startNode->h;
    nodeGrid[startX][startY] = startNode;
    
    PriorityQueue* openList = createPQ(100);
    pushPQ(openList, startNode);
    
    // Directions: up, down, left, right, and diagonals
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    
    while (!isEmptyPQ(openList)) {
        Node* current = popPQ(openList);
        int x = current->x;
        int y = current->y;
        
        closedList[x][y] = true;
        nodesExplored++;
        
        // Check if we reached the destination
        if (x == endX && y == endY) {
            // Count path length
            int length = 0;
            Node* temp = current;
            while (temp != NULL) {
                length++;
                temp = temp->parent;
            }
            
            // Store path
            pathInfo->length = length;
            pathInfo->cost = current->g;
            pathInfo->coordinates = (int**)malloc(length * sizeof(int*));
            
            temp = current;
            for (int i = length - 1; i >= 0; i--) {
                pathInfo->coordinates[i] = (int*)malloc(2 * sizeof(int));
                pathInfo->coordinates[i][0] = temp->x;
                pathInfo->coordinates[i][1] = temp->y;
                
                if (!(temp->x == startX && temp->y == startY) && !(temp->x == endX && temp->y == endY)) {
                    grid[temp->x][temp->y] = PATH;
                }
                temp = temp->parent;
            }
            
            // Free memory
            for (int i = 0; i < GRID_SIZE; i++) {
                for (int j = 0; j < GRID_SIZE; j++) {
                    if (nodeGrid[i][j] != NULL) {
                        free(nodeGrid[i][j]);
                    }
                }
            }
            freePQ(openList);
            return true;
        }
        
        // Explore neighbors
        for (int i = 0; i < 8; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            if (isValid(newX, newY, grid) && !closedList[newX][newY]) {
                float newG = current->g + ((i < 4) ? 1.0 : 1.414); // Cost is higher for diagonal
                
                if (nodeGrid[newX][newY] == NULL) {
                    Node* neighbor = createNode(newX, newY);
                    neighbor->g = newG;
                    neighbor->h = heuristic(newX, newY, endX, endY);
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    nodeGrid[newX][newY] = neighbor;
                    pushPQ(openList, neighbor);
                } else if (newG < nodeGrid[newX][newY]->g) {
                    nodeGrid[newX][newY]->g = newG;
                    nodeGrid[newX][newY]->f = newG + nodeGrid[newX][newY]->h;
                    nodeGrid[newX][newY]->parent = current;
                }
            }
        }
    }
    
    // Free memory if no path found
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (nodeGrid[i][j] != NULL) {
                free(nodeGrid[i][j]);
            }
        }
    }
    freePQ(openList);
    return false;
}

// Print path information
void printPathInfo(PathInfo* pathInfo) {
    for (int i = 0; i < pathInfo->length; i++) {
        printf("(%d,%d)", pathInfo->coordinates[i][0], pathInfo->coordinates[i][1]);
        if (i < pathInfo->length - 1) printf(" -> ");
        if ((i + 1) % 5 == 0 && i < pathInfo->length - 1) printf("\n");
    }
    printf("\n");
}

// Free path information
void freePathInfo(PathInfo* pathInfo) {
    for (int i = 0; i < pathInfo->length; i++) {
        free(pathInfo->coordinates[i]);
    }
    free(pathInfo->coordinates);
}