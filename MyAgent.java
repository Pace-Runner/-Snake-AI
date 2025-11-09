import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;
import za.ac.wits.snake.DevelopmentAgent;

public class MyAgent extends DevelopmentAgent {
    // Movement directions - keeping it simple with array indices
    private static final int[] dx = {0, 0, -1, 1};
    private static final int[] dy = {-1, 1, 0, 0};
    private static final int UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3;
    
    // Game state variables - need to track everything important
    private int boardWidth, boardHeight;
    private int mySnakeNum;
    private List<Snake> snakes;
    private Point apple;
    private int appleValue; // apples lose value over time, so this matters
    private int timestep = 0; // tracking time for apple value calculations
    private boolean[][] grid; // collision detection grid - true means occupied

    public static void main(String args[]) {
        MyAgent agent = new MyAgent();
        MyAgent.start(agent, args);
    }

    @Override
    public void run() {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(System.in))) {
            // Initial setup - get board dimensions and number of snakes
            String initString = br.readLine();
            String[] temp = initString.split(" ");
            int nSnakes = Integer.parseInt(temp[0]);
            boardWidth = Integer.parseInt(temp[1]);
            boardHeight = Integer.parseInt(temp[2]);

            // Main game loop - keep playing until game over
            while (true) {
                String line = br.readLine();
                if (line.contains("Game Over")) {
                    break;
                }

                // Parse apple location - this is where we want to go (maybe)
                String[] appleCoords = line.split(" ");
                apple = new Point(Integer.parseInt(appleCoords[0]), Integer.parseInt(appleCoords[1]));
                
                // Calculate current apple value - they decay over time
                double rawValue = 5.0 - 0.1 * timestep;
                appleValue = (int)Math.ceil(rawValue);
                
                // Get my snake number and all snake states
                mySnakeNum = Integer.parseInt(br.readLine());
                snakes = new ArrayList<>();
                
                for (int i = 0; i < nSnakes; i++) {
                    String snakeLine = br.readLine();
                    Snake snake = parseSnake(snakeLine, i);
                    snakes.add(snake);
                }
                
                // Get my snake - if I'm dead or something's wrong, just move randomly
                Snake mySnake = snakes.get(mySnakeNum);
                if (!mySnake.alive || mySnake.body.isEmpty()) {
                    System.out.println(new Random().nextInt(4));
                    timestep++;
                    continue;
                }
                
                // Build the collision grid and decide on the best move
                buildCollisionGrid();
                int move = decideBestMove(mySnake);
                System.out.println(move);
                
                timestep++;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Parse snake data from input string - handles both alive and dead snakes
    private Snake parseSnake(String snakeLine, int index) {
        String[] parts = snakeLine.split(" ");
        boolean alive = parts[0].equals("alive");
        
        // Dead snakes are easy - just mark them as dead
        if (!alive) {
            return new Snake(index, false, 0, 0, new ArrayList<>());
        }
        
        // For alive snakes, parse their body segments
        int length = Integer.parseInt(parts[1]);
        int kills = Integer.parseInt(parts[2]);
        
        List<Point> body = new ArrayList<>();
        for (int i = 3; i < parts.length; i++) {
            String[] coords = parts[i].split(",");
            body.add(new Point(Integer.parseInt(coords[0]), Integer.parseInt(coords[1])));
        }
        
        return new Snake(index, alive, length, kills, body);
    }

    // Build a grid showing where collisions would happen - true means "don't go here"
    private void buildCollisionGrid() {
        grid = new boolean[boardWidth][boardHeight];
        
        // Mark all living snakes on the grid
        for (Snake snake : snakes) {
            if (snake.alive && !snake.body.isEmpty()) {
                markSnakeOnGrid(snake);
            }
        }
    }

    // Mark a single snake on the collision grid
    private void markSnakeOnGrid(Snake snake) {
        // Handle edge case of really short snakes
        if (snake.body.size() < 2) {
            if (!snake.body.isEmpty()) {
                Point p = snake.body.get(0);
                if (isValid(p.x, p.y)) {
                    grid[p.x][p.y] = true;
                }
            }
            return;
        }

        // Mark each body segment and fill in the gaps between them
        for (int i = 0; i < snake.body.size() - 1; i++) {
            Point from = snake.body.get(i);
            Point to = snake.body.get(i + 1);
            
            if (isValid(from.x, from.y)) {
                grid[from.x][from.y] = true;
            }
            
            // Fill in the line between body segments
            fillSegment(from, to);
        }
        
        // Don't forget the tail
        Point tail = snake.body.get(snake.body.size() - 1);
        if (isValid(tail.x, tail.y)) {
            grid[tail.x][tail.y] = true;
        }
    }

    // Fill in all the spaces between two body segments
    private void fillSegment(Point from, Point to) {
        if (from.x == to.x) {
            // Vertical line
            int startY = Math.min(from.y, to.y);
            int endY = Math.max(from.y, to.y);
            for (int y = startY + 1; y < endY; y++) {
                if (isValid(from.x, y)) {
                    grid[from.x][y] = true;
                }
            }
        } else if (from.y == to.y) {
            // Horizontal line
            int startX = Math.min(from.x, to.x);
            int endX = Math.max(from.x, to.x);
            for (int x = startX + 1; x < endX; x++) {
                if (isValid(x, from.y)) {
                    grid[x][from.y] = true;
                }
            }
        }
    }

    // This is where the magic happens - decide what move to make
    private int decideBestMove(Snake mySnake) {
        Point head = mySnake.body.get(0);
        
        // First, find all the moves that won't kill me immediately
        List<Integer> safeMoves = new ArrayList<>();
        for (int dir = 0; dir < 4; dir++) {
            if (isMovesSafe(head, dir)) {
                safeMoves.add(dir);
            }
        }
        
        // If no safe moves, well... pray and pick randomly
        if (safeMoves.isEmpty()) {
            return new Random().nextInt(4);
        }
        
        // If only one safe move, that's our choice
        if (safeMoves.size() == 1) {
            return safeMoves.get(0);
        }
        
        // Multiple safe moves - now we need to be smart about it
        return chooseBestSafeMove(head, safeMoves, mySnake);
    }

    // Check if a move is safe - won't hit walls, snakes, or enemy heads
    private boolean isMovesSafe(Point head, int direction) {
        Point newPos = new Point(head.x + dx[direction], head.y + dy[direction]);
        
        // Don't go out of bounds
        if (!isValid(newPos.x, newPos.y)) {
            return false;
        }
        
        // Don't hit existing snake bodies
        if (grid[newPos.x][newPos.y]) {
            return false;
        }
        
        // Don't move where enemy snakes might move (head-to-head collision)
        for (Snake enemy : snakes) {
            if (enemy.index == mySnakeNum || !enemy.alive || enemy.body.isEmpty()) {
                continue;
            }
            
            Point enemyHead = enemy.body.get(0);
            
            // Check all possible enemy moves
            for (int enemyDir = 0; enemyDir < 4; enemyDir++) {
                Point enemyNewPos = new Point(enemyHead.x + dx[enemyDir], enemyHead.y + dy[enemyDir]);
                if (enemyNewPos.equals(newPos)) {
                    return false; // Potential head-to-head collision
                }
            }
        }
        
        return true;
    }

    // Pick the best move from our safe options using scoring
    private int chooseBestSafeMove(Point head, List<Integer> safeMoves, Snake mySnake) {
        int bestMove = safeMoves.get(0);
        double bestScore = Double.NEGATIVE_INFINITY;
        
        // Score each safe move and pick the highest
        for (int move : safeMoves) {
            double score = evaluateMove(head, move, mySnake);
            if (score > bestScore) {
                bestScore = score;
                bestMove = move;
            }
        }
        
        return bestMove;
    }

    // This is where we get fancy - score moves based on multiple factors
    private double evaluateMove(Point head, int direction, Snake mySnake) {
        Point newPos = new Point(head.x + dx[direction], head.y + dy[direction]);
        double score = 0;
        
        // Should we go for the apple or not?
        if (shouldPursueApple()) {
            // Apple pursuit mode - pathfind to the apple
            List<Point> path = findPathAStar(newPos, apple);
            
            if (path != null && path.size() > 0) {
                // Shorter paths are better
                score += 20000.0 / Math.max(1, path.size());
                
                // Bonus if we're following the optimal path
                List<Point> currentPath = findPathAStar(head, apple);
                if (currentPath != null && currentPath.size() > 1) {
                    Point nextOptimalStep = currentPath.get(1);
                    if (newPos.equals(nextOptimalStep)) {
                        score += 15000; // Big bonus for staying on optimal path
                    }
                }
                
                // Score based on apple value - better apples worth more risk
                if (appleValue > 0) {
                    score += appleValue * 2000;
                } else if (appleValue >= -1) {
                    score += 1000; // Still okay
                } else if (appleValue >= -3) {
                    score += 500; // Meh
                } else {
                    score += 200; // Really not great but better than nothing
                }
                
                // Bonus for being close to apple
                double distToApple = manhattanDistance(newPos, apple);
                if (distToApple <= 3) {
                    score += 1000;
                } else if (distToApple <= 6) {
                    score += 500;
                }
                
            } else {
                // Can't reach apple from this position - bad
                score -= 5000;
            }
            
            // Always prefer moves that keep our options open
            int openNeighbors = 0;
            for (int dir = 0; dir < 4; dir++) {
                Point neighbor = new Point(newPos.x + dx[dir], newPos.y + dy[dir]);
                if (isValid(neighbor.x, neighbor.y) && !grid[neighbor.x][neighbor.y]) {
                    openNeighbors++;
                }
            }
            score += openNeighbors * 25;
            
        } else {
            // Not pursuing apple - focus on survival and positioning
            double distToApple = manhattanDistance(newPos, apple);
            double currentDistToApple = manhattanDistance(head, apple);
            
            // Still prefer to get closer to apple, but less aggressively
            if (distToApple < currentDistToApple) {
                score += 5000;
            } else if (distToApple > currentDistToApple) {
                score -= 2000;
            }
            
            // Really value having escape routes when not chasing apples
            int openNeighbors = 0;
            for (int dir = 0; dir < 4; dir++) {
                Point neighbor = new Point(newPos.x + dx[dir], newPos.y + dy[dir]);
                if (isValid(neighbor.x, neighbor.y) && !grid[neighbor.x][neighbor.y]) {
                    openNeighbors++;
                }
            }
            score += openNeighbors * 100;
            
            // Staying near walls can be good for defensive play
            if (isNearWallOrBoundary(newPos)) {
                score += 200;
            }
            
            // Prefer staying towards the center generally
            double centerX = boardWidth / 2.0;
            double centerY = boardHeight / 2.0;
            double distFromCenter = manhattanDistance(newPos, new Point((int)centerX, (int)centerY));
            score += (Math.max(boardWidth, boardHeight) - distFromCenter) * 5;
        }
        
        // Always check that we won't trap ourselves
        int reachableSpaces = countReachableSpaces(newPos, mySnake.length + 8);
        if (reachableSpaces < mySnake.length / 3) {
            score -= 3000; // This would be bad - we'd get trapped
        } else {
            score += Math.min(reachableSpaces, 30);
        }
        
        // When not chasing apples, add exploration bonus
        if (!shouldPursueApple()) {
            score += evaluateExplorationMove(newPos, mySnake);
        }
        
        return score;
    }

    // Decide whether the apple is worth going for
    private boolean shouldPursueApple() {
        Snake mySnake = snakes.get(mySnakeNum);
        Point myHead = mySnake.body.get(0);
        
        // If we can't even reach it, don't bother
        List<Point> path = findPathAStar(myHead, apple);
        if (path == null) {
            return false;
        }
        
        // If apple value is terrible and we're small, skip it
        if (appleValue <= -15 && mySnake.length <= 3) {
            return false;
        }
        
        return true;
    }

    // Score moves when we're in exploration/survival mode
    private double evaluateExplorationMove(Point newPos, Snake mySnake) {
        double explorationScore = 0;
        
        // Look for open areas around this position
        int localOpenSpace = 0;
        for (int dx = -3; dx <= 3; dx++) {
            for (int dy = -3; dy <= 3; dy++) {
                int checkX = newPos.x + dx;
                int checkY = newPos.y + dy;
                if (isValid(checkX, checkY) && !grid[checkX][checkY]) {
                    localOpenSpace++;
                }
            }
        }
        
        explorationScore += localOpenSpace * 5;
        
        // Wall-hugging can be a good defensive strategy
        if (isNearWallOrBoundary(newPos)) {
            explorationScore += 150;
        }
        
        // Encourage movement over staying still
        Point head = mySnake.body.get(0);
        double movementDistance = manhattanDistance(head, newPos);
        if (movementDistance > 0) {
            explorationScore += 100;
        }
        
        return explorationScore;
    }
    
    // Check if we're near the edge of the board
    private boolean isNearWallOrBoundary(Point pos) {
        return pos.x <= 2 || pos.x >= boardWidth - 3 || pos.y <= 2 || pos.y >= boardHeight - 3;
    }

    // A* pathfinding - find the shortest path between two points
    private List<Point> findPathAStar(Point start, Point goal) {
        if (start.equals(goal)) return Arrays.asList(start);
        
        PriorityQueue<AStarNode> openList = new PriorityQueue<>();
        Set<Point> closedSet = new HashSet<>();
        Map<Point, AStarNode> allNodes = new HashMap<>();
        
        AStarNode startNode = new AStarNode(start, 0, manhattanDistance(start, goal), null);
        openList.add(startNode);
        allNodes.put(start, startNode);
        
        while (!openList.isEmpty()) {
            AStarNode current = openList.poll();
            
            // Found the goal!
            if (current.pos.equals(goal)) {
                return reconstructPath(current);
            }
            
            closedSet.add(current.pos);
            
            // Check all neighboring positions
            for (int dir = 0; dir < 4; dir++) {
                Point neighbor = new Point(current.pos.x + dx[dir], current.pos.y + dy[dir]);
                
                // Skip invalid or already processed positions
                if (!isValid(neighbor.x, neighbor.y) || closedSet.contains(neighbor)) {
                    continue;
                }
                
                // Can't go through obstacles (unless it's the goal)
                if (grid[neighbor.x][neighbor.y] && !neighbor.equals(goal)) {
                    continue;
                }
                
                double tentativeG = current.g + 1;
                AStarNode neighborNode = allNodes.get(neighbor);
                
                if (neighborNode == null) {
                    // New node
                    neighborNode = new AStarNode(neighbor, tentativeG, manhattanDistance(neighbor, goal), current);
                    allNodes.put(neighbor, neighborNode);
                    openList.add(neighborNode);
                } else if (tentativeG < neighborNode.g) {
                    // Better path to existing node
                    neighborNode.g = tentativeG;
                    neighborNode.f = tentativeG + neighborNode.h;
                    neighborNode.parent = current;
                }
            }
        }
        
        // No path found
        return null;
    }

    // Trace back the path from goal to start
    private List<Point> reconstructPath(AStarNode node) {
        List<Point> path = new ArrayList<>();
        while (node != null) {
            path.add(0, node.pos);
            node = node.parent;
        }
        return path;
    }

    // Count how many spaces we can reach from a position (flood fill)
    private int countReachableSpaces(Point start, int maxCount) {
        Set<Point> reachable = new HashSet<>();
        Queue<Point> queue = new LinkedList<>();
        queue.offer(start);
        reachable.add(start);
        
        // Breadth-first search to find all reachable spaces
        while (!queue.isEmpty() && reachable.size() < maxCount) {
            Point current = queue.poll();
            
            for (int dir = 0; dir < 4; dir++) {
                Point next = new Point(current.x + dx[dir], current.y + dy[dir]);
                
                if (isValid(next.x, next.y) && !grid[next.x][next.y] && !reachable.contains(next)) {
                    reachable.add(next);
                    queue.offer(next);
                }
            }
        }
        
        return reachable.size();
    }

    // Simple bounds checking
    private boolean isValid(int x, int y) {
        return x >= 0 && x < boardWidth && y >= 0 && y < boardHeight;
    }

    // Manhattan distance - good enough for grid-based games
    private double manhattanDistance(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    // Snake data structure - keeps track of everything about a snake
    private static class Snake {
        int index;
        boolean alive;
        int length;
        int kills;
        List<Point> body; // head is at index 0
        
        Snake(int index, boolean alive, int length, int kills, List<Point> body) {
            this.index = index;
            this.alive = alive;
            this.length = length;
            this.kills = kills;
            this.body = body;
        }
    }

    // Simple point class for coordinates
    private static class Point {
        int x, y;
        
        Point(int x, int y) {
            this.x = x;
            this.y = y;
        }
        
        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof Point)) return false;
            Point p = (Point) obj;
            return x == p.x && y == p.y;
        }
        
        @Override
        public int hashCode() {
            return x * 1000 + y; // Simple hash function
        }
    }

    // A* node for pathfinding
    private static class AStarNode implements Comparable<AStarNode> {
        Point pos;
        double g, h, f; // g = cost from start, h = heuristic to goal, f = g + h
        AStarNode parent; // for path reconstruction
        
        AStarNode(Point pos, double g, double h, AStarNode parent) {
            this.pos = pos;
            this.g = g;
            this.h = h;
            this.f = g + h;
            this.parent = parent;
        }
        
        @Override
        public int compareTo(AStarNode other) {
            return Double.compare(this.f, other.f); // Priority queue will use lowest f
        }
    }
}