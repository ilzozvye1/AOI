using System;
using System.Collections.Generic;
using System.Linq;

namespace OrthogonalLineRouting
{
    // Basic data structures
    public struct Point
    {
        public double X { get; set; }
        public double Y { get; set; }

        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }

        public static Point operator +(Point a, Point b) => new Point(a.X + b.X, a.Y + b.Y);
        public static Point operator -(Point a, Point b) => new Point(a.X - b.X, a.Y - b.Y);
        
        public double DistanceTo(Point other)
        {
            return Math.Sqrt(Math.Pow(X - other.X, 2) + Math.Pow(Y - other.Y, 2));
        }

        public double ManhattanDistanceTo(Point other)
        {
            return Math.Abs(X - other.X) + Math.Abs(Y - other.Y);
        }

        public override string ToString() => $"({X}, {Y})";
    }

    public struct Rectangle
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Width { get; set; }
        public double Height { get; set; }

        public Rectangle(double x, double y, double width, double height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        public double Left => X;
        public double Right => X + Width;
        public double Top => Y;
        public double Bottom => Y + Height;
        public Point Center => new Point(X + Width / 2, Y + Height / 2);

        public bool Contains(Point point)
        {
            return point.X >= X && point.X <= Right && point.Y >= Y && point.Y <= Bottom;
        }

        public bool Intersects(Rectangle other)
        {
            return !(other.Left > Right || other.Right < Left || other.Top > Bottom || other.Bottom < Top);
        }

        public Rectangle Expand(double margin)
        {
            return new Rectangle(X - margin, Y - margin, Width + 2 * margin, Height + 2 * margin);
        }
    }

    // Grid node for pathfinding
    public class GridNode
    {
        public Point Position { get; set; }
        public bool IsWalkable { get; set; } = true;
        public double GCost { get; set; } = double.MaxValue; // Distance from start
        public double HCost { get; set; } = 0; // Heuristic distance to end
        public double FCost => GCost + HCost; // Total cost
        public GridNode Parent { get; set; }
        public int TurnCount { get; set; } = 0; // Number of turns to reach this node

        public GridNode(Point position)
        {
            Position = position;
        }
    }

    // Main orthogonal line routing algorithm
    public class OrthogonalLineRouter
    {
        private const double TURN_PENALTY = 1.0; // Cost penalty for making turns
        private const double GRID_SPACING = 20.0; // Grid spacing for pathfinding
        private const double OBSTACLE_MARGIN = 10.0; // Margin around obstacles

        public List<Point> FindPath(Point start, Point end, List<Rectangle> obstacles = null)
        {
            obstacles = obstacles ?? new List<Rectangle>();
            
            // Create expanded grid based on start, end points and obstacles
            var gridBounds = CalculateGridBounds(start, end, obstacles);
            var grid = CreateGrid(gridBounds, obstacles);
            
            // Find closest walkable nodes to start and end points
            var startNode = FindClosestWalkableNode(grid, start);
            var endNode = FindClosestWalkableNode(grid, end);
            
            if (startNode == null || endNode == null)
                return new List<Point> { start, end }; // Fallback to direct line
            
            // Use A* algorithm to find path
            var path = AStar(startNode, endNode, grid);
            
            if (path == null || path.Count == 0)
                return new List<Point> { start, end }; // Fallback to direct line
            
            // Optimize path by reducing unnecessary turns
            var optimizedPath = OptimizePath(path, obstacles);
            
            // Apply center-line correction for better visual appearance
            var correctedPath = ApplyCenterLineCorrection(optimizedPath, start, end);
            
            return correctedPath;
        }

        private Rectangle CalculateGridBounds(Point start, Point end, List<Rectangle> obstacles)
        {
            var minX = Math.Min(start.X, end.X);
            var maxX = Math.Max(start.X, end.X);
            var minY = Math.Min(start.Y, end.Y);
            var maxY = Math.Max(start.Y, end.Y);

            foreach (var obstacle in obstacles)
            {
                minX = Math.Min(minX, obstacle.Left);
                maxX = Math.Max(maxX, obstacle.Right);
                minY = Math.Min(minY, obstacle.Top);
                maxY = Math.Max(maxY, obstacle.Bottom);
            }

            // Expand bounds with margin
            var margin = GRID_SPACING * 2;
            return new Rectangle(minX - margin, minY - margin, 
                               maxX - minX + 2 * margin, maxY - minY + 2 * margin);
        }

        private Dictionary<string, GridNode> CreateGrid(Rectangle bounds, List<Rectangle> obstacles)
        {
            var grid = new Dictionary<string, GridNode>();
            var expandedObstacles = obstacles.Select(o => o.Expand(OBSTACLE_MARGIN)).ToList();

            // Create ruler lines based on obstacles and boundaries
            var verticalLines = new HashSet<double>();
            var horizontalLines = new HashSet<double>();

            // Add boundary lines
            verticalLines.Add(bounds.Left);
            verticalLines.Add(bounds.Right);
            horizontalLines.Add(bounds.Top);
            horizontalLines.Add(bounds.Bottom);

            // Add obstacle-based ruler lines
            foreach (var obstacle in obstacles)
            {
                verticalLines.Add(obstacle.Left - OBSTACLE_MARGIN);
                verticalLines.Add(obstacle.Right + OBSTACLE_MARGIN);
                verticalLines.Add(obstacle.Center.X); // Center line
                
                horizontalLines.Add(obstacle.Top - OBSTACLE_MARGIN);
                horizontalLines.Add(obstacle.Bottom + OBSTACLE_MARGIN);
                horizontalLines.Add(obstacle.Center.Y); // Center line
            }

            // Create grid nodes at intersections of ruler lines
            foreach (var x in verticalLines)
            {
                foreach (var y in horizontalLines)
                {
                    var point = new Point(x, y);
                    var node = new GridNode(point);
                    
                    // Check if node is inside any expanded obstacle
                    node.IsWalkable = !expandedObstacles.Any(obs => obs.Contains(point));
                    
                    grid[GetNodeKey(point)] = node;
                }
            }

            return grid;
        }

        private GridNode FindClosestWalkableNode(Dictionary<string, GridNode> grid, Point target)
        {
            return grid.Values
                .Where(n => n.IsWalkable)
                .OrderBy(n => n.Position.ManhattanDistanceTo(target))
                .FirstOrDefault();
        }

        private List<Point> AStar(GridNode start, GridNode end, Dictionary<string, GridNode> grid)
        {
            var openSet = new SortedSet<GridNode>(Comparer<GridNode>.Create((a, b) =>
            {
                var fCompare = a.FCost.CompareTo(b.FCost);
                return fCompare != 0 ? fCompare : a.Position.X.CompareTo(b.Position.X);
            }));
            
            var closedSet = new HashSet<GridNode>();
            
            start.GCost = 0;
            start.HCost = start.Position.ManhattanDistanceTo(end.Position);
            openSet.Add(start);

            while (openSet.Count > 0)
            {
                var current = openSet.Min;
                openSet.Remove(current);
                closedSet.Add(current);

                if (current == end)
                {
                    return ReconstructPath(current);
                }

                foreach (var neighbor in GetNeighbors(current, grid))
                {
                    if (!neighbor.IsWalkable || closedSet.Contains(neighbor))
                        continue;

                    var tentativeGCost = current.GCost + current.Position.ManhattanDistanceTo(neighbor.Position);
                    
                    // Add turn penalty
                    var turnPenalty = CalculateTurnPenalty(current, neighbor);
                    tentativeGCost += turnPenalty;

                    if (tentativeGCost < neighbor.GCost)
                    {
                        neighbor.Parent = current;
                        neighbor.GCost = tentativeGCost;
                        neighbor.HCost = neighbor.Position.ManhattanDistanceTo(end.Position);
                        neighbor.TurnCount = current.TurnCount + (turnPenalty > 0 ? 1 : 0);

                        if (!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                    }
                }
            }

            return null; // No path found
        }

        private double CalculateTurnPenalty(GridNode current, GridNode neighbor)
        {
            if (current.Parent == null)
                return 0; // No turn at start

            var prevDirection = GetDirection(current.Parent.Position, current.Position);
            var currentDirection = GetDirection(current.Position, neighbor.Position);
            
            return prevDirection != currentDirection ? TURN_PENALTY : 0;
        }

        private Direction GetDirection(Point from, Point to)
        {
            var dx = to.X - from.X;
            var dy = to.Y - from.Y;
            
            if (Math.Abs(dx) > Math.Abs(dy))
                return dx > 0 ? Direction.Right : Direction.Left;
            else
                return dy > 0 ? Direction.Down : Direction.Up;
        }

        private List<GridNode> GetNeighbors(GridNode node, Dictionary<string, GridNode> grid)
        {
            var neighbors = new List<GridNode>();
            var directions = new[]
            {
                new Point(GRID_SPACING, 0),  // Right
                new Point(-GRID_SPACING, 0), // Left
                new Point(0, GRID_SPACING),  // Down
                new Point(0, -GRID_SPACING)  // Up
            };

            foreach (var dir in directions)
            {
                var neighborPos = node.Position + dir;
                var key = GetNodeKey(neighborPos);
                
                if (grid.TryGetValue(key, out var neighbor))
                    neighbors.Add(neighbor);
            }

            return neighbors;
        }

        private List<Point> ReconstructPath(GridNode endNode)
        {
            var path = new List<Point>();
            var current = endNode;

            while (current != null)
            {
                path.Add(current.Position);
                current = current.Parent;
            }

            path.Reverse();
            return path;
        }

        private List<Point> OptimizePath(List<Point> path, List<Rectangle> obstacles)
        {
            if (path.Count <= 2)
                return path;

            var optimized = new List<Point> { path[0] };

            for (int i = 1; i < path.Count - 1; i++)
            {
                var prev = optimized.Last();
                var current = path[i];
                var next = path[i + 1];

                // Check if we can skip the current point (remove unnecessary waypoints)
                if (!IsDirectLineBlocked(prev, next, obstacles))
                {
                    continue; // Skip this point
                }

                optimized.Add(current);
            }

            optimized.Add(path.Last());
            return optimized;
        }

        private bool IsDirectLineBlocked(Point start, Point end, List<Rectangle> obstacles)
        {
            foreach (var obstacle in obstacles)
            {
                if (LineIntersectsRectangle(start, end, obstacle.Expand(OBSTACLE_MARGIN)))
                    return true;
            }
            return false;
        }

        private bool LineIntersectsRectangle(Point start, Point end, Rectangle rect)
        {
            // Simple line-rectangle intersection check
            if (rect.Contains(start) || rect.Contains(end))
                return true;

            // Check if line crosses any edge of the rectangle
            return LineSegmentIntersect(start, end, new Point(rect.Left, rect.Top), new Point(rect.Right, rect.Top)) ||
                   LineSegmentIntersect(start, end, new Point(rect.Right, rect.Top), new Point(rect.Right, rect.Bottom)) ||
                   LineSegmentIntersect(start, end, new Point(rect.Right, rect.Bottom), new Point(rect.Left, rect.Bottom)) ||
                   LineSegmentIntersect(start, end, new Point(rect.Left, rect.Bottom), new Point(rect.Left, rect.Top));
        }

        private bool LineSegmentIntersect(Point p1, Point q1, Point p2, Point q2)
        {
            // Find the four orientations needed for general and special cases
            int o1 = Orientation(p1, q1, p2);
            int o2 = Orientation(p1, q1, q2);
            int o3 = Orientation(p2, q2, p1);
            int o4 = Orientation(p2, q2, q1);

            // General case
            if (o1 != o2 && o3 != o4)
                return true;

            // Special cases
            return (o1 == 0 && OnSegment(p1, p2, q1)) ||
                   (o2 == 0 && OnSegment(p1, q2, q1)) ||
                   (o3 == 0 && OnSegment(p2, p1, q2)) ||
                   (o4 == 0 && OnSegment(p2, q1, q2));
        }

        private int Orientation(Point p, Point q, Point r)
        {
            double val = (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);
            if (Math.Abs(val) < 1e-10) return 0;  // Collinear
            return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
        }

        private bool OnSegment(Point p, Point q, Point r)
        {
            return q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) &&
                   q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y);
        }

        private List<Point> ApplyCenterLineCorrection(List<Point> path, Point start, Point end)
        {
            if (path.Count < 3)
                return path;

            var corrected = new List<Point>(path);
            
            // Find potential center line alignments
            for (int i = 1; i < corrected.Count - 1; i++)
            {
                var prev = corrected[i - 1];
                var current = corrected[i];
                var next = corrected[i + 1];

                // Check if we can align this segment with a center line
                if (Math.Abs(prev.X - next.X) < 1e-10) // Vertical alignment possible
                {
                    var centerX = (prev.X + next.X) / 2;
                    corrected[i] = new Point(centerX, current.Y);
                }
                else if (Math.Abs(prev.Y - next.Y) < 1e-10) // Horizontal alignment possible
                {
                    var centerY = (prev.Y + next.Y) / 2;
                    corrected[i] = new Point(current.X, centerY);
                }
            }

            return corrected;
        }

        private string GetNodeKey(Point point)
        {
            return $"{point.X:F1},{point.Y:F1}";
        }
    }

    // Helper enum for directions
    public enum Direction
    {
        Up, Down, Left, Right
    }

    // Example usage program
    public class Program
    {
        public static void Main(string[] args)
        {
            var router = new OrthogonalLineRouter();
            
            // Example 1: Simple connection without obstacles
            var start1 = new Point(50, 50);
            var end1 = new Point(200, 150);
            var path1 = router.FindPath(start1, end1);
            
            Console.WriteLine("Example 1 - Simple path:");
            PrintPath(path1);
            Console.WriteLine();

            // Example 2: Connection with obstacles
            var start2 = new Point(50, 50);
            var end2 = new Point(250, 200);
            var obstacles = new List<Rectangle>
            {
                new Rectangle(100, 75, 80, 60),   // Obstacle 1
                new Rectangle(180, 120, 60, 80)   // Obstacle 2
            };
            
            var path2 = router.FindPath(start2, end2, obstacles);
            
            Console.WriteLine("Example 2 - Path with obstacles:");
            PrintPath(path2);
            Console.WriteLine();

            // Example 3: Complex scenario with multiple obstacles
            var start3 = new Point(30, 30);
            var end3 = new Point(300, 250);
            var complexObstacles = new List<Rectangle>
            {
                new Rectangle(80, 50, 60, 40),    // Obstacle 1
                new Rectangle(160, 80, 70, 50),   // Obstacle 2
                new Rectangle(120, 150, 80, 60),  // Obstacle 3
                new Rectangle(220, 100, 50, 80)   // Obstacle 4
            };
            
            var path3 = router.FindPath(start3, end3, complexObstacles);
            
            Console.WriteLine("Example 3 - Complex path with multiple obstacles:");
            PrintPath(path3);
        }

        private static void PrintPath(List<Point> path)
        {
            Console.WriteLine($"Path with {path.Count} points:");
            for (int i = 0; i < path.Count; i++)
            {
                Console.WriteLine($"  {i + 1}: {path[i]}");
                if (i > 0)
                {
                    var distance = path[i - 1].DistanceTo(path[i]);
                    Console.WriteLine($"      Distance from previous: {distance:F2}");
                }
            }
            
            var totalDistance = 0.0;
            for (int i = 1; i < path.Count; i++)
            {
                totalDistance += path[i - 1].DistanceTo(path[i]);
            }
            Console.WriteLine($"Total path distance: {totalDistance:F2}");
            Console.WriteLine($"Number of turns: {CountTurns(path)}");
        }

        private static int CountTurns(List<Point> path)
        {
            if (path.Count < 3)
                return 0;

            int turns = 0;
            for (int i = 1; i < path.Count - 1; i++)
            {
                var dir1 = GetDirection(path[i - 1], path[i]);
                var dir2 = GetDirection(path[i], path[i + 1]);
                if (dir1 != dir2)
                    turns++;
            }
            return turns;
        }

        private static Direction GetDirection(Point from, Point to)
        {
            var dx = to.X - from.X;
            var dy = to.Y - from.Y;
            
            if (Math.Abs(dx) > Math.Abs(dy))
                return dx > 0 ? Direction.Right : Direction.Left;
            else
                return dy > 0 ? Direction.Down : Direction.Up;
        }
    }
}