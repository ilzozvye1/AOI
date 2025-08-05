using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace FlowchartOrthogonalRouting
{
    /// <summary>
    /// 表示一个矩形形状
    /// </summary>
    public class Shape
    {
        public Rectangle Bounds { get; set; }
        public Point ConnectionPoint { get; set; }
        public ConnectionSide ConnectionSide { get; set; }
        
        public Shape(Rectangle bounds, ConnectionSide side, double connectionRatio = 0.5)
        {
            Bounds = bounds;
            ConnectionSide = side;
            ConnectionPoint = CalculateConnectionPoint(side, connectionRatio);
        }
        
        private Point CalculateConnectionPoint(ConnectionSide side, double ratio)
        {
            ratio = Math.Max(0, Math.Min(1, ratio));
            
            switch (side)
            {
                case ConnectionSide.Top:
                    return new Point(Bounds.Left + (int)(Bounds.Width * ratio), Bounds.Top);
                case ConnectionSide.Bottom:
                    return new Point(Bounds.Left + (int)(Bounds.Width * ratio), Bounds.Bottom);
                case ConnectionSide.Left:
                    return new Point(Bounds.Left, Bounds.Top + (int)(Bounds.Height * ratio));
                case ConnectionSide.Right:
                    return new Point(Bounds.Right, Bounds.Top + (int)(Bounds.Height * ratio));
                default:
                    return new Point(Bounds.Left + Bounds.Width / 2, Bounds.Top + Bounds.Height / 2);
            }
        }
        
        /// <summary>
        /// 获取扩展边界（用于避免连线穿过形状）
        /// </summary>
        public Rectangle GetOuterBounds(int margin = 20)
        {
            return new Rectangle(
                Bounds.X - margin,
                Bounds.Y - margin,
                Bounds.Width + 2 * margin,
                Bounds.Height + 2 * margin);
        }
    }

    /// <summary>
    /// 连接点位置枚举
    /// </summary>
    public enum ConnectionSide
    {
        Top, Bottom, Left, Right
    }

    /// <summary>
    /// 网格节点
    /// </summary>
    public class GridNode
    {
        public Point Position { get; set; }
        public bool IsBlocked { get; set; }
        public double GCost { get; set; } // 从起点到当前节点的实际代价
        public double HCost { get; set; } // 从当前节点到终点的启发式代价
        public double FCost => GCost + HCost; // 总代价
        public GridNode? Parent { get; set; }
        public NodeType Type { get; set; }
        
        public GridNode(Point position, NodeType type = NodeType.Normal)
        {
            Position = position;
            Type = type;
            IsBlocked = false;
            GCost = double.MaxValue;
            HCost = 0;
            Parent = null;
        }
        
        public void Reset()
        {
            GCost = double.MaxValue;
            HCost = 0;
            Parent = null;
        }
    }

    /// <summary>
    /// 节点类型
    /// </summary>
    public enum NodeType
    {
        Normal,
        Start,
        End,
        Corner,
        EdgeMiddle,
        Center
    }

    /// <summary>
    /// 正交连线生成器
    /// </summary>
    public class OrthogonalLineGenerator
    {
        private const int TURN_PENALTY = 10; // 转弯惩罚
        private const int GRID_SPACING = 10; // 网格间距
        private readonly int _margin;
        
        private Dictionary<Point, GridNode> _gridNodes;
        private List<Shape> _shapes;
        
        public OrthogonalLineGenerator(int margin = 20)
        {
            _margin = margin;
            _gridNodes = new Dictionary<Point, GridNode>();
            _shapes = new List<Shape>();
        }
        
        /// <summary>
        /// 生成两个形状之间的正交连线
        /// </summary>
        public List<Point> GenerateRoute(Shape sourceShape, Shape targetShape, List<Shape> obstacles = null)
        {
            _shapes.Clear();
            _shapes.Add(sourceShape);
            _shapes.Add(targetShape);
            
            if (obstacles != null)
            {
                _shapes.AddRange(obstacles);
            }
            
            // 1. 构建网格
            BuildGrid();
            
            // 2. 标记被阻挡的节点
            MarkBlockedNodes();
            
            // 3. 使用A*算法寻找路径
            var path = FindPathAStar(sourceShape.ConnectionPoint, targetShape.ConnectionPoint);
            
            // 4. 优化路径（减少转弯点）
            if (path != null && path.Count > 0)
            {
                path = OptimizePath(path);
                path = ApplyCenterLineCorrection(path, sourceShape, targetShape);
            }
            
            return path ?? new List<Point>();
        }
        
        /// <summary>
        /// 构建网格点
        /// </summary>
        private void BuildGrid()
        {
            _gridNodes.Clear();
            
            // 收集所有相关的x和y坐标
            var xCoords = new HashSet<int>();
            var yCoords = new HashSet<int>();
            
            foreach (var shape in _shapes)
            {
                var outerBounds = shape.GetOuterBounds(_margin);
                
                // 添加形状边界坐标
                xCoords.Add(outerBounds.Left);
                xCoords.Add(outerBounds.Right);
                xCoords.Add(outerBounds.Left + outerBounds.Width / 2); // 中心线
                
                yCoords.Add(outerBounds.Top);
                yCoords.Add(outerBounds.Bottom);
                yCoords.Add(outerBounds.Top + outerBounds.Height / 2); // 中心线
                
                // 添加连接点坐标
                xCoords.Add(shape.ConnectionPoint.X);
                yCoords.Add(shape.ConnectionPoint.Y);
            }
            
            // 在坐标之间添加中间点以提高路径质量
            var sortedX = xCoords.OrderBy(x => x).ToList();
            var sortedY = yCoords.OrderBy(y => y).ToList();
            
            for (int i = 0; i < sortedX.Count - 1; i++)
            {
                int midX = (sortedX[i] + sortedX[i + 1]) / 2;
                if (Math.Abs(sortedX[i + 1] - sortedX[i]) > GRID_SPACING * 2)
                {
                    xCoords.Add(midX);
                }
            }
            
            for (int i = 0; i < sortedY.Count - 1; i++)
            {
                int midY = (sortedY[i] + sortedY[i + 1]) / 2;
                if (Math.Abs(sortedY[i + 1] - sortedY[i]) > GRID_SPACING * 2)
                {
                    yCoords.Add(midY);
                }
            }
            
            // 创建网格节点
            foreach (var x in xCoords)
            {
                foreach (var y in yCoords)
                {
                    var point = new Point(x, y);
                    var nodeType = DetermineNodeType(point);
                    _gridNodes[point] = new GridNode(point, nodeType);
                }
            }
        }
        
        /// <summary>
        /// 确定节点类型
        /// </summary>
        private NodeType DetermineNodeType(Point point)
        {
            foreach (var shape in _shapes)
            {
                if (point == shape.ConnectionPoint)
                {
                    return _shapes.IndexOf(shape) == 0 ? NodeType.Start : NodeType.End;
                }
            }
            return NodeType.Normal;
        }
        
        /// <summary>
        /// 标记被阻挡的节点
        /// </summary>
        private void MarkBlockedNodes()
        {
            foreach (var node in _gridNodes.Values)
            {
                // 检查节点是否在任何形状内部
                foreach (var shape in _shapes)
                {
                    if (shape.Bounds.Contains(node.Position) && 
                        node.Position != shape.ConnectionPoint)
                    {
                        node.IsBlocked = true;
                        break;
                    }
                }
            }
        }
        
        /// <summary>
        /// 使用A*算法寻找路径
        /// </summary>
        private List<Point> FindPathAStar(Point start, Point target)
        {
            // 重置所有节点
            foreach (var node in _gridNodes.Values)
            {
                node.Reset();
            }
            
            if (!_gridNodes.ContainsKey(start) || !_gridNodes.ContainsKey(target))
            {
                return null;
            }
            
            var startNode = _gridNodes[start];
            var targetNode = _gridNodes[target];
            
            var openSet = new List<GridNode> { startNode };
            var closedSet = new HashSet<GridNode>();
            
            startNode.GCost = 0;
            startNode.HCost = CalculateHeuristic(start, target);
            
            while (openSet.Count > 0)
            {
                // 选择F代价最小的节点
                var currentNode = openSet.OrderBy(n => n.FCost).ThenBy(n => n.HCost).First();
                openSet.Remove(currentNode);
                closedSet.Add(currentNode);
                
                if (currentNode == targetNode)
                {
                    return ReconstructPath(currentNode);
                }
                
                foreach (var neighbor in GetNeighbors(currentNode))
                {
                    if (neighbor.IsBlocked || closedSet.Contains(neighbor))
                        continue;
                    
                    var tentativeGCost = currentNode.GCost + CalculateDistance(currentNode, neighbor);
                    
                    // 添加转弯惩罚
                    if (currentNode.Parent != null)
                    {
                        var prevDirection = GetDirection(currentNode.Parent.Position, currentNode.Position);
                        var currDirection = GetDirection(currentNode.Position, neighbor.Position);
                        
                        if (prevDirection != currDirection)
                        {
                            tentativeGCost += TURN_PENALTY;
                        }
                    }
                    
                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                    else if (tentativeGCost >= neighbor.GCost)
                    {
                        continue;
                    }
                    
                    neighbor.Parent = currentNode;
                    neighbor.GCost = tentativeGCost;
                    neighbor.HCost = CalculateHeuristic(neighbor.Position, target);
                }
            }
            
            return null; // 没有找到路径
        }
        
        /// <summary>
        /// 获取邻居节点
        /// </summary>
        private List<GridNode> GetNeighbors(GridNode node)
        {
            var neighbors = new List<GridNode>();
            var directions = new[] 
            { 
                new Point(0, 1),   // 上
                new Point(0, -1),  // 下
                new Point(1, 0),   // 右
                new Point(-1, 0)   // 左
            };
            
            foreach (var dir in directions)
            {
                // 寻找该方向上最近的节点
                var current = new Point(node.Position.X + dir.X, node.Position.Y + dir.Y);
                GridNode nearestNode = null;
                double nearestDistance = double.MaxValue;
                
                foreach (var gridNode in _gridNodes.Values)
                {
                    if (IsInDirection(node.Position, gridNode.Position, dir))
                    {
                        var distance = CalculateDistance(node.Position, gridNode.Position);
                        if (distance < nearestDistance && !IsPathBlocked(node.Position, gridNode.Position))
                        {
                            nearestDistance = distance;
                            nearestNode = gridNode;
                        }
                    }
                }
                
                if (nearestNode != null)
                {
                    neighbors.Add(nearestNode);
                }
            }
            
            return neighbors;
        }
        
        /// <summary>
        /// 检查两点是否在指定方向上
        /// </summary>
        private bool IsInDirection(Point from, Point to, Point direction)
        {
            if (direction.X != 0) // 水平方向
            {
                return to.Y == from.Y && Math.Sign(to.X - from.X) == Math.Sign(direction.X);
            }
            else // 垂直方向
            {
                return to.X == from.X && Math.Sign(to.Y - from.Y) == Math.Sign(direction.Y);
            }
        }
        
        /// <summary>
        /// 检查路径是否被阻挡
        /// </summary>
        private bool IsPathBlocked(Point from, Point to)
        {
            foreach (var shape in _shapes)
            {
                if (IsLineIntersectingRectangle(from, to, shape.Bounds) &&
                    from != shape.ConnectionPoint && to != shape.ConnectionPoint)
                {
                    return true;
                }
            }
            return false;
        }
        
        /// <summary>
        /// 检查线段是否与矩形相交
        /// </summary>
        private bool IsLineIntersectingRectangle(Point p1, Point p2, Rectangle rect)
        {
            // 如果线段是水平的
            if (p1.Y == p2.Y)
            {
                int minX = Math.Min(p1.X, p2.X);
                int maxX = Math.Max(p1.X, p2.X);
                return p1.Y > rect.Top && p1.Y < rect.Bottom &&
                       minX < rect.Right && maxX > rect.Left;
            }
            // 如果线段是垂直的
            else if (p1.X == p2.X)
            {
                int minY = Math.Min(p1.Y, p2.Y);
                int maxY = Math.Max(p1.Y, p2.Y);
                return p1.X > rect.Left && p1.X < rect.Right &&
                       minY < rect.Bottom && maxY > rect.Top;
            }
            
            return false;
        }
        
        /// <summary>
        /// 获取方向
        /// </summary>
        private Point GetDirection(Point from, Point to)
        {
            var dx = to.X - from.X;
            var dy = to.Y - from.Y;
            
            if (Math.Abs(dx) > Math.Abs(dy))
            {
                return new Point(Math.Sign(dx), 0);
            }
            else
            {
                return new Point(0, Math.Sign(dy));
            }
        }
        
        /// <summary>
        /// 计算启发式距离（曼哈顿距离）
        /// </summary>
        private double CalculateHeuristic(Point a, Point b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
        }
        
        /// <summary>
        /// 计算两个节点之间的距离
        /// </summary>
        private double CalculateDistance(GridNode a, GridNode b)
        {
            return CalculateDistance(a.Position, b.Position);
        }
        
        /// <summary>
        /// 计算两个点之间的距离
        /// </summary>
        private double CalculateDistance(Point a, Point b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);
        }
        
        /// <summary>
        /// 重构路径
        /// </summary>
        private List<Point> ReconstructPath(GridNode targetNode)
        {
            var path = new List<Point>();
            var currentNode = targetNode;
            
            while (currentNode != null)
            {
                path.Add(currentNode.Position);
                currentNode = currentNode.Parent;
            }
            
            path.Reverse();
            return path;
        }
        
        /// <summary>
        /// 优化路径（移除不必要的转弯点）
        /// </summary>
        private List<Point> OptimizePath(List<Point> path)
        {
            if (path.Count <= 2)
                return path;
            
            var optimized = new List<Point> { path[0] };
            
            for (int i = 1; i < path.Count - 1; i++)
            {
                var prev = optimized[optimized.Count - 1];
                var current = path[i];
                var next = path[i + 1];
                
                // 检查是否可以直接从prev到next
                if (!CanConnectDirectly(prev, next))
                {
                    optimized.Add(current);
                }
            }
            
            optimized.Add(path[path.Count - 1]);
            return optimized;
        }
        
        /// <summary>
        /// 检查两点是否可以直接连接
        /// </summary>
        private bool CanConnectDirectly(Point from, Point to)
        {
            // 只有水平或垂直连接才被认为是直接连接
            if (from.X != to.X && from.Y != to.Y)
                return false;
            
            return !IsPathBlocked(from, to);
        }
        
        /// <summary>
        /// 应用中心线修正
        /// </summary>
        private List<Point> ApplyCenterLineCorrection(List<Point> path, Shape source, Shape target)
        {
            if (path.Count < 3)
                return path;
            
            // 计算中心线
            var centerX = (source.Bounds.Left + source.Bounds.Right + target.Bounds.Left + target.Bounds.Right) / 4;
            var centerY = (source.Bounds.Top + source.Bounds.Bottom + target.Bounds.Top + target.Bounds.Bottom) / 4;
            
            var corrected = new List<Point>();
            
            for (int i = 0; i < path.Count; i++)
            {
                var point = path[i];
                
                // 对于中间的水平线段，尝试移动到垂直中心线
                if (i > 0 && i < path.Count - 1)
                {
                    var prev = path[i - 1];
                    var next = path[i + 1];
                    
                    // 水平线段
                    if (prev.Y == point.Y && point.Y == next.Y)
                    {
                        var newPoint = new Point(point.X, centerY);
                        if (CanUsePoint(newPoint))
                        {
                            corrected.Add(newPoint);
                            continue;
                        }
                    }
                    // 垂直线段
                    else if (prev.X == point.X && point.X == next.X)
                    {
                        var newPoint = new Point(centerX, point.Y);
                        if (CanUsePoint(newPoint))
                        {
                            corrected.Add(newPoint);
                            continue;
                        }
                    }
                }
                
                corrected.Add(point);
            }
            
            return corrected;
        }
        
        /// <summary>
        /// 检查是否可以使用该点
        /// </summary>
        private bool CanUsePoint(Point point)
        {
            foreach (var shape in _shapes)
            {
                if (shape.Bounds.Contains(point))
                {
                    return false;
                }
            }
            return true;
        }
        
        /// <summary>
        /// 获取路径的总长度
        /// </summary>
        public double GetPathLength(List<Point> path)
        {
            if (path == null || path.Count < 2)
                return 0;
            
            double length = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                length += CalculateDistance(path[i], path[i + 1]);
            }
            return length;
        }
        
        /// <summary>
        /// 获取路径的转弯次数
        /// </summary>
        public int GetTurnCount(List<Point> path)
        {
            if (path == null || path.Count < 3)
                return 0;
            
            int turns = 0;
            for (int i = 1; i < path.Count - 1; i++)
            {
                var dir1 = GetDirection(path[i - 1], path[i]);
                var dir2 = GetDirection(path[i], path[i + 1]);
                
                if (dir1 != dir2)
                {
                    turns++;
                }
            }
            return turns;
        }
    }
    

}