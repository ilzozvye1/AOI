using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using FlowchartOrthogonalRouting;

namespace OrthogonalRoutingDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== 流程图正交连线算法演示 ===\n");
            
            // 运行基本示例
            RunBasicExample();
            
            Console.WriteLine("\n" + new string('=', 50) + "\n");
            
            // 运行复杂示例
            RunComplexExample();
            
            Console.WriteLine("\n按任意键退出...");
            Console.ReadKey();
        }
        
        /// <summary>
        /// 基本示例：两个简单形状之间的连线
        /// </summary>
        static void RunBasicExample()
        {
            Console.WriteLine("【基本示例】两个形状之间的简单连线");
            Console.WriteLine();
            
            // 创建源形状（矩形，右侧连接）
            var sourceShape = new Shape(
                new Rectangle(50, 50, 100, 60), 
                ConnectionSide.Right, 
                0.5  // 连接点在右侧中间
            );
            
            // 创建目标形状（矩形，左侧连接）
            var targetShape = new Shape(
                new Rectangle(300, 100, 120, 80), 
                ConnectionSide.Left, 
                0.4  // 连接点在左侧40%位置
            );
            
            Console.WriteLine($"源形状: 位置({sourceShape.Bounds.X}, {sourceShape.Bounds.Y}), " +
                            $"大小({sourceShape.Bounds.Width}x{sourceShape.Bounds.Height}), " +
                            $"连接点({sourceShape.ConnectionPoint.X}, {sourceShape.ConnectionPoint.Y})");
            
            Console.WriteLine($"目标形状: 位置({targetShape.Bounds.X}, {targetShape.Bounds.Y}), " +
                            $"大小({targetShape.Bounds.Width}x{targetShape.Bounds.Height}), " +
                            $"连接点({targetShape.ConnectionPoint.X}, {targetShape.ConnectionPoint.Y})");
            
            // 生成连线
            var generator = new OrthogonalLineGenerator(margin: 20);
            var route = generator.GenerateRoute(sourceShape, targetShape);
            
            // 输出结果
            Console.WriteLine("\n生成的路径点:");
            for (int i = 0; i < route.Count; i++)
            {
                var point = route[i];
                var prefix = i == 0 ? "起点" : (i == route.Count - 1 ? "终点" : $"转点{i}");
                Console.WriteLine($"  {prefix}: ({point.X}, {point.Y})");
            }
            
            Console.WriteLine($"\n路径统计:");
            Console.WriteLine($"  总长度: {generator.GetPathLength(route):F1} 像素");
            Console.WriteLine($"  转弯次数: {generator.GetTurnCount(route)} 次");
        }
        
        /// <summary>
        /// 复杂示例：包含多个障碍物的连线
        /// </summary>
        static void RunComplexExample()
        {
            Console.WriteLine("【复杂示例】带障碍物的复杂连线");
            Console.WriteLine();
            
            // 创建源形状
            var sourceShape = new Shape(
                new Rectangle(50, 150, 80, 50), 
                ConnectionSide.Bottom, 
                0.7
            );
            
            // 创建目标形状
            var targetShape = new Shape(
                new Rectangle(400, 80, 90, 70), 
                ConnectionSide.Top, 
                0.3
            );
            
            // 创建障碍物
            var obstacles = new List<Shape>
            {
                new Shape(new Rectangle(150, 120, 100, 60), ConnectionSide.Top),
                new Shape(new Rectangle(280, 180, 80, 50), ConnectionSide.Left),
                new Shape(new Rectangle(320, 30, 60, 40), ConnectionSide.Bottom),
                new Shape(new Rectangle(200, 250, 70, 45), ConnectionSide.Right)
            };
            
            Console.WriteLine($"源形状: 位置({sourceShape.Bounds.X}, {sourceShape.Bounds.Y}), " +
                            $"连接点({sourceShape.ConnectionPoint.X}, {sourceShape.ConnectionPoint.Y})");
            
            Console.WriteLine($"目标形状: 位置({targetShape.Bounds.X}, {targetShape.Bounds.Y}), " +
                            $"连接点({targetShape.ConnectionPoint.X}, {targetShape.ConnectionPoint.Y})");
            
            Console.WriteLine($"障碍物数量: {obstacles.Count} 个");
            for (int i = 0; i < obstacles.Count; i++)
            {
                var obs = obstacles[i];
                Console.WriteLine($"  障碍物{i + 1}: 位置({obs.Bounds.X}, {obs.Bounds.Y}), " +
                                $"大小({obs.Bounds.Width}x{obs.Bounds.Height})");
            }
            
            // 生成连线
            var generator = new OrthogonalLineGenerator(margin: 25);
            var route = generator.GenerateRoute(sourceShape, targetShape, obstacles);
            
            // 输出结果
            Console.WriteLine("\n生成的路径点:");
            if (route.Count > 0)
            {
                for (int i = 0; i < route.Count; i++)
                {
                    var point = route[i];
                    var prefix = i == 0 ? "起点" : (i == route.Count - 1 ? "终点" : $"转点{i}");
                    Console.WriteLine($"  {prefix}: ({point.X}, {point.Y})");
                }
                
                Console.WriteLine($"\n路径统计:");
                Console.WriteLine($"  总长度: {generator.GetPathLength(route):F1} 像素");
                Console.WriteLine($"  转弯次数: {generator.GetTurnCount(route)} 次");
                
                // 分析路径特征
                AnalyzePath(route);
            }
            else
            {
                Console.WriteLine("  未找到有效路径！");
            }
        }
        
        /// <summary>
        /// 分析路径特征
        /// </summary>
        static void AnalyzePath(List<Point> path)
        {
            if (path.Count < 2) return;
            
            Console.WriteLine("\n路径分析:");
            
            var horizontalSegments = 0;
            var verticalSegments = 0;
            var totalHorizontalLength = 0;
            var totalVerticalLength = 0;
            
            for (int i = 0; i < path.Count - 1; i++)
            {
                var current = path[i];
                var next = path[i + 1];
                
                if (current.Y == next.Y) // 水平线段
                {
                    horizontalSegments++;
                    totalHorizontalLength += Math.Abs(next.X - current.X);
                }
                else if (current.X == next.X) // 垂直线段
                {
                    verticalSegments++;
                    totalVerticalLength += Math.Abs(next.Y - current.Y);
                }
            }
            
            Console.WriteLine($"  水平线段: {horizontalSegments} 段, 总长度 {totalHorizontalLength} 像素");
            Console.WriteLine($"  垂直线段: {verticalSegments} 段, 总长度 {totalVerticalLength} 像素");
            
            // 计算路径的边界框
            var minX = path.Min(p => p.X);
            var maxX = path.Max(p => p.X);
            var minY = path.Min(p => p.Y);
            var maxY = path.Max(p => p.Y);
            
            Console.WriteLine($"  路径边界: ({minX}, {minY}) 到 ({maxX}, {maxY})");
            Console.WriteLine($"  占用区域: {maxX - minX} x {maxY - minY} 像素");
        }
    }
    
    /// <summary>
    /// 扩展方法，用于简化集合操作
    /// </summary>
    public static class Extensions
    {
        public static int Min(this IEnumerable<Point> source, Func<Point, int> selector)
        {
            return source.Select(selector).Min();
        }
        
        public static int Max(this IEnumerable<Point> source, Func<Point, int> selector)
        {
            return source.Select(selector).Max();
        }
    }
}