using System;
using System.Collections.Generic;
using System.Drawing;
using FlowchartOrthogonalRouting;

class SimpleTest
{
    static void Main()
    {
        Console.WriteLine("=== 流程图正交连线算法 C# 实现演示 ===\n");
        
        // 简单示例
        TestBasicRouting();
        
        Console.WriteLine("\n" + new string('=', 50) + "\n");
        
        // 复杂示例
        TestComplexRouting();
        
        Console.WriteLine("\n算法演示完成！");
    }
    
    static void TestBasicRouting()
    {
        Console.WriteLine("【基本示例】两个形状之间的简单连线");
        
        var sourceShape = new Shape(
            new Rectangle(50, 50, 100, 60), 
            ConnectionSide.Right, 
            0.5
        );
        
        var targetShape = new Shape(
            new Rectangle(300, 100, 120, 80), 
            ConnectionSide.Left, 
            0.4
        );
        
        Console.WriteLine($"源形状: ({sourceShape.Bounds.X}, {sourceShape.Bounds.Y}), " +
                         $"大小 {sourceShape.Bounds.Width}x{sourceShape.Bounds.Height}");
        Console.WriteLine($"目标形状: ({targetShape.Bounds.X}, {targetShape.Bounds.Y}), " +
                         $"大小 {targetShape.Bounds.Width}x{targetShape.Bounds.Height}");
        
        var generator = new OrthogonalLineGenerator(margin: 20);
        var route = generator.GenerateRoute(sourceShape, targetShape);
        
        Console.WriteLine("\n生成的路径:");
        for (int i = 0; i < route.Count; i++)
        {
            var point = route[i];
            var type = i == 0 ? "起点" : (i == route.Count - 1 ? "终点" : $"转点{i}");
            Console.WriteLine($"  {type}: ({point.X}, {point.Y})");
        }
        
        Console.WriteLine($"\n统计信息:");
        Console.WriteLine($"  路径长度: {generator.GetPathLength(route):F1} 像素");
        Console.WriteLine($"  转弯次数: {generator.GetTurnCount(route)} 次");
    }
    
    static void TestComplexRouting()
    {
        Console.WriteLine("【复杂示例】包含障碍物的连线");
        
        var sourceShape = new Shape(
            new Rectangle(50, 150, 80, 50), 
            ConnectionSide.Bottom, 
            0.7
        );
        
        var targetShape = new Shape(
            new Rectangle(400, 80, 90, 70), 
            ConnectionSide.Top, 
            0.3
        );
        
        var obstacles = new List<Shape>
        {
            new Shape(new Rectangle(150, 120, 100, 60), ConnectionSide.Top),
            new Shape(new Rectangle(280, 180, 80, 50), ConnectionSide.Left)
        };
        
        Console.WriteLine($"源形状: ({sourceShape.Bounds.X}, {sourceShape.Bounds.Y}), " +
                         $"连接点 ({sourceShape.ConnectionPoint.X}, {sourceShape.ConnectionPoint.Y})");
        Console.WriteLine($"目标形状: ({targetShape.Bounds.X}, {targetShape.Bounds.Y}), " +
                         $"连接点 ({targetShape.ConnectionPoint.X}, {targetShape.ConnectionPoint.Y})");
        Console.WriteLine($"障碍物数量: {obstacles.Count} 个");
        
        var generator = new OrthogonalLineGenerator(margin: 25);
        var route = generator.GenerateRoute(sourceShape, targetShape, obstacles);
        
        if (route.Count > 0)
        {
            Console.WriteLine("\n生成的路径:");
            for (int i = 0; i < route.Count; i++)
            {
                var point = route[i];
                var type = i == 0 ? "起点" : (i == route.Count - 1 ? "终点" : $"转点{i}");
                Console.WriteLine($"  {type}: ({point.X}, {point.Y})");
            }
            
            Console.WriteLine($"\n统计信息:");
            Console.WriteLine($"  路径长度: {generator.GetPathLength(route):F1} 像素");
            Console.WriteLine($"  转弯次数: {generator.GetTurnCount(route)} 次");
            
            // 分析路径特征
            AnalyzePath(route);
        }
        else
        {
            Console.WriteLine("\n未找到有效路径！");
        }
    }
    
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
        
        var minX = int.MaxValue;
        var maxX = int.MinValue;
        var minY = int.MaxValue;
        var maxY = int.MinValue;
        
        foreach (var point in path)
        {
            minX = Math.Min(minX, point.X);
            maxX = Math.Max(maxX, point.X);
            minY = Math.Min(minY, point.Y);
            maxY = Math.Max(maxY, point.Y);
        }
        
        Console.WriteLine($"  路径边界: ({minX}, {minY}) 到 ({maxX}, {maxY})");
        Console.WriteLine($"  占用区域: {maxX - minX} x {maxY - minY} 像素");
    }
}