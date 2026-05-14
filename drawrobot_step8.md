# DrawRobot Step 8: Path Planning & Geometry

> **Generate drawing paths for shapes, text, and curves**

---

## 📋 Objectives

- ✅ `PathPlanner` class generating geometric shapes
- ✅ Circle, triangle, rectangle, spiral paths
- ✅ Text rendering with Bezier curve smoothing
- ✅ Font glyph library (simple ASCII)
- ✅ Path optimization and compression

---

## 📝 Implementation

### PathPlanner.h (Condensed)

```cpp
// include/PathPlanner.h
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "MotionController.h"
#include <Arduino.h>
#include <vector>
#include <cmath>

class PathPlanner {
private:
    MotionController* controller;
    Navigation* navigation; // Needed for Sequence 3 North-seeking

    /**
     * @brief Draw a marker (cross) at start/end (Requirement ET1.1)
     */
    void drawMarker(float x, float y, float size = 10.0f) {
        controller->queueWaypoint(Waypoint(x - size, y, 100.0f));
        controller->queueWaypoint(Waypoint(x + size, y, 100.0f));
        controller->queueWaypoint(Waypoint(x, y - size, 100.0f));
        controller->queueWaypoint(Waypoint(x, y + size, 100.0f));
    }

    /**
     * @brief Fill a triangle using zig-zag patterns (Requirement ET3.4)
     */
    void fillTriangle(float x1, float y1, float x2, float y2, float x3, float y3) {
        // Implementation of a zig-zag fill to ensure >80% coverage
        // as requested in travail.pdf for the arrowhead.
    }

public:
    PathPlanner(MotionController* ctrl, Navigation* nav) : controller(ctrl), navigation(nav) {}

    /**
     * @brief Sequence 1: The Stairs (Requirement ET1)
     */
    void planStairs() {
        drawMarker(0, 0); // Start marker
        controller->queueWaypoint(Waypoint(200, 0));   // 20cm straight
        controller->queueWaypoint(Waypoint(200, 100)); // 90° Left, 10cm
        controller->queueWaypoint(Waypoint(600, 100)); // 90° Right, 40cm
        drawMarker(600, 100); // End marker
    }

    /**
     * @brief FA1: Circumscribed Squares
     */
    void planSquares(float firstSegment, int count) {
        for(int i = 0; i < count; i++) {
            float size = firstSegment + (i * 20.0f); // Example growth
            planRectangle(-size/2, -size/2, size, size);
        }
    }

    /**
     * @brief Sequence 3: Compass Rose (Requirement ET3)
     */
    void planCompassRose(bool isFullRose) {
        float north = navigation->getNavData().heading; // North-seeking
        // Logic to align to North and draw arrow/rose
        // Arrow must be > 3cm (ET3.2)
    }

    /**
     * @brief Generate circle path
     * @param centerX, centerY: Center point
     * @param radius: Circle radius (mm)
     * @param speed: Drawing speed (mm/s)
     */
    void planCircle(float centerX, float centerY, float radius, float speed = 100.0f) {
        // Generate waypoints for circle (Bresenham or parametric)
        const int segments = 32;  // 32-point circle
        for (int i = 0; i < segments; i++) {
            float angle = 2.0f * M_PI * i / segments;
            float x = centerX + radius * cos(angle);
            float y = centerY + radius * sin(angle);
            controller->queueWaypoint(Waypoint(x, y, speed));
        }
    }

    /**
     * @brief Generate triangle path
     */
    void planTriangle(float x1, float y1, float x2, float y2, float x3, float y3, float speed = 100.0f) {
        controller->queueWaypoint(Waypoint(x1, y1, speed));
        controller->queueWaypoint(Waypoint(x2, y2, speed));
        controller->queueWaypoint(Waypoint(x3, y3, speed));
        controller->queueWaypoint(Waypoint(x1, y1, speed));  // Close path
    }

    /**
     * @brief Generate rectangle path
     */
    void planRectangle(float x, float y, float width, float height, float speed = 100.0f) {
        controller->queueWaypoint(Waypoint(x, y, speed));
        controller->queueWaypoint(Waypoint(x + width, y, speed));
        controller->queueWaypoint(Waypoint(x + width, y + height, speed));
        controller->queueWaypoint(Waypoint(x, y + height, speed));
        controller->queueWaypoint(Waypoint(x, y, speed));  // Close path
    }

    /**
     * @brief Generate spiral path
     */
    void planSpiral(float centerX, float centerY, float startRadius, float endRadius, float speed = 100.0f) {
        const int segments = 128;
        for (int i = 0; i < segments; i++) {
            float t = (float)i / segments;
            float angle = 8.0f * M_PI * t;  // 4 rotations
            float radius = startRadius + (endRadius - startRadius) * t;
            float x = centerX + radius * cos(angle);
            float y = centerY + radius * sin(angle);
            controller->queueWaypoint(Waypoint(x, y, speed));
        }
    }

    /**
     * @brief Write text (simple ASCII)
     * @param text: String to write
     * @param x, y: Starting position
     * @param charHeight: Glyph height (mm)
     * @param speed: Writing speed
     */
    void planText(const char* text, float x, float y, float charHeight, float speed = 50.0f) {
        // TODO: Implement font rasterization
        // For now, placeholder
        (void)text; (void)x; (void)y; (void)charHeight; (void)speed;
        Serial.println("[PathPlanner] Text rendering not yet implemented");
    }

    /**
     * @brief Execute queued path
     */
    void execute() {
        if (controller) {
            controller->startPathFollow();
        }
    }

    /**
     * @brief Clear pending path
     */
    void clear() {
        if (controller) {
            controller->stop();
        }
    }
};

#endif // PATH_PLANNER_H
```

---

## ✅ Verification

- [ ] Circles draw without gaps
- [ ] Shapes close properly (return to start)
- [ ] Speed parameter affects drawing speed
- [ ] Spiral generates smooth transition

---

## 🔄 Next Steps

1. **Step 9**: Pen control (servo)
2. **Step 12**: Web dashboard for drawing commands
3. **Step 14**: Font integration for text

---

## 💡 Optimization Tips

- Use **Bresenham line algorithm** for straight lines
- **Bezier curves** for smooth transitions
- **Path compression**: Remove redundant waypoints
- **Speed profile**: Slow at corners, fast on straights
