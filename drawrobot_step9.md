# DrawRobot Step 9: Pen Wheel Mechanics

> **Understand pen operation as a passive third wheel in contact with drawing surface**

---

## 📋 Objectives

- ✅ Understand pen wheel mechanical operation
- ✅ Verify pen maintains contact with surface
- ✅ Validate drawing consistency across rotation
- ✅ Monitor pen wear and alignment
- ✅ No active servo control (mechanical/gravitational)

---

## 🎯 Pen Wheel Concept

The pen is a **passive third wheel** mounted at the front of the robot:

```
        Pen Wheel (third wheel)
              |
         _____|_____
        |     o     |  <- Ink-bearing wheel
        |           |
        |  Robot    |  Pen Offset from motor axle: 130 mm
        |  (top)    |
        |___________|
              ^   ^
              |   |
          Left   Right
         Motor  Motor
         Wheels Wheels
```

**Key Characteristics**:
- ✅ **Passive**: Gravity or spring keeps it in contact
- ✅ **Always Drawing**: No lift/lower mechanism needed
- ✅ **Mechanical**: Worn contact must be monitored
- ✅ **Three-Point Stability**: Creates stable tripod base

---

## 📝 Physics & Geometry

### Pen Position Relative to Motor Axle

```
      Pen Tip
        |
        v (y_pen = 130 mm forward)
   _____|_____
  |     P     |  Pen wheel
  |  y_pen    |
  |  (130mm)  |
  |___________|
      Left Motor Axle
      |                |
      |   Wheel Base   |  (83 mm)
      |   (83 mm)      |
      |________________|
```

**Kinematics**: When robot moves forward, the pen wheel also moves forward.  
**Drawing**: The trajectory of the pen follows the path of the robot's motion.

---

## 🔍 Mechanical Considerations

### Pen Wheel Contact

1. **Ink Supply**: Pen wheel must have adequate ink/graphite contact
2. **Pressure**: Gravity provides consistent downward force
3. **Wear**: Monitor contact wear over time (re-felt or replace)
4. **Slip**: Wheel rotation + forward motion = drawing without slipping

### Surface Alignment

- Robot must remain horizontal (level)
- Paper/surface must be smooth and flat
- Pen wheel height matches paper thickness

---

## 🧪 Validation Procedure

### Test 1: Visual Inspection
```
1. Place robot on blank paper
2. Move robot forward 100mm (straight line)
3. Inspect: Should draw a straight line
4. Check: Line width uniform, no gaps
```

### Test 2: Rotation Test
```
1. Place pen wheel at paper center
2. Rotate robot 360° in place
3. Inspect: Should draw a complete circle
4. Check: Circle radius = pen offset (130 mm approximately)
```

### Test 3: Complex Path
```
1. Execute predefined path (square, triangle)
2. Inspect: Path accuracy ±10-15mm acceptable
3. Check: No pen slipping or chattering
```

---

## 📊 Pen Wheel Wear Monitoring

Monitor contact quality:

| Indicator | Status | Action |
|-----------|--------|--------|
| Clear, dark lines | ✅ Good | Continue operation |
| Faint or uneven | ⚠️ Worn | Replace felt/graphite soon |
| No contact | ❌ Failed | Replace pen wheel immediately |

**Maintenance**: Inspect after every 10m of drawing, replace felt every 50m

---

## 🎨 Drawing Strategy

### Path Execution

The robot draws by simply **moving along the planned path**:

```cpp
// Drawing happens automatically as robot moves
// No pen control needed
void drawCircle(float radius) {
    for (int i = 0; i < 32; i++) {
        float angle = 2.0f * M_PI * i / 32;
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        
        // Move to point - pen draws automatically
        motionController->queueWaypoint(Waypoint(x, y));
    }
    motionController->startPathFollow();
}
```

**No lift/lower calls needed** - pen is always in contact.

---

## ✅ Verification Checklist

- [ ] Pen wheel visible and aligned properly
- [ ] Pen wheel rotates freely
- [ ] Ink/graphite visible on contact surface
- [ ] Straight line test produces clear 100mm line
- [ ] Rotation test produces circle ~130mm radius
- [ ] Complex path draws with good accuracy
- [ ] No pen skipping or dragging

---

## 🔄 Next Steps

1. **Step 8**: Path planning (geometry generation)
2. **Step 10**: Configuration manager (motor tuning)
3. **Step 14**: Full system integration

---

## 💡 Design Notes

- **Passive Operation**: Simplifies firmware (no servo control)
- **Mechanical Reliability**: Fewer moving parts = more robust
- **Wear Patterns**: Consumable part, like printer ink
- **Alignment Critical**: Must remain level for consistent contact

---

## 🔗 References

- Three-point robot geometry (tripod stability)
- Pen wheel diameter: Typically 5-10mm (check hardware)
- Ink medium: Water-based markers, graphite wheels, or felt pens
