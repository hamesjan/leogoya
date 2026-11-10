# Painting algorithms: what to paint, and how to move the arm to paint it

The problem splits into three layers, and each needs its own algorithm:

| Layer | Question | Implemented now | Upgrade path |
|---|---|---|---|
| **Perception** | What does the reference look like, and what does the canvas look like right now? | Marker homography + Lab color (`sim/vision/vision.c`) | Lens-distortion correction, per-pixel lighting normalization |
| **Planning** | Which strokes (path, width, color, order)? | Hertzmann layered strokes + greedy simulator acceptance (`sim/planning/planner.c`) | Stroke optimization, learned stroke proposer (TinyML) |
| **Actuation** | How must X/Y/Z/yaw move so the stroke on canvas matches the plan? | Spline + tangent yaw + pressure profile + paint-load model (`sim/motion/motion.c`, `sim/planning/brush_model.c`) | Learned inverse brush model, height-map compensation, speed-as-a-parameter |

Feedback ties them together: **plan → paint → photograph → re-plan**. That loop matters most. Paint never does exactly what a simulator predicts (it flows, dries lighter or darker, goes translucent at the end of a load), so a cheap camera that keeps correcting will beat a clever open-loop planner.

---

## 1. What's already running in `sim/`

### 1.1 Palette with k-means (unsupervised ML)
`sim/vision/palette.c`: k-means++ in CIELAB on 8k sampled pixels gives `PALETTE_N` paint colors, sorted dark to light. Because the robot can't mix paint, *you* mix the pots, guided by the printed hex values and mixing hints.
- Lab is used because Euclidean distance in Lab is roughly perceptual difference (ΔE).
- **Do next:** after you mix the pots, paint a swatch of each and measure their real colors from a photo (not implemented yet: sample the center of each swatch in the rectified image). The simulator then uses the colors you actually mixed rather than the ones it asked for, which closes the biggest sim-to-real gap for almost no work.

### 1.2 Stroke planning: Hertzmann (1998), plus a greedy optimizer
`sim/planning/planner.c` implements *Painterly Rendering with Curved Brush Strokes of Multiple Sizes* (Hertzmann, SIGGRAPH 1998) with one important change:

1. **Coarse to fine layers.** Big brush first (24 mm), then 12 mm, then 5 mm. Each layer compares against the reference blurred to that brush's scale, so big brushes don't chase detail they can't render.
2. **Seed** a stroke in every grid cell whose error is above the threshold, at the cell's worst pixel.
3. **Grow** the stroke along the direction perpendicular to the luminance gradient, which means *along* edges, the way a painter follows a contour. A curvature filter keeps it smooth, and growth stops when continuing would make the canvas worse.
4. **The change: accept only if it helps.** The stroke is rasterized into a simulated canvas, and it's kept only if mean ΔE improves by more than 1 per covered pixel. Plain Hertzmann paints every seeded stroke; this version rejects 30–60% of them, which saves paint, time and mud.
5. **Order** by color (fewest rinses), then nearest neighbor (least travel).

With the host simulator on the test scene, mean ΔE goes 54.7 → 8.1 → 4.7 → 4.3 across the three layers, from about 2.3k strokes and 150 m of brush path.

### 1.3 Brush model with a learned regression
`sim/planning/brush_model.c`: `width = w0 + k·depth` is fit by least squares from a calibration strip. Run the simulator with `--calibrate`, and the robot paints lines at 6 press depths per brush, photographs them, measures the widths, and fits the model. The paint-load model `opacity = 1 − (used/capacity)^dry` decides when to re-dip, and the planner and the machine share that decision (`stroke_t.reload`), so the simulator and reality agree on when the brush is running dry.

### 1.4 Closed loop
With `CLOSED_LOOP=1`, before every layer the gantry parks, the camera photographs the canvas, and that photo *replaces* the simulated canvas. The next layer is planned against what is really on the canvas.

---

## 2. Planning upgrades (ranked by value ÷ effort)

### A. Close the loop more often (easy, high value)
Photograph every ~100 strokes instead of every layer. Re-plan only the grid cells where photo and simulation disagree by more than ΔE 10 (that's the paint misbehaving). Cost: about 3 s per park-and-capture.

### B. Stroke optimization by hill climbing (easy, cheap)
After a stroke is grown, perturb its control points, width and color 20–50 times and keep the best score. This is the "Geometrize / primitive" approach. It runs in microseconds per try on the S3 because `score()` only touches the stroke's bounding box. Expect roughly 15–25% fewer strokes for the same error.

### C. Region fill for big flat areas (easy, big time saver on a 38" canvas)
For low-gradient regions of one palette color (sky, backgrounds), skip the many short strokes and fill with long boustrophedon (zig-zag) strokes along a chosen angle, like hatching. It's far fewer dips and lifts, and it looks deliberate.

### D. Data-driven brush footprint (medium)
Replace the round "stamp" in the simulator with the *real* stroke appearance. From calibration photos, cut out real strokes (start blob, body, dry tail) and build a small library indexed by (brush, depth, load). Rendering then pastes these textures. This is what **FRIDA** (Schaldenbrand et al., CMU, ICRA 2023) does to get a robot arm painting well: learn the stroke simulator from photos of the robot's own strokes.

### E. Learned stroke proposer (TinyML; the real "ML" upgrade)
The simulator runs on the PC, so a learned model can run there too:
- **Input:** a 32×32 patch of (reference − current canvas) in Lab, plus brush size.
- **Output:** stroke parameters (start, 2 control points, end, width, palette index), or a direction field.
- **Training:** use a *differentiable* stroke renderer (a tiny neural renderer as in **Learning to Paint**, Huang et al. ICCV 2019, or analytical as in **Stylized Neural Painting**, Zou et al. CVPR 2021, and **Paint Transformer**, Liu et al. ICCV 2021). Train with the same objective the greedy planner uses, then distill to a ~50–200 k-parameter CNN.
- **Deploy:** call it from the simulator (ONNX Runtime or a hand-written forward pass in C). Keep the greedy acceptance test on top: the network *proposes*, the simulator *verifies*. A bad prediction then costs nothing.
- Paint Transformer itself (~11 M parameters) also runs fine on a laptop; distill it only if you later move planning onto an embedded board.

### F. Style controls (fun)
Stroke length/curvature limits, jitter of color within the palette neighborhood, and a "minimum stroke length" give looser or tighter styles. Painting the edge-direction field from a *structure tensor* instead of the raw gradient gives smoother, more painterly flow.

---

## 3. Actuation: how to drive the axes so a stroke looks right

### 3.1 Kinematics
The machine is a Cartesian gantry with a 2-DOF wrist, so inverse kinematics is the identity: `machine = canvas_origin + px · mm_per_px`. That's the main reason to choose a gantry over an arm at this size (see `MECHANICAL.md`). If you build a SCARA anyway, the closed-form IK for link lengths L1, L2 and target (x, y) is:
```
c2 = (x² + y² − L1² − L2²) / (2·L1·L2)            # |c2| > 1 → unreachable
θ2 = ±acos(c2)                                    # pick one elbow and keep it
θ1 = atan2(y, x) − atan2(L2·sin θ2, L1 + L2·cos θ2)
yaw_motor = stroke_tangent − θ1 − θ2              # brush angle is relative to link 2
```
Then resample the path in *joint* space finely (≤1 mm Cartesian error between segments), because straight lines in joint space are curves on the canvas.

### 3.2 Path: spline + tangent-aligned yaw (implemented)
- Catmull-Rom through the planner's control points, resampled every 2 mm, streamed as `G1` segments. Marlin's junction-deviation lookahead keeps the speed smooth.
- **Yaw** follows the path tangent so a flat or filbert brush is always dragged "broadside". The brush is symmetric under 180°, so pick the equivalent angle nearest the current yaw. Rotation is minimized and the yaw is re-centered between strokes.

### 3.3 Pressure profile: the part that makes it look painted
A real stroke is not a constant-depth drag. `depth_at()` implements:
```
 depth
   ▲        ┌───────────────────────┐
   │       /                         \
   │      /  landing    body          \  lift-off flick
   │_____/                             \______
   └──── approach (hover) ──────────────────────► arc length
```
- Touch down **while already moving** (approach from 3 mm behind). A vertical plunge leaves a blob.
- Ramp to full depth over 4 mm and taper out over 6 mm while continuing along the tangent, which gives a natural tapered tail.
- Keep at least 35% depth mid-stroke so short strokes don't skip.

### 3.4 Speed is a painting parameter (upgrade)
Paint deposition per mm falls as speed rises. Slow gives opaque, wet, wide strokes; fast gives "dry brush" texture. Add `feed` to `stroke_t`, and let the planner pick slow for the first stroke after a dip (heavy paint) and fast for scumbling highlights. Measure it in calibration as a second regression input: `width = f(depth, speed)`.

### 3.5 Learned inverse brush model (upgrade)
Generalize 1.3 into a small regression or MLP: **(target width, target opacity, curvature, paint used) → (depth, speed)**. Train it from a few hundred calibration strokes photographed by the robot. An MLP with 4→16→16→2 weights is trivially fast and can be trained with plain SGD in the planner from the calibration photos.

### 3.6 Canvas sag / height map (do this before your first real painting)
A 38" canvas sags several mm in the middle and is never flat. Put a microswitch on the spring-compliant brush slide so it trips when the brush touches. Use it as Marlin's Z probe and run `G29` bilinear leveling on a 5×5 grid. Marlin then compensates Z everywhere, and press depth means the same thing in the corners as in the center.

### 3.7 Why not reinforcement learning on the real robot?
Each episode costs paint, canvas and minutes, and the reward needs a camera photo. Learn in the simulator (calibrated by 1.3, 2.D and 3.5), then use the camera loop to absorb the remaining sim-to-real error. That's the FRIDA / e-David recipe. **e-David** (Deussen et al., Univ. Konstanz) is the classic visual-feedback painting robot and is worth reading before you over-engineer.

---

## 4. Suggested order of work
1. Run the host sim with your own references (`make run REF=...`) and tune `LAYER_*` in `config.h` until the simulated painting looks right.
2. Build the gantry, then calibrate `CANVAS_X0/Y0`, pots and `Z_TOUCH` by jogging.
3. Run brush calibration (button held at boot). Check that R² > 0.9.
4. First painting open-loop (`CLOSED_LOOP 0`) on a cheap 16×20 board with scaled `CANVAS_MM`.
5. Turn on closed loop, then add the swatch measurement (1.1).
6. Height map (3.6), then region fill (2.C), then hill climbing (2.B).
7. Only then: the learned proposer (2.E) and learned brush model (3.5).

## References
- A. Hertzmann, *Painterly Rendering with Curved Brush Strokes of Multiple Sizes*, SIGGRAPH 1998.
- P. Schaldenbrand, J. McCann, J. Oh, *FRIDA: A Collaborative Robot Painter with a Differentiable, Real2Sim2Real Planning Environment*, ICRA 2023.
- Z. Huang, W. Heng, S. Zhou, *Learning to Paint with Model-based Deep Reinforcement Learning*, ICCV 2019.
- Z. Zou et al., *Stylized Neural Painting*, CVPR 2021.
- S. Liu et al., *Paint Transformer: Feed Forward Neural Painting with Stroke Prediction*, ICCV 2021.
- O. Deussen, T. Lindemeier et al., *e-David* painting robot papers, Univ. Konstanz (2012–2016).
