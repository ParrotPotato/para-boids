# Log

Short notes on the actual code changes, skipping the readme/screenshot commits.

- Got a basic threaded boid sim running: separation/alignment/cohesion rules, the
  simulation split into per-thread lanes from the start, synced with a hand-rolled
  sense-reversing barrier, each thread pulling memory from its own arena carved out
  of a single reserved mmap block.
- Reworked the data layout for SIMD: boids moved into flat SoA arrays, and the
  spatial grid switched from a pointer-chased linked list to a sorted CSR-style
  layout, so a cell's boids sit next to each other in memory instead of scattered.
- Vectorized the neighbor search with AVX2 (8 boids at a time, merging the three
  row-cells into one range before scanning). Dropped per-thread update time from
  ~25ms to ~5ms.
- Replaced per-boid draw calls with a single GPU-instanced draw through rlgl.
  Render time went from ~11ms to under 1ms, and boids got upgraded from dots to
  oriented triangles/tetrahedrons.
- Migrated the whole thing from 2D to 3D: grid, physics, camera, mesh, the lot.
  Also chased down a bug where boids slowly drifted into each other (separation
  radius/weight were still tuned for 2D density) and added boundary avoidance so
  they don't fly off into open space.
