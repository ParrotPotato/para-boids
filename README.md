# para-boids

![boids](boids.png)

A boids (flocking) simulation, mostly as an excuse to play with multi-threading and
SIMD in C.

The flocking itself is the usual separation/alignment/cohesion rules, nothing fancy.
The actual point is using it as a concrete, visual workload for trying out
threaded-by-default programs: the simulation is split into per-thread lanes from
the start, synced with a hand-rolled barrier, with SIMD (AVX2) layered on top for
the neighbor search once the threading was in place. Rendering goes through a single
GPU-instanced draw call via `rlgl` instead of one call per boid, and the whole thing
now runs in 3D with a simple orbit camera.

See [`LOG.md`](LOG.md) for a rundown of how it got here.

## Layout

- `main.c`: the simulation, update/render loop, flocking rules, spatial grid for
  neighbor lookups, and the thread/lane setup.
- `util.c`: the memory allocator, a big `mmap`'d reservation that hands out
  page-aligned arenas, committing pages lazily as things get allocated.

## Building

```bash
./build.sh
./main
```

Requires [raylib](https://www.raylib.com/) and pthreads. Press `Tab` while running
to toggle the on-screen perf stats.
