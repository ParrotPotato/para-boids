#include <bits/time.h>
#include <immintrin.h>
#include <math.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>

#include <time.h>
#include <unistd.h>

// WTH C library: why does this not exist ?? 
void timespec_delta(const struct timespec * start, const struct timespec * end, struct timespec * out_delta) {
    out_delta->tv_nsec = end->tv_nsec - start->tv_nsec;
    out_delta->tv_sec = end->tv_sec - start->tv_sec;

    if (out_delta->tv_nsec < 0){
        out_delta->tv_nsec += 1000000000L;
        out_delta->tv_sec  -= 1;
    }
}

#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <string.h>

#include "util.c"


typedef struct {
    atomic_int count;
    atomic_int sense;
    int total;
}Barrier;

void barrier_init(Barrier * barrier, int thread_count) {
    assert(barrier);
    assert(thread_count);

    atomic_init(&barrier->count, 0);
    atomic_init(&barrier->sense, 0);
    barrier->total = thread_count;
}

__thread int thread_sense = 0;

void barrier_lane(Barrier * barrier) {
    
    int expected_sense = !thread_sense;
    
    int new_count = atomic_fetch_add_explicit(&barrier->count, 1, memory_order_acq_rel) + 1;
    if (new_count == barrier->total){
        atomic_store_explicit(&barrier->count, 0, memory_order_relaxed);
        atomic_store_explicit(&barrier->sense, expected_sense, memory_order_release);
    } else {
        while(atomic_load_explicit(&barrier->sense, memory_order_acquire) != expected_sense);
    }

    thread_sense = expected_sense;
}

typedef struct {
    double ms_update_duration;
    double ms_avg_neignbour_calcualtion;
} ThreadPerformance ;

typedef struct {
    Barrier barrier;

    size_t thread_count;
    ThreadPerformance * thread_performance;

    void * app;
}LaneGroup ;

typedef struct {
    LaneGroup * group;
    int lane_id;
    Arena * arena;
}LaneCtx;

typedef struct {
    LaneCtx ctx;
}_Thread_Data;

__thread LaneCtx ctx;

#define THREAD_ARENA_PTR() (ctx.arena)
#define LANE_COUNT() (ctx.group->thread_count)
#define LANE_ID() (ctx.lane_id)
#define LANE_PERFORMANCE() (&(ctx.group->thread_performance[LANE_ID()]))
#define LANE_BARRIER() (barrier_lane(&ctx.group->barrier))

void lane_range(size_t start, size_t end, size_t * out_thread_start, size_t * out_thread_end){

    size_t total_count = end - start;
    size_t per_thread =  total_count / LANE_COUNT();
    size_t remaining_count = total_count % LANE_COUNT();

    size_t this_thread_count = per_thread;
    if (remaining_count > LANE_ID()){
        this_thread_count = per_thread + 1;
    } 

    size_t thread_start = 0;
    size_t thread_end = 0;
    if (remaining_count > LANE_ID()){
        thread_start = (per_thread + 1) * LANE_ID();
        thread_end = thread_start + this_thread_count;
    } else {
        thread_start = (per_thread + 1) * remaining_count + (LANE_ID() - remaining_count) * (per_thread);
        thread_end = thread_start + this_thread_count;
    }

    * out_thread_start = thread_start;
    * out_thread_end   = thread_end;
}

typedef struct {
    Vector2 pos;
    Vector2 vel;
} Boid ; 

#define WORLD_WIDTH  1600
#define WORLD_HEIGHT 900
#define WORLD_DEPTH  1200

#define NEIGHBOUR_RADIUS  30.0f
#define SEPARATION_RADIUS 20.0f
#define SEPARATION_WEIGHT 16.0f
#define ALIGNMENT_WEIGHT  5.0f
#define COHESION_WEIGHT   2.0f
#define MAX_SPEED         400.0f
#define MIN_SPEED         200.0f

#define CELL_SIZE NEIGHBOUR_RADIUS
#define GRID_W ((int) (WORLD_WIDTH  / CELL_SIZE) + 1)
#define GRID_H ((int) (WORLD_HEIGHT / CELL_SIZE) + 1)
#define GRID_Z ((int) (WORLD_DEPTH  / CELL_SIZE) + 1) 
#define GRID_CELLS (GRID_W * GRID_H * GRID_Z)

#define MAX_BOID_COUNT  100000

typedef struct {
    int _offset[GRID_CELLS + 1];
    int indices[MAX_BOID_COUNT];
} GridSOA;

int offset(GridSOA * grid_soa, int x) {
        if (x < 0) return grid_soa->_offset[0];
        if (x > GRID_CELLS) return grid_soa->_offset[GRID_CELLS];
        return grid_soa->_offset[x];
}

typedef struct {
    float x[MAX_BOID_COUNT];
    float y[MAX_BOID_COUNT];
    float z[MAX_BOID_COUNT];
    float vx[MAX_BOID_COUNT];
    float vy[MAX_BOID_COUNT];
    float vz[MAX_BOID_COUNT];
} BoidSOA;

typedef struct {
    // sim information

    BoidSOA boid_soa[2];
    int front;
    size_t boid_count;
    GridSOA grid_soa;
    float delta_time;
    
    // render information 
    unsigned int vao, tribuffer, instancebuffer;
    unsigned int shader_id;
    int mvp_loc;

    float cam_yaw, cam_pitch, cam_distance;


    Matrix mvp, proj;
}  App;

#define APP()              ((App *)ctx.group->app)
#define BOID_SOA()         ((APP())->boid_soa[(APP())->front])
#define NEXT_BOID_SOA()    ((APP())->boid_soa[((APP())->front + 1) % 2])
#define FLIP()             (APP())->front = (((APP())->front + 1) % 2)

void init_window() {
    InitWindow(1600, 900, "para boid");
    SetRandomSeed(100);

    for (int i = 0 ; i < MAX_BOID_COUNT; i++){

        float x =GetRandomValue(0, WORLD_WIDTH * 1000)  / 1000.0f;
        float y = GetRandomValue(0, WORLD_HEIGHT * 1000) / 1000.0f;
        float z = GetRandomValue(0, WORLD_DEPTH * 1000) / 1000.0f;

        float vx = GetRandomValue(-1000, 1000) / 1000.0f;
        float vy = GetRandomValue(-1000, 1000) / 1000.0f;
        float vz = GetRandomValue(-1000, 1000) / 1000.0f;
        
        Vector3 vel = Vector3Normalize((Vector3){vx, vy, vz});

        BOID_SOA().x[i] = x;
        BOID_SOA().y[i] = y;
        BOID_SOA().z[i] = z;
        BOID_SOA().vx[i] = vel.x;
        BOID_SOA().vy[i] = vel.y;
        BOID_SOA().vz[i] = vel.z;
    }
    APP()->boid_count = MAX_BOID_COUNT;
}

void init_renderer() {
    unsigned int vao = rlLoadVertexArray();
    rlEnableVertexArray(vao);

    float tetraheadron[] = {
        0.0f,  0.0f,  4.0f,   // v0
        -2.4f, -2.0f, -2.0f,  // v1
        2.4f, -2.0f, -2.0f,   // v2
        0.0f,  2.0f, -2.0f,   // v3
    };

    unsigned short indices[] = {
        0,1,2, 0,3,1, 0,2,3, 1,3,2
    };

    unsigned int tribuffer = rlLoadVertexBuffer(tetraheadron, sizeof(tetraheadron), false);
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, 0, 0);
    rlEnableVertexAttribute(0);

    unsigned int indexbuffer = rlLoadVertexBufferElement(indices, sizeof(indices), false); 
    rlEnableVertexBufferElement(indexbuffer);

    unsigned int instancebuffer = rlLoadVertexBuffer(NULL, MAX_BOID_COUNT * 6 * sizeof(float), true);

    rlSetVertexAttribute(1, 3, RL_FLOAT, false, 6 * sizeof(float), 0);
    rlEnableVertexAttribute(1);
    rlSetVertexAttributeDivisor(1, 1);

    rlSetVertexAttribute(2, 3, RL_FLOAT, false, 6 * sizeof(float), 3 * sizeof(float));
    rlEnableVertexAttribute(2);
    rlSetVertexAttributeDivisor(2, 1);

    rlEnableDepthTest();

    const char * vertex_src = "#version 400\n"
        "layout(location = 0) in vec3 tri;\n"
        "layout(location = 1) in vec3 pos;\n"
        "layout(location = 2) in vec3 vel;\n"
        "uniform mat4 mvp;\n"
        "void main(){\n"
        "vec3 forward = normalize(vel);\n"
        "vec3 world_up = vec3(0.0, 1.0, 0.0);\n"
        "vec3 right = normalize(cross(world_up, forward));\n"
        "vec3 up = cross(forward, right);\n"
        "vec3 rotated = tri.x *  right + tri.y * up + tri.z * forward;\n"
        "gl_Position = mvp * vec4(pos + rotated, 1.0);\n"
        "}";

    const char * fragment_src = "#version 400\n"
        "out vec4 outcolor;\n"
        "void main() {\n"
        "outcolor = vec4(1.0, 1.0, 1.0, 0.2);\n"
        "}\n";

    unsigned int shader_id = rlLoadShaderCode(vertex_src, fragment_src);
    int mvp_loc = rlGetLocationUniform(shader_id, "mvp");


    Vector3 camera_pos    = {WORLD_WIDTH * 0.5f, WORLD_HEIGHT * 0.1f, -WORLD_DEPTH * 2.0f};
    Vector3 camera_target = {WORLD_WIDTH * 0.5f, WORLD_HEIGHT * 0.5f,  WORLD_DEPTH * 0.5f};
    Vector3 camera_up     = {0.0f, 1.0f, 0.0f};

    Matrix view = MatrixLookAt(camera_pos, camera_target, camera_up);
    Matrix proj = MatrixPerspective(
            DEG2RAD * 60.0f, 
            (float) WORLD_WIDTH / (float) WORLD_HEIGHT, 10.0f, WORLD_DEPTH * 4.0f);

    Matrix mvp = MatrixMultiply(view, proj);

    APP()->vao = vao;
    APP()->tribuffer = tribuffer;
    APP()->instancebuffer = instancebuffer ;
    APP()->shader_id = shader_id;
    APP()->mvp_loc = mvp_loc;
    APP()->mvp = mvp;
    APP()->proj = proj;
    APP()->cam_yaw = 0.0f;
    APP()->cam_pitch = 0.3f;
    APP()->cam_distance = WORLD_DEPTH * 2.0f;
}

void render_update_camera() {

    Vector3 target = {WORLD_WIDTH * 0.5, WORLD_HEIGHT * 0.5, WORLD_DEPTH * 0.5};
    
    if (IsKeyDown(KEY_LEFT)) APP()->cam_yaw -= 2.0f * GetFrameTime();
    if (IsKeyDown(KEY_RIGHT)) APP()->cam_yaw += 2.0f * GetFrameTime();
    if (IsKeyDown(KEY_UP)) APP()->cam_pitch += 2.0f * GetFrameTime();
    if (IsKeyDown(KEY_DOWN)) APP()->cam_pitch -= 2.0f * GetFrameTime();

    APP()->cam_pitch = Clamp(APP()->cam_pitch, -1.5f, 1.5f);
    APP()->cam_distance -= GetMouseWheelMove() * 100.0f;
    APP()->cam_distance = Clamp(APP()->cam_distance, WORLD_DEPTH * 0.5f, WORLD_DEPTH * 6.0f);

    Vector3 cam_pos = {
        target.x  + APP()->cam_distance * cosf(APP()->cam_pitch) * sinf(APP()->cam_yaw),
        target.y  + APP()->cam_distance * sinf(APP()->cam_pitch),
        target.z  + APP()->cam_distance * cosf(APP()->cam_pitch) * cosf(APP()->cam_yaw),
    };

    Matrix view = MatrixLookAt(cam_pos, target, (Vector3){0.0, 1.0f, 0.0});
    APP()->mvp = MatrixMultiply(view, APP()->proj);
}

void render() {
    static float instance_data[MAX_BOID_COUNT * 6] = {};
    for (int i = 0 ; i < APP()->boid_count; i++){
        instance_data[i * 6 + 0] = BOID_SOA().x[i];
        instance_data[i * 6 + 1] = BOID_SOA().y[i];
        instance_data[i * 6 + 2] = BOID_SOA().z[i];
        instance_data[i * 6 + 3] = BOID_SOA().vx[i];
        instance_data[i * 6 + 4] = BOID_SOA().vy[i];
        instance_data[i * 6 + 5] = BOID_SOA().vz[i];
    }
    rlUpdateVertexBuffer(APP()->instancebuffer, instance_data, APP()->boid_count * 6 * sizeof(float), 0);

    rlDrawRenderBatchActive();
    rlEnableShader(APP()->shader_id);
    rlSetUniformMatrix(APP()->mvp_loc, APP()->mvp);
    rlEnableVertexArray(APP()->vao);
    rlDrawVertexArrayElementsInstanced(0, 12, 0, APP()->boid_count);
}

void build_grid_soa(GridSOA * grid_soa) {
    long long insert[GRID_CELLS] = {};

    for(size_t i = 0 ; i < GRID_CELLS; i++) grid_soa->_offset[i] = 0;

    for(size_t i = 0 ; i < MAX_BOID_COUNT; i++) {
        int x = (int) floorf(BOID_SOA().x[i] / CELL_SIZE);
        int y = (int) floorf(BOID_SOA().y[i] / CELL_SIZE);
        int z = (int) floorf(BOID_SOA().z[i] / CELL_SIZE);
        long long cell = y * GRID_W * GRID_Z + z * GRID_W + x;
        grid_soa->_offset[cell] += 1;
        insert[cell] += 1;
    }

    long long cummulative = 0;
    for(size_t i = 0 ; i < GRID_CELLS ; i++){
        int count = grid_soa->_offset[i];
        grid_soa->_offset[i] = cummulative;
        insert[i] = cummulative;
        cummulative += count;
    }
    grid_soa->_offset[GRID_CELLS] = cummulative;

    for(size_t i = 0 ; i < MAX_BOID_COUNT; i++){
        int x = (int) floorf(BOID_SOA().x[i] / CELL_SIZE);
        int y = (int) floorf(BOID_SOA().y[i] / CELL_SIZE);
        int z = (int) floorf(BOID_SOA().z[i] / CELL_SIZE);
        long long cell = y * GRID_W * GRID_Z +  z * GRID_W + x;

        long long offset = insert[cell];
        insert[cell] += 1;
        grid_soa->indices[offset] = i;

        NEXT_BOID_SOA().x[offset] = BOID_SOA().x[i];
        NEXT_BOID_SOA().y[offset] = BOID_SOA().y[i];
        NEXT_BOID_SOA().z[offset] = BOID_SOA().z[i];

        NEXT_BOID_SOA().vx[offset] = BOID_SOA().vx[i];
        NEXT_BOID_SOA().vy[offset] = BOID_SOA().vy[i];
        NEXT_BOID_SOA().vz[offset] = BOID_SOA().vz[i];
    }

    FLIP();
}

typedef struct {
    Vector3 separation, align_sum, cohesion_sum;
    int neighbours;
}NeighbourAcc;

NeighbourAcc acculumate_neighbours(Vector3 self_pos, int self_idx, int start, int end, float * x, float * y, float * z, float * vx, float * vy, float * vz) {
    NeighbourAcc result = {};
    for (int j = start; j < end; j++){
        if (j == self_idx) continue;

        Vector3 other_pos = {x[j], y[j], z[j]};
        Vector3 other_vel = {vx[j], vy[j], vz[j]};

        float dist_sq =  Vector3DistanceSqr(self_pos, other_pos);
        if (dist_sq >= NEIGHBOUR_RADIUS * NEIGHBOUR_RADIUS) continue;

        if (dist_sq < SEPARATION_RADIUS * SEPARATION_RADIUS && dist_sq > 0.000001f * 0.000001f) {
            float dist = sqrtf(dist_sq);
            Vector3 away = Vector3Scale(Vector3Subtract(self_pos, other_pos), 1.0f/dist);
            result.separation = Vector3Add(result.separation, away);
        }

        result.align_sum = Vector3Add(result.align_sum, other_vel);
        result.cohesion_sum = Vector3Add(result.cohesion_sum, other_pos);
        result.neighbours++;
    }
    return result;
}

static inline float hsum256_ps(__m256 v) {
    // v = [0, 1, 2, 3, 4, 5, 6, 7]
    __m128 lo = _mm256_castps256_ps128(v); // [0, 1, 2, 3]
    __m128 hi = _mm256_extractf128_ps(v, 1); // [4, 5, 6, 7]

    lo = _mm_add_ps(lo, hi); // [0 + 4, 1 + 5, 2 + 6, 3 + 7]

    lo = _mm_hadd_ps(lo, lo); // [0 + 4 + 1 + 5, 2 + 6 + 3 + 7, 0 + 4 + 1 + 5, 2 + 6 + 3 + 7]
    lo = _mm_hadd_ps(lo, lo); // [0 + 4 + 1 + 5+ 2 + 6 + 3 + 7, ... ]
    return _mm_cvtss_f32(lo); // return the first value 
}

NeighbourAcc simd_acculumate_neighbours(Vector3 self_pos, int self_idx, int start, int end, float * x, float * y, float * z, float * vx, float * vy, float * vz) {
    NeighbourAcc result = {};

    __m256 self_x = _mm256_set1_ps(self_pos.x);
    __m256 self_y = _mm256_set1_ps(self_pos.y);
    __m256 self_z = _mm256_set1_ps(self_pos.z);

    __m256 self_idx8 = _mm256_set1_ps((float) self_idx);

    __m256 lane_offsets = _mm256_setr_ps(0, 1, 2, 3, 4, 5, 6, 7);

    __m256 sep_x = _mm256_setzero_ps(), sep_y = _mm256_setzero_ps(), sep_z = _mm256_setzero_ps();
    __m256 align_x = _mm256_setzero_ps(), align_y = _mm256_setzero_ps(), align_z = _mm256_setzero_ps();
    __m256 coh_x = _mm256_setzero_ps(), coh_y = _mm256_setzero_ps(), coh_z = _mm256_setzero_ps();
    
    __m256 count = _mm256_setzero_ps();

    int j = start;

    for(; j + 8 <= end ; j+=8){
        __m256 ox = _mm256_loadu_ps(&x[j]);
        __m256 oy = _mm256_loadu_ps(&y[j]);
        __m256 oz = _mm256_loadu_ps(&z[j]);
        __m256 ovx = _mm256_loadu_ps(&vx[j]);
        __m256 ovy = _mm256_loadu_ps(&vy[j]);
        __m256 ovz = _mm256_loadu_ps(&vz[j]);

        __m256 j_idx = _mm256_add_ps(_mm256_set1_ps((float) j), lane_offsets);
        __m256 not_self = _mm256_cmp_ps(j_idx, self_idx8, _CMP_NEQ_OQ);

        __m256 dx = _mm256_sub_ps(ox, self_x);
        __m256 dy = _mm256_sub_ps(oy, self_y);
        __m256 dz = _mm256_sub_ps(oz, self_z);

        __m256 dist_sq = _mm256_add_ps(_mm256_mul_ps(dx, dx), 
                _mm256_add_ps(_mm256_mul_ps(dy, dy), _mm256_mul_ps(dz, dz)));
        __m256 inv_dist  = _mm256_rsqrt_ps(dist_sq);

        __m256 neighbour_mask = _mm256_and_ps(
                not_self, 
                _mm256_cmp_ps(
                    dist_sq, 
                    _mm256_set1_ps(NEIGHBOUR_RADIUS*NEIGHBOUR_RADIUS), _CMP_LT_OQ));

        __m256 sep_mask = _mm256_and_ps(
                not_self, 
                _mm256_and_ps(
                    _mm256_cmp_ps(
                        dist_sq, 
                        _mm256_set1_ps(SEPARATION_RADIUS*SEPARATION_RADIUS), _CMP_LT_OQ),
                    _mm256_cmp_ps(
                        _mm256_set1_ps(0.000001f * 0.000001f),
                        dist_sq, _CMP_LT_OQ)
                    )
                );

        align_x = _mm256_add_ps(align_x, _mm256_and_ps(ovx, neighbour_mask));
        align_y = _mm256_add_ps(align_y, _mm256_and_ps(ovy, neighbour_mask));
        align_z = _mm256_add_ps(align_z, _mm256_and_ps(ovz, neighbour_mask));

        coh_x = _mm256_add_ps(coh_x, _mm256_and_ps(ox, neighbour_mask));
        coh_y = _mm256_add_ps(coh_y, _mm256_and_ps(oy, neighbour_mask));
        coh_z = _mm256_add_ps(coh_z, _mm256_and_ps(oz, neighbour_mask));

        sep_x = _mm256_add_ps(sep_x, _mm256_and_ps(
                    _mm256_mul_ps(dx, inv_dist), sep_mask));
        sep_y = _mm256_add_ps(sep_y, _mm256_and_ps(
                    _mm256_mul_ps(dy, inv_dist), sep_mask));
        sep_z = _mm256_add_ps(sep_z, _mm256_and_ps(
                    _mm256_mul_ps(dz, inv_dist), sep_mask));

        count = _mm256_add_ps(count, _mm256_and_ps(_mm256_set1_ps(1.0f), neighbour_mask));
    }
    
    NeighbourAcc tail_result = acculumate_neighbours(self_pos, self_idx, j, end, x, y, z, vx, vy, vz);
    
    result.neighbours = hsum256_ps(count) + tail_result.neighbours;
    result.separation = (Vector3){
        .x = (-1.0f * hsum256_ps(sep_x)) + tail_result.separation.x, // since separation is self - other and dx and dy were other - self
        .y = (-1.0f * hsum256_ps(sep_y)) + tail_result.separation.y,
        .z = (-1.0f * hsum256_ps(sep_z)) + tail_result.separation.z,
    };
    result.cohesion_sum = (Vector3){
        .x = hsum256_ps(coh_x) + tail_result.cohesion_sum.x,
        .y = hsum256_ps(coh_y) + tail_result.cohesion_sum.y,
        .z = hsum256_ps(coh_z) + tail_result.cohesion_sum.z,
    };
    result.align_sum = (Vector3) {
        .x = hsum256_ps(align_x) + tail_result.align_sum.x,
        .y = hsum256_ps(align_y) + tail_result.align_sum.y,
        .z = hsum256_ps(align_z) + tail_result.align_sum.z,
    };

    return result;
}

void update(float dt, App * app){

    ThreadPerformance * perf = LANE_PERFORMANCE();
    struct timespec start_time = {}, end_time = {}, avg_neighbour_calc_time = {};

    clock_gettime(CLOCK_MONOTONIC, &start_time);

    size_t thread_start = 0;
    size_t thread_end = 0;

    lane_range(0, APP()->boid_count, &thread_start, &thread_end);

    for(size_t i = thread_start; i < thread_end ; i++){
        
        int x = (int) floorf(BOID_SOA().x[i] / CELL_SIZE);
        int y = (int) floorf(BOID_SOA().y[i] / CELL_SIZE);
        int z = (int) floorf(BOID_SOA().z[i] / CELL_SIZE);

        Vector3 self_pos = {BOID_SOA().x[i], BOID_SOA().y[i], BOID_SOA().z[i]};
        Vector3 self_vel = {BOID_SOA().vx[i], BOID_SOA().vy[i], BOID_SOA().vz[i]};

        Vector3 separation = {};
        Vector3 align_sum = {};
        Vector3 cohesion_sum = {};

        int neighbours = 0;
        
        struct timespec start_time , end_time = {};

        clock_gettime(CLOCK_MONOTONIC, &start_time);
        {
// performance showed that simpler passing around has better perforamnce
// #define ARENA_VECTORIZED 
#ifndef ARENA_VECTORIZED 

            for (int dy = -1 ; dy <= 1; dy++){
                for(int dz = -1; dz <= 1; dz++){
                    int start = offset(&APP()->grid_soa, (y + dy) * GRID_W * GRID_Z + (z + dz) * GRID_W + x - 1);
                    int end = offset(&APP()->grid_soa, (y + dy) * GRID_W  * GRID_Z + (z + dz) * GRID_W + x + 2);

                    NeighbourAcc this_cell= simd_acculumate_neighbours(self_pos, i, start, end,
                            BOID_SOA().x,
                            BOID_SOA().y,
                            BOID_SOA().z,
                            BOID_SOA().vx,
                            BOID_SOA().vy,
                            BOID_SOA().vz
                            );

                    separation = Vector3Add(separation, this_cell.separation);
                    align_sum = Vector3Add(align_sum, this_cell.align_sum);
                    cohesion_sum = Vector3Add(cohesion_sum, this_cell.cohesion_sum);
                    neighbours += this_cell.neighbours;
                }
            }
#else
            int boid_list_size = 0;
            for (int dy = -1 ; dy <= 1; dy++){
                int start = offset(&APP()->grid_soa, (y + dy) * GRID_W + x - 1);
                int end = offset(&APP()->grid_soa, (y + dy) * GRID_W + x + 2);
                boid_list_size += end - start;
            }


            // create space for entire grid
            arena_reset(THREAD_ARENA_PTR());
            float * xarr = ARENA_PUSH_TYPE_ARRAY(THREAD_ARENA_PTR(), float, boid_list_size);
            float * yarr = ARENA_PUSH_TYPE_ARRAY(THREAD_ARENA_PTR(), float, boid_list_size);
            float * vxarr = ARENA_PUSH_TYPE_ARRAY(THREAD_ARENA_PTR(), float, boid_list_size);
            float * vyarr = ARENA_PUSH_TYPE_ARRAY(THREAD_ARENA_PTR(), float, boid_list_size);

            // now we populate stuff 

            size_t memoffset = 0;
            int self_idx_in_arrs = -1;
            for (int dy = -1 ; dy <= 1; dy++){

                int start = offset(&APP()->grid_soa, (y + dy) * GRID_W + x - 1);
                int end = offset(&APP()->grid_soa, (y + dy) * GRID_W + x + 2);

                memcpy(xarr + memoffset, BOID_SOA().x + start, sizeof(float) * (end - start));
                memcpy(yarr + memoffset, BOID_SOA().y + start, sizeof(float) * (end - start));
                memcpy(vxarr + memoffset, BOID_SOA().vx + start, sizeof(float) * (end - start));
                memcpy(vyarr + memoffset, BOID_SOA().vy + start, sizeof(float) * (end - start));

                memoffset += end - start;

                if (start <= i && end > i) {
                    self_idx_in_arrs = memoffset + (i - start);
                }
            }

            NeighbourAcc this_cell= simd_acculumate_neighbours(
                    self_pos, self_idx_in_arrs, 0, boid_list_size, 
                    xarr, yarr, vxarr, vyarr);

            separation = Vector2Add(separation, this_cell.separation);
            align_sum = Vector2Add(align_sum, this_cell.align_sum);
            cohesion_sum = Vector2Add(cohesion_sum, this_cell.cohesion_sum);
            neighbours += this_cell.neighbours;
#endif
#undef ARENA_VECTORIZED
        }
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        struct timespec delta = {};
        timespec_delta(&start_time, &end_time, &delta);
        avg_neighbour_calc_time.tv_nsec  += delta.tv_nsec;
        avg_neighbour_calc_time.tv_sec  += delta.tv_sec;


        Vector3 accel = {0};
        if (neighbours > 0){
            Vector3 avg_vel = Vector3Scale(align_sum, 1.0f / neighbours);
            Vector3 center  = Vector3Scale(cohesion_sum, 1.0f / neighbours);
            Vector3 toward_center = Vector3Subtract(center, self_pos);

            accel = Vector3Add(accel, Vector3Scale(separation, SEPARATION_WEIGHT));
            accel = Vector3Add(accel, Vector3Scale(Vector3Subtract(avg_vel, self_vel), ALIGNMENT_WEIGHT));
            accel = Vector3Add(accel, Vector3Scale(toward_center, COHESION_WEIGHT));
        }

#define NOISE_STRENGTH 0.3f

        Vector3 vel = Vector3Add(self_vel, Vector3Scale(accel, dt));
        vel = Vector3ClampValue(vel, MIN_SPEED, MAX_SPEED);
        Vector3 pos = Vector3Add(self_pos, Vector3Scale(vel, dt));

        if (pos.x < 0.0f) {
            pos.x += WORLD_WIDTH;
        } else if (pos.x > WORLD_WIDTH) {
            pos.x -= WORLD_WIDTH;
        }

        if (pos.y < 0.0f) {
            pos.y += WORLD_HEIGHT;
        } else if (pos.y > WORLD_HEIGHT) {
            pos.y -= WORLD_HEIGHT;
        }

        if (pos.z < 0.0f){
            pos.z += WORLD_DEPTH;
        } else if (pos.z > WORLD_DEPTH){
            pos.z -= WORLD_DEPTH;
        }

        self_pos  = pos;
        self_vel  = vel;

        NEXT_BOID_SOA().x[i] = self_pos.x;
        NEXT_BOID_SOA().y[i] = self_pos.y;
        NEXT_BOID_SOA().z[i] = self_pos.z;
        NEXT_BOID_SOA().vx[i] = self_vel.x;
        NEXT_BOID_SOA().vy[i] = self_vel.y;
        NEXT_BOID_SOA().vz[i] = self_vel.z;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    struct timespec delta = {};
    timespec_delta(&start_time, &end_time, &delta);
    perf->ms_update_duration = (double) delta.tv_sec * (double)1000 + (double) delta.tv_nsec / (double)1000000.0;
    perf->ms_avg_neignbour_calcualtion = 
        ((double) avg_neighbour_calc_time.tv_sec * (double)1000 + 
         (double) avg_neighbour_calc_time.tv_nsec / (double)1000000.0) / 
        (double) (thread_end - thread_start);


    LANE_BARRIER();

    if (LANE_ID() == 0) { FLIP(); }
}

int thread_main() {
    LANE_BARRIER();

    if (LANE_ID() == 0) {
        init_window();
        init_renderer();
    }
    static atomic_int window_should_close = 0;

    double update_time = 0;
    double render_time = 0;
    double build_time = 0;

    while(!atomic_load_explicit(&window_should_close, memory_order_acquire)){

        LANE_BARRIER();

        if (LANE_ID() == 0) {
            int result = WindowShouldClose() ? 1 : 0;
            atomic_store_explicit(&window_should_close, result, memory_order_relaxed);

            APP()->delta_time = GetFrameTime();

            BeginDrawing();
            ClearBackground(BLUE);

            render_update_camera();

            char buffer[1024] = {};
            double render_start_time = GetTime();
            render();
            double render_stop_time = GetTime();
            render_time = render_stop_time - render_start_time;

            double build_start_time = GetTime();
            build_grid_soa(&APP()->grid_soa);
            double build_stop_time = GetTime();
            build_time = build_stop_time - build_start_time;

            
            DrawRectangle(0, 0, 300, 500,  (Color){255, 255, 255, 180});
            float yoffset = 10;

            int font_size = 10, y_increment = 15;
                
            unsigned int thread_count = LANE_COUNT(); // I don't  like the name thing which is happening here 
            double thread_update_avg = 0.0f;
            LaneGroup * grp = ctx.group;
            for (int i = 0 ; i < LANE_COUNT(); i++){

                thread_update_avg += grp->thread_performance[i].ms_update_duration;

                snprintf(buffer, 1024, "thread %d update time : %f", i, grp->thread_performance[i].ms_update_duration);
                DrawText(buffer, 10, yoffset, font_size, BLUE);
                yoffset += y_increment;

                snprintf(buffer, 1024, "thread %d avg update time : %f", i, grp->thread_performance[i].ms_avg_neignbour_calcualtion);
                DrawText(buffer, 10, yoffset, font_size, BLUE);
                yoffset += y_increment;
            }


            snprintf(buffer, 1024, "FPS: %d", GetFPS());
            DrawText(buffer, 10, yoffset, font_size, BLUE);
            yoffset += y_increment;

            snprintf(buffer, 1024,  "Render: %lf ms", render_time * 1000.0);
            DrawText(buffer, 10, yoffset, font_size, BLUE);
            yoffset += y_increment;

            snprintf(buffer, 1024,  "Build : %lf ms", build_time * 1000.0);
            DrawText(buffer, 10, yoffset, font_size, BLUE);
            yoffset += y_increment;

            snprintf(buffer, 1024,  "Update : %lf ms", thread_update_avg / LANE_COUNT());
            DrawText(buffer, 10, yoffset, font_size, BLUE);
            yoffset += y_increment;

            EndDrawing();

        }

        LANE_BARRIER();

        double update_start_time = 0, update_stop_time = 0;
        if (LANE_ID() == 0) {
            update_start_time = GetTime();
        }
        update(APP()->delta_time, APP());
        if (LANE_ID() == 0) { 
            update_stop_time = GetTime();
            update_time = update_stop_time - update_start_time;
        }

    }
    return 0;
}

void * _thread_start(void * data) {
    ctx = ((_Thread_Data *) data)->ctx;
    free(data);
    thread_main();
    return NULL;
}


#define THREAD_COUNT 11
int main(){

    App app = {};
    app.front = 0;

    Reserve reserve = {};
    reserve_init(&reserve, GB(4));

    Arena thread_arenas[THREAD_COUNT]= {};
    for (int i = 0; i < THREAD_COUNT; i++){
        reserve_alloc_subarena(&reserve, &thread_arenas[i], MB(200));
    }

    ThreadPerformance thread_perforamnce[THREAD_COUNT] = {};

    LaneGroup lane_group =  {};
    lane_group.thread_count = THREAD_COUNT;
    lane_group.app = &app;
    lane_group.thread_performance = (ThreadPerformance *)&thread_perforamnce;
    barrier_init(&lane_group.barrier, THREAD_COUNT);

    pthread_t other_threads[THREAD_COUNT - 1];
    for(int i = 0 ;i < THREAD_COUNT - 1; i++){
        _Thread_Data * thread_data=  (_Thread_Data *) malloc(sizeof(_Thread_Data));
        thread_data->ctx.group = &lane_group;
        thread_data->ctx.lane_id = i + 1;
        thread_data->ctx.arena = &thread_arenas[i + 1];
        pthread_create(&other_threads[i], NULL, _thread_start, thread_data);
    }

    _Thread_Data * thread_data=  (_Thread_Data *) malloc(sizeof(_Thread_Data));
    thread_data->ctx.group = &lane_group;
    thread_data->ctx.lane_id = 0;
    thread_data->ctx.arena = &thread_arenas[0];
    _thread_start((void *)thread_data);


    for(int i =0 ; i < THREAD_COUNT - 1; i++){
        pthread_join(other_threads[i], NULL);
    }

    return 0;
}

