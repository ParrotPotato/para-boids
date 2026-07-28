#include <immintrin.h>
#include <math.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <assert.h>
#include <stdalign.h>

#include <raylib.h>
#include <raymath.h>

#include "util.c"


#define COLOR_COUNT 19
Color colors[] = {
 YELLOW,
 GOLD,
 ORANGE,
 PINK,
 RED,
 MAROON,
 GREEN,
 LIME,
 DARKGREEN,
 SKYBLUE,
 BLUE,
 DARKBLUE,
 PURPLE,
 VIOLET,
 DARKPURPLE,
 BEIGE,
 BROWN,
 DARKBROWN,
 MAGENTA,
};

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
    Barrier barrier;
    size_t thread_count;
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

#define NEIGHBOUR_RADIUS  30.0f
#define SEPARATION_RADIUS 10.0f
#define SEPARATION_WEIGHT 8.0f
#define ALIGNMENT_WEIGHT  5.0f
#define COHESION_WEIGHT   2.0f
#define MAX_SPEED         100.0f
#define MIN_SPEED         50.0f


#define CELL_SIZE NEIGHBOUR_RADIUS  
#define GRID_W ((int) (WORLD_WIDTH / CELL_SIZE) + 1)
#define GRID_H ((int) (WORLD_HEIGHT / CELL_SIZE) + 1)
#define GRID_CELLS (GRID_W * GRID_H)

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
    float vx[MAX_BOID_COUNT];
    float vy[MAX_BOID_COUNT];
} BoidSOA;

typedef struct {
    BoidSOA boid_soa[2];
    int front;
    size_t boid_count;
    GridSOA grid_soa;
//    Grid grid;
    float delta_time;
} App;

#define APP()              ((App *)ctx.group->app)
#define BOID_SOA()         ((APP())->boid_soa[(APP())->front])
#define NEXT_BOID_SOA()    ((APP())->boid_soa[((APP())->front + 1) % 2])
#define FLIP()             (APP())->front = (((APP())->front + 1) % 2)

void init_window() {
    InitWindow(1600, 900, "para boid");
    SetTargetFPS(144);

    for (int i = 0 ; i < MAX_BOID_COUNT; i++){

        float x =GetRandomValue(0, WORLD_WIDTH * 1000)  / 1000.0f;
        float y = GetRandomValue(0, WORLD_HEIGHT * 1000) / 1000.0f;

        float vx = GetRandomValue(-1000, 1000) / 1000.0f;
        float vy = GetRandomValue(-1000, 1000) / 1000.0f;
        
        Vector2 vel = Vector2Normalize((Vector2){vx, vy});

        BOID_SOA().x[i] = x;
        BOID_SOA().y[i] = y;
        BOID_SOA().vx[i] = vel.x;
        BOID_SOA().vy[i] = vel.y;
        
    }
    APP()->boid_count = MAX_BOID_COUNT;
}

void render() {
    for (int i = 0 ; i < APP()->boid_count; i++){
        DrawPixel(BOID_SOA().x[i], BOID_SOA().y[i],  WHITE);
    }
}

void build_grid_soa(GridSOA * grid_soa) {
    int insert[GRID_CELLS] = {};

    for(size_t i = 0 ; i < GRID_CELLS; i++) grid_soa->_offset[i] = 0;

    for(size_t i = 0 ; i < MAX_BOID_COUNT; i++) {
        int x = (int) floorf(BOID_SOA().x[i] / CELL_SIZE);
        int y = (int) floorf(BOID_SOA().y[i] / CELL_SIZE);
        int cell = y * GRID_W + x;
        grid_soa->_offset[cell] += 1;
        insert[cell] += 1;
    }

    int cummulative = 0;
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
        int cell = y * GRID_W + x;

        int offset = insert[cell];
        insert[cell] += 1;
        grid_soa->indices[offset] = i;

        NEXT_BOID_SOA().x[offset] = BOID_SOA().x[i];
        NEXT_BOID_SOA().y[offset] = BOID_SOA().y[i];
        NEXT_BOID_SOA().vx[offset] = BOID_SOA().vx[i];
        NEXT_BOID_SOA().vy[offset] = BOID_SOA().vy[i];
    }

    FLIP();
}

typedef struct {
    Vector2 separation, align_sum, cohesion_sum;
    int neighbours;
}NeighbourAcc;

NeighbourAcc acculumate_neighbours(Vector2 self_pos, int self_idx, int start, int end) {
    NeighbourAcc result = {};
    for (int j = start; j < end; j++){
        if (j == self_idx) continue;

        Vector2 other_pos = {BOID_SOA().x[j], BOID_SOA().y[j]};
        Vector2 other_vel = {BOID_SOA().vx[j], BOID_SOA().vy[j]};

        float dist_sq =  Vector2DistanceSqr(self_pos, other_pos);
        if (dist_sq >= NEIGHBOUR_RADIUS * NEIGHBOUR_RADIUS) continue;

        if (dist_sq < SEPARATION_RADIUS * SEPARATION_RADIUS && dist_sq > 0.000001f * 0.000001f) {
            float dist = sqrtf(dist_sq);
            Vector2 away = Vector2Scale(Vector2Subtract(self_pos, other_pos), 1.0f/dist);
            result.separation = Vector2Add(result.separation, away);
        }

        result.align_sum = Vector2Add(result.align_sum, other_vel);
        result.cohesion_sum = Vector2Add(result.cohesion_sum, other_pos);
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

NeighbourAcc simd_acculumate_neighbours(Vector2 self_pos, int self_idx, int start, int end) {
    NeighbourAcc result = {};

    __m256 self_x = _mm256_set1_ps(self_pos.x);
    __m256 self_y = _mm256_set1_ps(self_pos.y);
    __m256 self_idx8 = _mm256_set1_ps((float) self_idx);

    __m256 lane_offsets = _mm256_setr_ps(0, 1, 2, 3, 4, 5, 6, 7);

    __m256 sep_x = _mm256_setzero_ps(), sep_y = _mm256_setzero_ps();
    __m256 align_x = _mm256_setzero_ps(), align_y = _mm256_setzero_ps();
    __m256 coh_x = _mm256_setzero_ps(), coh_y = _mm256_setzero_ps();
    __m256 count = _mm256_setzero_ps();

    int j = start;

    for(; j + 8 <= end ; j+=8){
        __m256 ox = _mm256_loadu_ps(&BOID_SOA().x[j]);
        __m256 oy = _mm256_loadu_ps(&BOID_SOA().y[j]);
        __m256 ovx = _mm256_loadu_ps(&BOID_SOA().vx[j]);
        __m256 ovy = _mm256_loadu_ps(&BOID_SOA().vy[j]);

        __m256 j_idx = _mm256_add_ps(_mm256_set1_ps((float) j), lane_offsets);
        __m256 not_self = _mm256_cmp_ps(j_idx, self_idx8, _CMP_NEQ_OQ);

        __m256 dx = _mm256_sub_ps(ox, self_x);
        __m256 dy = _mm256_sub_ps(oy, self_y);
        __m256 dist_sq = _mm256_add_ps(_mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
        __m256 inv_dist  = _mm256_rsqrt_ps(dist_sq);

        __m256 valid = _mm256_and_ps(
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

        align_x = _mm256_add_ps(align_x, _mm256_and_ps(ovx, valid));
        align_y = _mm256_add_ps(align_y, _mm256_and_ps(ovy, valid));
        coh_x = _mm256_add_ps(coh_x, _mm256_and_ps(ox, valid));
        coh_y = _mm256_add_ps(coh_y, _mm256_and_ps(oy, valid));

        sep_x = _mm256_add_ps(sep_x, _mm256_and_ps(
                    _mm256_mul_ps(dx, inv_dist), sep_mask));
        sep_y = _mm256_add_ps(sep_y, _mm256_and_ps(
                    _mm256_mul_ps(dy, inv_dist), sep_mask));

        count = _mm256_add_ps(count, _mm256_and_ps(_mm256_set1_ps(1.0f), valid));
    }
    
    NeighbourAcc tail_result = acculumate_neighbours(self_pos, self_idx, j, end);
    
    result.neighbours = hsum256_ps(count) + tail_result.neighbours;
    result.separation = (Vector2){
        .x = (-1.0f * hsum256_ps(sep_x)) + tail_result.separation.x, // since separation is self - other and dx and dy were other - self
        .y = (-1.0f * hsum256_ps(sep_y)) + tail_result.separation.y,
    };
    result.cohesion_sum = (Vector2){
        .x = hsum256_ps(coh_x) + tail_result.cohesion_sum.x,
        .y = hsum256_ps(coh_y) + tail_result.cohesion_sum.y,
    };
    result.align_sum = (Vector2) {
        .x = hsum256_ps(align_x) + tail_result.align_sum.x,
        .y = hsum256_ps(align_y) + tail_result.align_sum.y,
    };

    return result;
}

void update(float delta_time, App * app){
    size_t thread_start = 0;
    size_t thread_end = 0;

    lane_range(0, APP()->boid_count, &thread_start, &thread_end);
    arena_reset(THREAD_ARENA_PTR());

    for(size_t i = thread_start; i < thread_end ; i++){
        
        int x = (int) floorf(BOID_SOA().x[i] / CELL_SIZE);
        int y = (int) floorf(BOID_SOA().y[i] / CELL_SIZE);

        Vector2 self_pos = {BOID_SOA().x[i], BOID_SOA().y[i]};
        Vector2 self_vel = {BOID_SOA().vx[i], BOID_SOA().vy[i]};

        Vector2 separation = {};
        Vector2 align_sum = {};
        Vector2 cohesion_sum = {};

        int neighbours = 0;

        for (int dy = -1 ; dy <= 1; dy++){

            //for (int dx = -1; dx <= 1; dx++){

            //    int nx = x + dx, ny = y + dy;

            //    if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) continue;
            //    int cell = ny * GRID_W + nx;

            //    int start = APP()->grid_soa.offset[cell];
            //    int end = APP()->grid_soa.offset[cell + 1];

            //    NeighbourAcc this_cell= simd_acculumate_neighbours(self_pos, i, start, end);
            //    // NeighbourAcc this_cell= acculumate_neighbours(self_pos, i, start, end);
            //    
            //    separation = Vector2Add(separation, this_cell.separation);
            //    align_sum = Vector2Add(align_sum, this_cell.align_sum);
            //    cohesion_sum = Vector2Add(cohesion_sum, this_cell.cohesion_sum);
            //    neighbours += this_cell.neighbours;
            //}

            int start = offset(&APP()->grid_soa, (y + dy) * GRID_W + x - 1);
            int end = offset(&APP()->grid_soa, (y + dy) * GRID_W + x + 2);

            NeighbourAcc this_cell= simd_acculumate_neighbours(self_pos, i, start, end);
            
            separation = Vector2Add(separation, this_cell.separation);
            align_sum = Vector2Add(align_sum, this_cell.align_sum);
            cohesion_sum = Vector2Add(cohesion_sum, this_cell.cohesion_sum);
            neighbours += this_cell.neighbours;
            
        }

        Vector2 accel = {0};
        if (neighbours > 0){
            Vector2 avg_vel = Vector2Scale(align_sum, 1.0f / neighbours);
            Vector2 center  = Vector2Scale(cohesion_sum, 1.0f / neighbours);
            Vector2 toward_center = Vector2Subtract(center, self_pos);

            accel = Vector2Add(accel, Vector2Scale(separation, SEPARATION_WEIGHT));
            accel = Vector2Add(accel, Vector2Scale(Vector2Subtract(avg_vel, self_vel), ALIGNMENT_WEIGHT));
            accel = Vector2Add(accel, Vector2Scale(toward_center, COHESION_WEIGHT));
        }

#define NOISE_STRENGTH 0.3f
        //Vector2 noise = (Vector2) {
        //    (GetRandomValue(-100, 100) / 100.0f * NOISE_STRENGTH),
        //    (GetRandomValue(-100, 100) / 100.0f * NOISE_STRENGTH),
        //};
        //accel = Vector2Add(accel, noise);

        Vector2 vel = Vector2Add(self_vel, Vector2Scale(accel, delta_time));
        vel = Vector2ClampValue(vel, MIN_SPEED, MAX_SPEED);
        Vector2 pos = Vector2Add(self_pos, Vector2Scale(vel, delta_time));

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

        self_pos  = pos;
        self_vel  = vel;

        NEXT_BOID_SOA().x[i] = self_pos.x;
        NEXT_BOID_SOA().y[i] = self_pos.y;
        NEXT_BOID_SOA().vx[i] = self_vel.x;
        NEXT_BOID_SOA().vy[i] = self_vel.y;
    }

    LANE_BARRIER();

    if (LANE_ID() == 0) { FLIP(); }
}

int thread_main() {
    LANE_BARRIER();

    if (LANE_ID() == 0) init_window();
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
            ClearBackground(BLACK);

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

            snprintf(buffer, 1024, "FPS: %d", GetFPS());
            DrawText(buffer, 10, 10, 20, BLUE);

            snprintf(buffer, 1024,  "Render: %lf ms", render_time * 1000.0);
            DrawText(buffer, 10, 35, 20, BLUE);
            
            snprintf(buffer, 1024,  "Build : %lf ms", build_time * 1000.0);
            DrawText(buffer, 10, 60, 20, BLUE);

            snprintf(buffer, 1024,  "Update : %lf ms", update_time * 1000.0);
            DrawText(buffer, 10, 85, 20, BLUE);

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


#define THREAD_COUNT 10
int main(){

    App app = {};
    app.front = 0;

    Reserve reserve = {};
    reserve_init(&reserve, GB(4));

    Arena   thread_arenas[THREAD_COUNT]= {};
    for (int i = 0; i < THREAD_COUNT; i++){
        reserve_alloc_subarena(&reserve, &thread_arenas[i], MB(200));
    }

    LaneGroup lane_group =  {};
    lane_group.thread_count = THREAD_COUNT;
    lane_group.app = &app;
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

