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
    size_t  part;
    Color   color;
} Boid ; 

#define WORLD_WIDTH  1600
#define WORLD_HEIGHT 900

#define NEIGHBOUR_RADIUS  30.0f
#define SEPARATION_RADIUS 15.0f
#define SEPARATION_WEIGHT 14.0f
#define ALIGNMENT_WEIGHT  8.0f
#define COHESION_WEIGHT   3.0f
#define MAX_SPEED         100.0f
#define MIN_SPEED         50.0f


#define CELL_SIZE NEIGHBOUR_RADIUS  
#define GRID_W ((int) (WORLD_WIDTH / CELL_SIZE) + 1)
#define GRID_H ((int) (WORLD_HEIGHT / CELL_SIZE) + 1)
#define GRID_CELLS (GRID_W * GRID_H)

#define MAX_BOID_COUNT  20000

typedef struct {
    int head[GRID_CELLS];
    int next[MAX_BOID_COUNT];
} Grid;

typedef struct {
    Boid boids[MAX_BOID_COUNT];
    size_t boid_count;
    Grid grid;
    float delta_time;
} App ;

#define APP() ((App *)ctx.group->app)

void init_window() {
    InitWindow(1600, 900, "main window");
    SetTargetFPS(60);

    for (int i = 0 ; i < MAX_BOID_COUNT; i++){
        Boid * boid = &APP()->boids[i];

        boid->pos = (Vector2){
            GetRandomValue(0, WORLD_WIDTH * 1000)  / 1000.0f,
            GetRandomValue(0, WORLD_HEIGHT * 1000) / 1000.0f};
        boid->vel = Vector2Normalize((Vector2){
            GetRandomValue(-1000, 1000) / 1000.0f,
            GetRandomValue(-1000, 1000) / 1000.0f
        });

        float cell_size = 100.0f;
        int x = (int)(floorf(boid->pos.x / cell_size));
        int y = (int)(floorf(boid->pos.y / cell_size));

        int offset = y * (GetScreenWidth() / 100.0f)  + x;

        boid->part = offset;

        Color color = colors[boid->part % COLOR_COUNT];
        color.a = 180.0f;
        boid->color = color;
    }
    APP()->boid_count = MAX_BOID_COUNT;
}

void render() {
    for (int i = 0 ; i < APP()->boid_count; i++){
        const Boid * boid = &APP()->boids[i];
        float angle = atan2(boid->vel.y, boid->vel.x);
        DrawPoly(boid->pos, 3, 5, angle * RAD2DEG, (Color){255, 255, 255, 80});
    }
}

void build_grid(Grid * grid, App * app){
    for (int c = 0 ; c < GRID_CELLS ; c++) grid->head[c] = -1;
    for (size_t i = 0 ; i < app->boid_count; i++) {
        int x = (int) floorf(app->boids[i].pos.x / CELL_SIZE);
        int y = (int) floorf(app->boids[i].pos.y / CELL_SIZE);
        int cell = y * GRID_W + x;
        grid->next[i] = grid->head[cell];
        grid->head[cell] = (int)i;
    }
}

void update(float delta_time, App * app, Grid * grid){
    size_t thread_start = 0;
    size_t thread_end = 0;

    lane_range(0, APP()->boid_count, &thread_start, &thread_end);
    
    arena_reset(THREAD_ARENA_PTR());

    Boid * boids = ARENA_PUSH_TYPE_ARRAY(THREAD_ARENA_PTR(), Boid, thread_end - thread_start);
    size_t boid_count = thread_end - thread_start;

    for(size_t i = thread_start; i < thread_end ; i++){
        Boid * self = &APP()->boids[i];
        
        int x = (int) floorf(app->boids[i].pos.x / CELL_SIZE);
        int y = (int) floorf(app->boids[i].pos.y / CELL_SIZE);

        Vector2 separation = {};
        Vector2 align_sum = {};
        Vector2 cohesion_sum = {};

        int neighbours = 0;

        for (int dy = -1 ; dy <= 1; dy++){
            for (int dx = -1; dx <= 1; dx++){

                int nx = x + dx, ny = y + dy;

                if (nx < 0 || ny < 0 || nx >= GRID_W || ny >= GRID_H) continue;
                int cell = ny * GRID_W + nx;

                for (int j = grid->head[cell]; j != -1; j = grid->next[j]) {
                    if ((size_t) j == i) continue;
                    Boid * other = &APP()->boids[j];
                    float dist = Vector2Distance(self->pos, other->pos);
                    if (dist >= NEIGHBOUR_RADIUS) continue;

                    if (dist < SEPARATION_RADIUS && dist > 0.000001f) {
                        Vector2 away = Vector2Scale(Vector2Subtract(self->pos, other->pos), 1.0f/dist);
                        separation = Vector2Add(separation, away);
                    }

                    align_sum = Vector2Add(align_sum, other->vel);
                    cohesion_sum = Vector2Add(cohesion_sum, other->pos);
                    neighbours++;
                }
            }
        }

        Vector2 accel = {0};
        if (neighbours > 0){
            Vector2 avg_vel = Vector2Scale(align_sum, 1.0f / neighbours);
            Vector2 center  = Vector2Scale(cohesion_sum, 1.0f / neighbours);
            Vector2 toward_center = Vector2Subtract(center, self->pos);

            accel = Vector2Add(accel, Vector2Scale(separation, SEPARATION_WEIGHT));
            accel = Vector2Add(accel, Vector2Scale(Vector2Subtract(avg_vel, self->vel), ALIGNMENT_WEIGHT));
            accel = Vector2Add(accel, Vector2Scale(toward_center, COHESION_WEIGHT));
        }

#define NOISE_STRENGTH 0.3f
        Vector2 noise = (Vector2) {
            (GetRandomValue(-100, 100) / 100.0f * NOISE_STRENGTH),
            (GetRandomValue(-100, 100) / 100.0f * NOISE_STRENGTH),
        };
        accel = Vector2Add(accel, noise);

        Vector2 vel = Vector2Add(self->vel, Vector2Scale(accel, delta_time));
        vel = Vector2ClampValue(vel, MIN_SPEED, MAX_SPEED);
        Vector2 pos = Vector2Add(self->pos, Vector2Scale(vel, delta_time));

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

        self->pos = pos;
        self->vel = vel;

        boids[i - thread_start].pos = self->pos;
        boids[i - thread_start].vel = self->vel;
    }

    LANE_BARRIER();

    for(size_t i = thread_start ; i < thread_end ; i++){
        APP()->boids[i] = boids[i - thread_start];
    }
}

int thread_main() {
    LANE_BARRIER();

    if (LANE_ID() == 0) init_window();
    static atomic_int window_should_close = 0;

    while(!atomic_load_explicit(&window_should_close, memory_order_acquire)){
        LANE_BARRIER();
        if (LANE_ID() == 0) {
            int result = WindowShouldClose() ? 1 : 0;
            atomic_store_explicit(&window_should_close, result, memory_order_relaxed);

            APP()->delta_time = GetFrameTime();

            BeginDrawing();
            ClearBackground(BLACK);

            render();

            char buffer[1024] = {};
            snprintf(buffer, 1024, "FPS: %d", GetFPS());
            DrawText(buffer, 10, 10, 10, RED);

            EndDrawing();


            build_grid(&APP()->grid, APP());
        }

        LANE_BARRIER();

        update(APP()->delta_time, APP(), &APP()->grid);

    }
    return 0;
}

void * _thread_start(void * data) {
    ctx = ((_Thread_Data *) data)->ctx;
    free(data);
    thread_main();
    return NULL;
}


#define THREAD_COUNT 8
int main(){

    App app = {};

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

