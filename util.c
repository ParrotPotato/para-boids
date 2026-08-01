#pragma once

#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdalign.h>

#define KB(x) ((x) * 1024LLU)
#define MB(x) ((KB(x)) * 1024LLU)
#define GB(x) ((MB(x)) * 1024LLU)

size_t page_size() {
    static size_t size = 0;
    if (size == 0) size = sysconf(_SC_PAGE_SIZE);
    return size;
}

size_t round_up_to_factor(size_t number, size_t factor){
    return ((number + factor - 1) / factor) * factor;
}

typedef struct {
    void * ptr;
    size_t size;
    size_t used;
} Reserve;

typedef struct {
    void * ptr;
    size_t size;
    size_t used;
    size_t commited;
} Arena;

void reserve_init(Reserve * reserve, size_t size) {
    if (!reserve) exit(1);

    size_t aligned_size = round_up_to_factor(size, page_size());

    reserve->ptr = mmap(NULL, aligned_size, PROT_NONE , MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (reserve->ptr == MAP_FAILED){
        perror("mmap");
        exit(1);
    }
    reserve->size = aligned_size;
    reserve->used = 0;
}

void reserve_reset(Reserve * reserve){
    reserve->used = 0;
}


void reserve_alloc_subarena(Reserve * reserve, Arena * arena, size_t size){
    if (!reserve || !arena) exit(1);

    size_t aligned_used = round_up_to_factor(reserve->used, page_size());
    size_t aligned_size = round_up_to_factor(size, page_size());

    if (aligned_used + aligned_size > reserve->size){
        fprintf(stderr, "reserve full! needed %zu, have %zu\n", aligned_size, reserve->size - aligned_used);
        exit(1);
    }

    arena->ptr = (char *) reserve->ptr + aligned_used;
    arena->size = aligned_size;
    arena->used = 0;
    arena->commited = 0;

    reserve->used = aligned_used + aligned_size;
    return;
}

void * arena_alloc(Arena * arena, size_t size, size_t alignment){
    if (!arena) exit(1);

    size_t aligned_commited = round_up_to_factor(arena->commited, page_size());
    size_t aligned_used = round_up_to_factor(arena->used, alignment);
    size_t total_needed = (aligned_used - arena->used) + size;

    if (arena->used + total_needed > arena->size){
        fprintf(stderr, "arena full! needed %zu (with alignment), have %zu\n", total_needed, arena->size - arena->used);
        exit(1);
    }

    void * ptr = (char *) arena->ptr + aligned_used;
    if (arena->used + total_needed > aligned_commited) {
        size_t commit_start = round_up_to_factor(arena->commited, page_size());
        size_t commit_end = round_up_to_factor(aligned_used + size, page_size());
        size_t to_commit = commit_end - commit_start;
        if (to_commit > 0){
            void * commit_ptr = (char *) arena->ptr + commit_start;
            if (mprotect(commit_ptr, to_commit, PROT_READ | PROT_WRITE) == -1) {
                perror("mprotect");
                exit(1);
            }
            arena->commited += to_commit;
        }
    } 
    arena->used = aligned_used + size;
    
    return ptr;
}

void arena_reset(Arena * arena) {
    arena->used = 0;
}

#define ARENA_PUSH_TYPE_ARRAY(arena, type, count) ((type *) arena_alloc((arena), sizeof(type) * count, alignof(type)))
#define ARENA_PUSH_TYPE(arena, type)              ((type *) arena_alloc((arena), sizeof(type) , alignof(type)))
