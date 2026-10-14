/* Optional RViz-only GLX presentation timing. Not loaded in the simulator.
 * Build: cc -shared -fPIC -O2 rviz_frame_probe.c -o probe.so -ldl -pthread
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef void (*swap_function)(void *, unsigned long);
typedef void (*query_function)(void *, unsigned long, int, unsigned int *);
static swap_function real_swap;
static query_function real_query;
static FILE *frames;
static pthread_once_t once = PTHREAD_ONCE_INIT;

static void initialize(void) {
    real_swap = (swap_function)dlsym(RTLD_NEXT, "glXSwapBuffers");
    real_query = (query_function)dlsym(RTLD_NEXT, "glXQueryDrawable");
    const char *path = getenv("DOUGIE_RVIZ_FRAME_LOG");
    if (path) {
        frames = fopen(path, "w");
        if (frames) setvbuf(frames, NULL, _IOLBF, 0);
    }
}

void glXSwapBuffers(void *display, unsigned long drawable) {
    pthread_once(&once, initialize);
    if (!real_swap) {
        fputs("Dougie frame probe could not resolve glXSwapBuffers\n", stderr);
        abort();
    }
    real_swap(display, drawable);
    if (frames) {
        struct timespec stamp;
        unsigned int width = 0, height = 0;
        if (real_query) {
            real_query(display, drawable, 0x801D /* GLX_WIDTH */, &width);
            real_query(display, drawable, 0x801E /* GLX_HEIGHT */, &height);
        }
        clock_gettime(CLOCK_MONOTONIC, &stamp);
        fprintf(frames, "%.9f,%lu,%u,%u\n", stamp.tv_sec + stamp.tv_nsec * 1e-9, drawable, width, height);
    }
}

__attribute__((destructor)) static void close_log(void) {
    if (frames) fclose(frames);
}
