#include <stdio.h>
#include <SDL2/SDL.h>
#include <math.h>
#include <stdlib.h>

#define BYTES_PER_PIXEL 4

#define NUM_PTS 4


typedef unsigned char byte;

typedef struct {
    int width;
    int height;
    int bytesPerPixel;
    int pitch;
    byte* pixels;
} PixelBuffer;


void sdl_update_window(SDL_Renderer* renderer, SDL_Texture* texture, PixelBuffer* pixBuff) {
    SDL_RenderClear(renderer);
    SDL_UpdateTexture(texture, 0, pixBuff->pixels, pixBuff->pitch);
    SDL_RenderCopy(renderer, texture, 0, 0);
    SDL_RenderPresent(renderer);
}


SDL_Texture* sdl_handle_window_resize(PixelBuffer* pixBuff, SDL_Texture* texture, SDL_Renderer* renderer, int width, int height) {
    if (texture) {
        SDL_DestroyTexture(texture);
        texture=0;
    }
    if (pixBuff->pixels) {
        free(pixBuff->pixels);
        pixBuff->pixels = 0;
    }

    SDL_Texture* res = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        width,
        height
    );

    pixBuff->width = width;
    pixBuff->pitch = width * BYTES_PER_PIXEL;
    pixBuff->height = height;
    pixBuff->pixels = (byte*) malloc(height * pixBuff->pitch);
    
    return res;
}


typedef struct {
    float x;
    float y;
} Vec2;


const Vec2 energyPoints[NUM_PTS] = {
    {0.3, 0.3},
    {-0.2,-0.2},
    {-0.6, 0.7},
    {0.3, -0.3}
};

Vec2 vec2Add(Vec2 a, Vec2 b) {
    Vec2 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    return res;
}


float scaledInvSquareDist(float x, float y, float s, float ax, float ay) {
    double distAx = (x - ax);
    double distAy = (y - ay);
    double distA = distAx * distAx + distAy * distAy;
    return (float) 1 / (1 + distA*s);
}

Vec2 gradScaledInvSquareDist(float x, float y, float s, float ax, float ay) {
    double distAx = (x - ax);
    double distAy = (y - ay);
    double denom = (1 + s * (distAx*distAx + distAy*distAy));
    double denomSq = denom * denom;
    double xGrad = -2 * s * distAx / denomSq;
    double yGrad = -2 * s * distAy / denomSq;
    Vec2 res = {xGrad, yGrad};
    return res;
}

float energyFunction(float x, float y) {
    float res = 0;
    for (int i=0; i < NUM_PTS; i++) {
        res += scaledInvSquareDist(x, y, 16.0, energyPoints[i].x, energyPoints[i].y);
    }
    return res; 
}


Vec2 gradEnergyFunction(float x, float y) {
    Vec2 res = gradScaledInvSquareDist(x, y, 16.0, energyPoints[0].x, energyPoints[0].y);
    for (int i=1; i < NUM_PTS; i++) {
        res = vec2Add(res, gradScaledInvSquareDist(x, y, 16.0, energyPoints[i].x, energyPoints[i].y));
    }
    return res;
}


/*
 *  TODO: Implement normal distribution (box muller transform)
 */
float randomNormal(float scale) {
    return scale * (((float) rand()) / RAND_MAX * 2 - 1);
}


/*
 *  Apply one step of langevin sampling to a point at pos.
 */
void langevinStep(Vec2* pos) {
    Vec2 grad = gradEnergyFunction(pos->x, pos->y);
    pos->x += 1e-2 * grad.x + randomNormal(1e-1);
    pos->y += 1e-2 * grad.y + randomNormal(1e-1);
}


void updateAndRender(PixelBuffer* pixBuff) {
    // Current position/state of langevin markov chain, start at 0
    static Vec2 curr = {0.0, 0.0};
    static float maxEnergy = 1;

    // Update current position/state
    langevinStep(&curr);

    // Draw energy surface, only really needs to change when the surface changes 
    // (which is currently never) but this works for now.
    int currRad = 10;
    byte* pixels = pixBuff->pixels;
    for (int y=0; y < pixBuff->height; y++) {
        for (int x=0; x < pixBuff->width; x++) {
            float rx = 2.0 * x / (float) pixBuff->width - 1;
            float ry = 2.0 * y / (float) pixBuff->height - 1;
            float energy = energyFunction(rx, ry);
            if (energy > maxEnergy) {
                maxEnergy = energy;
            }
            // Ensure energy < 1 for easier coloring. Will give one nonesense frame
            // at start but that doesn't matter rn
            energy = energy / maxEnergy;

            // Energy truncated to highlight contours
            float energyTrunc = (((byte) (energy * 255)) & 0xf8) / 255.0;

            *pixels++ = (byte) (energyTrunc * 0xe0);
            *pixels++ = (byte) (energyTrunc * 0x98);
            *pixels++ = (byte) (energyTrunc * 0xc6);
            *pixels++ = 0xff;
        }
    }


    // Draw a marker at the current position of the langevin chain

    int currYMid = ((int) (pixBuff->height * (curr.y + 1) / 2));
    int currYStart = currYMid-currRad;
    int currYEnd = currYMid+currRad;

    int currXMid = ((int) (pixBuff->width * (curr.x + 1) / 2));
    int currXStart = currXMid-currRad;
    int currXEnd = currXMid+currRad;

    for (int y=currYStart; y < currYEnd; y++) {
        pixels = (pixBuff->pixels + y * pixBuff->pitch + currXStart * pixBuff->bytesPerPixel);
        for (int x=currXStart; x < currXEnd; x++) {
            if (y >= 0 && y < pixBuff->height && x > 0 && x < pixBuff->width) {
                int distX = (x - currXMid);
                int distY = (y- currYMid);
                int distSq =  distX*distX + distY * distY;
                int shouldDraw = distSq < currRad * currRad;
                *pixels++ = *pixels * (1 - shouldDraw) + 255 * shouldDraw;
                *pixels++ = *pixels * (1 - shouldDraw) + 255 * shouldDraw;
                *pixels++ = *pixels * (1 - shouldDraw) + 255 * shouldDraw;
                *pixels++ = *pixels * (1 - shouldDraw) + 255 * shouldDraw;
            }
        }
    }
}


int main() {
    srand(42);
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("Error initializing SDL!\n");
    }

    SDL_Window *window = SDL_CreateWindow(
        "Langevin Dynamics",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        640,
        480,
        SDL_WINDOW_RESIZABLE
    );


    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);

    int width, height;
    SDL_GetWindowSize(window, &width, &height);

    SDL_Texture* texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        width,
        height
    );


    PixelBuffer pixBuff = {
        .width = width,
        .height = height,
        .bytesPerPixel = BYTES_PER_PIXEL,
        .pitch = width * BYTES_PER_PIXEL,
    };
    pixBuff.pixels = (byte*) malloc(height * pixBuff.pitch);
    

    SDL_Event event;
    int quit = 0;
    while (!quit) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    quit=1;
                    printf("Quit!\n");
                    break;
                case SDL_WINDOWEVENT:
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        width = event.window.data1;
                        height = event.window.data2;
                        texture = sdl_handle_window_resize(
                            &pixBuff, 
                            texture, 
                            renderer, 
                            width, 
                            height
                        );
                    }
                    break;
                default:;
            }
        }
        updateAndRender(&pixBuff);
        sdl_update_window(renderer, texture, &pixBuff);
    }


    SDL_Quit();
}
