#include <stdio.h>
#include <SDL2/SDL.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define BYTES_PER_PIXEL 4

// Number of attracting points in the energy function (low energy/high probability)
#define NUM_PTS 4
// What the value of the step size level (from which the step size is derived) should start as
#define START_STEP_SIZE_LEVEL -26
#define STEP_SIZE_CHANGE_MODIFIER 0.25
// Radius of pointer showing current position
#define POINTER_RADIUS 10

#define MOVE_SPEED 0.3
#define NUM_BUTTONS 6

// Crash on expr false
#define Assert(expr) if (!(expr)) {*(int*)0=0;}

#define Kilobytes(value) ((value)*((u64)1024))
#define Megabytes(value) (Kilobytes(value)*((u64)1024))
#define Gigabytes(value) (Megabytes(value)*((u64)1024))


typedef uint64_t u64;
typedef unsigned char byte;

typedef struct PixelBuffer {
    int width;
    int height;
    int bytesPerPixel;
    int pitch;
    byte* pixels;
} PixelBuffer;

typedef struct Input {
    union {
        int buttons[NUM_BUTTONS];
        struct {
            int moveL;
            int moveR;
            int moveU;
            int moveD;
            int incStep;
            int decStep;
        };
    };
    int setX;
    int setY;
    int doSet;
} Input;

typedef struct {
    float x;
    float y;
} Vec2;



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


typedef struct SimState {
    Vec2 curr;
    int stepSizeLevel;
    float xOffset, yOffset;
    float maxEnergy;

} SimState;



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


Vec2 randomNormal(float scale) {
    float u1 = ((float) rand()) / RAND_MAX;
    float u2 = ((float) rand()) / RAND_MAX;
    double r = sqrt(-2.0 * log(u1));
    double theta = 2.0 * M_PI * u2;

    Vec2 res = {(float) scale * r * cos(theta), (float) scale * r * sin(theta)};
    return res;
}


/*
 *  Apply one step of langevin sampling to a point at pos.
 *
 *  NOTE: The energy function is actually inverted so we are doing gradient ascent
 *        This is because I wanted a couple low energy islands for the chain to hop
 *        around and this was the first/easiest I thought of setting that up.
 */
void langevinStep(Vec2* pos, float stepSize) {
    // More stable than sqrt(2 * stepSize)
    float randSize = sqrt(stepSize);
    Vec2 grad = gradEnergyFunction(pos->x, pos->y);
    Vec2 jitter = randomNormal(randSize);
    pos->x += stepSize * grad.x + jitter.x;
    pos->y += stepSize * grad.y + jitter.y;

}


int max(int a, int b) {
    if (a < b) {
        return b;
    } else {
        return a;
    }
}

int min(int a, int b) {
    if (a > b) {
        return b;
    } else {
        return a;
    }
}


void drawCircle(PixelBuffer* pixBuff, int rad, int xPos, int yPos) {
    int startX = max(0, xPos - rad);
    int startY = max(0, yPos - rad);

    int endX = min(pixBuff->width-1, xPos + rad);
    int endY = min(pixBuff->height-1, yPos + rad);

    byte* pixels;

    int radSq = rad * rad;

    for (int y=startY; y < endY; y++) {
        pixels = pixBuff->pixels + y * pixBuff->pitch + startX * pixBuff->bytesPerPixel;
        for (int x=startX; x < endX; x++) {
            int rx = x - xPos;
            int ry = y - yPos;
            if (rx*rx + ry*ry <= radSq) {
                *pixels++ = 0xff;
                *pixels++ = 0xff;
                *pixels++ = 0xff;
                *pixels++ = 0xff;
            } else {
                pixels += 4;
            }
        }
    }
}


void copyPixels(PixelBuffer* dest, PixelBuffer* src, int xPos, int yPos) {
    byte* srcPixels = src->pixels;

    if (xPos < 0) {
        xPos = 0;
    }
    if (yPos < 0) {
        yPos = 0;
    }

    for (int y=yPos; y < dest->height && y - yPos < src->height; y++) {
        byte* destPixels = dest->pixels + y * dest->pitch + xPos * dest->bytesPerPixel;
        srcPixels = src->pixels + (y-yPos) * src->pitch;
        for (int x=xPos; x < dest->width && x - xPos < src->width; x++) {
            if (*(srcPixels+3) > 0x00) {
                *destPixels++ = *srcPixels++;
                *destPixels++ = *srcPixels++;
                *destPixels++ = *srcPixels++;
                *destPixels++ = *srcPixels++;
            } else {
                destPixels+=4;
                srcPixels+=4;
            }
        }
    }
}




void updateAndRender(PixelBuffer* pixBuff, Input* inp, SimState* state) {
    // Current position/state of langevin markov chain, start at 0
    if (inp->doSet) {
        state->curr.x = (float) inp->setX / pixBuff->width * 2.0 - 1 + state->xOffset;
        state->curr.y = (float) inp->setY / pixBuff->height * 2.0 - 1 + state->yOffset;
    }


    state->stepSizeLevel += inp->incStep - inp->decStep;


    float stepSize = exp(state->stepSizeLevel * STEP_SIZE_CHANGE_MODIFIER);


    state->xOffset += MOVE_SPEED * (inp->moveR - inp->moveL);
    state->yOffset += MOVE_SPEED * (inp->moveD - inp->moveU);
    

    // Update current position/state with a langevin step
    langevinStep(&state->curr, stepSize);

    // Draw energy surface, only really needs to change when the surface changes 
    // (which is currently never) but this works for now.
    byte* pixels = pixBuff->pixels;
    for (int y=0; y < pixBuff->height; y++) {
        for (int x=0; x < pixBuff->width; x++) {
            float rx = 2.0 * x / (float) pixBuff->width - 1 + state->xOffset;
            float ry = 2.0 * y / (float) pixBuff->height - 1 + state->yOffset;
            float energy = energyFunction(rx, ry);
            if (energy > state->maxEnergy) {
                state->maxEnergy = energy;
            }
            // Ensure energy < 1 for easier coloring. Will give one nonesense frame
            // at start but that doesn't matter rn
            energy = energy / state->maxEnergy;

            // Energy truncated to highlight contours
            float energyTrunc = (((byte) (energy * 255)) & 0xf8) / 255.0;

            *pixels++ = (byte) (energyTrunc * 0xe0);
            *pixels++ = (byte) (energyTrunc * 0x98);
            *pixels++ = (byte) (energyTrunc * 0xc6);
            *pixels++ = 0xff;
        }
    }
    

    // Draw a marker at the current position of the langevin chain
    int currX = ((int) (pixBuff->width * (state->curr.x - state->xOffset + 1) / 2));
    int currY = ((int) (pixBuff->height * (state->curr.y - state->yOffset + 1) / 2));
    drawCircle(pixBuff, POINTER_RADIUS, currX, currY);

}


void clearInput(Input* inp) {
    for (int i=0; i < NUM_BUTTONS; i++) {
        inp->buttons[i] = 0;
    }
    inp->doSet = 0;
}


void initState(SimState* state) {
    state->curr = (Vec2) {0.0, 0.0};
    state->stepSizeLevel = START_STEP_SIZE_LEVEL;
    state->xOffset = 0.0;
    state->yOffset = 0.0;
    state->maxEnergy = 1.0;
}


void handleKey(Input* inp, int sym, int isDown) {
    switch (sym) {
        case SDLK_LEFT:
            inp->moveL = isDown;
            break;
        case SDLK_RIGHT:
            inp->moveR = isDown;
            break;
        case SDLK_UP:
            inp->moveU = isDown;
            break;
        case SDLK_DOWN:
            inp->moveD = isDown;
            break;
        case SDLK_z:
            inp->decStep = isDown;
            break;
        case SDLK_x:
            inp->incStep = isDown;
            break;
    }
}


int main() {
    // Initialize random
    srand(time(0));

    // SDL init
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


    SimState state;
    initState(&state);

    
    Input inp;
    clearInput(&inp);
    

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
                case SDL_KEYDOWN:
                    handleKey(&inp, event.key.keysym.sym, 1);
                    break;
                case SDL_KEYUP:
                    handleKey(&inp, event.key.keysym.sym, 0);
                    break;

                case SDL_MOUSEBUTTONDOWN:
                    inp.setX = ((SDL_MouseButtonEvent*) &event)->x;
                    inp.setY = ((SDL_MouseButtonEvent*) &event)->y;
                    inp.doSet = 1;
                    break;

                case SDL_MOUSEBUTTONUP:
                    inp.doSet = 0;
                    break;
                default:;
            }
        }
        updateAndRender(&pixBuff, &inp, &state);
        sdl_update_window(renderer, texture, &pixBuff);
    }


    SDL_Quit();
}
