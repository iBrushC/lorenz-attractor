#include <stdio.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdlib.h>
#include <math.h>

#include "../constants.h"
#include "../engine3d/engine3d.h"
#include "../simplegui/simplegui.h"

// Enums for user control
enum CAM_MODE { WALK, ORBIT };

// ------------------------------------------------------
// Structs
// ------------------------------------------------------

// Queue setup
typedef struct VectorQueue {
    Vec3 queue[MAXTRAIL];
    int count;
} VecQueue;

void queueRemove(VecQueue* q) {
    int i;
    for (i = 0; i < q->count - 1; i++) {
        q->queue[i] = q->queue[i + 1];
    }
    q->count--;
    return;
}

void queueAdd(VecQueue* q, const Vec3 element) { //fly
    if (q->count < MAXTRAIL) {
        q->queue[q->count] = element;
    } else {
        queueRemove(q);
        q->queue[MAXTRAIL - 1] = element;
    }
    q->count++;
    return;
}

// ------------------------------------------------------
// Helper functions
// ------------------------------------------------------

// stdlib sometimes defines min and max as macros, so to prevent compile problems
#ifndef min
double min(double a, double b) {
    return a < b ? a : b;
}
#endif
#ifndef max
double max(double a, double b) {
    return a > b ? a : b;
}
#endif

double clamp(double x, double a, double b) {
    return min(b, max(a, x));
}

// Point is used for x, y, and z values
// Params is used for o, p, and B values
void eulerLorenzAttractor(Vec3* point, const Vec3 params, double delta) {
    double dxdt = params.x * (point->y - point->x);
    dxdt *= delta;

    double dydt = point->x * (params.y - point->z) - point->y;
    dydt *= delta;

    double dzdt = (point->x * point->y) - (params.z * point->z);
    dzdt *= delta;

    point->x += dxdt;
    point->y += dydt;
    point->z += dzdt;
}

void rk4LorenzAttractor(Vec3* point, Vec3* velocityOut, const Vec3 params, double delta) {
    // Not the cleanest code, but it works
    double dxk1 = (params.x * (point->y - point->x));               double x1 = point->x + delta * dxk1 / 2;
    double dxk2 = (params.x * (point->y - x1));                     double x2 = point->x + delta * dxk2 / 2;
    double dxk3 = (params.x * (point->y - x2));                     double x3 = point->x + delta * dxk3;
    double dxk4 = (params.x * (point->y - x3));

    double dxdt = (1.0 / 6.0) * (dxk1 + 2 * dxk2 + 2 * dxk3 + dxk4) * delta;

    double dyk1 = (point->x * (params.y - point->z) - point->y);    double y1 = point->y + delta * dyk1 / 2;
    double dyk2 = (point->x * (params.y - point->z) - y1);          double y2 = point->y + delta * dyk2 / 2;
    double dyk3 = (point->x * (params.y - point->z) - y2);          double y3 = point->y + delta * dyk3;
    double dyk4 = (point->x * (params.y - point->z) - y3);

    double dydt = (1.0 / 6.0) * (dyk1 + 2 * dyk2 + 2 * dyk3 + dyk4) * delta;

    double dzk1 = (point->x * point->y) - (params.z * point->z);    double z1 = point->z + delta * dzk1 / 2;
    double dzk2 = (point->x * point->y) - (params.z * z1);          double z2 = point->z + delta * dzk2 / 2;
    double dzk3 = (point->x * point->y) - (params.z * z2);          double z3 = point->z + delta * dzk3;
    double dzk4 = (point->x * point->y) - (params.z * z3);

    double dzdt = (1.0 / 6.0) * (dzk1 + 2 * dzk2 + 2 * dzk3 + dzk4) * delta;

    point->x += dxdt;
    point->y += dydt;
    point->z += dzdt;

    // For visualization purposes
    velocityOut->x += dxdt;
    velocityOut->y += dydt;
    velocityOut->z += dzdt;
}

void drawLine3D(SDL_Renderer* renderer, int width, int height, const Vec3 p1, const Vec3 p2, const Mat4 transformationMatrix, const Mat4 projectionMatrix, const Plane clippingPlanes[6]) {
    // Transform to desired space
    Vec3 p1Transformed, p2Transformed;
    Mat4MultiplyVec3(&p1Transformed, transformationMatrix, p1);
    Mat4MultiplyVec3(&p2Transformed, transformationMatrix, p2);

    int runningSum = 0; // Adds one for every check to make sure the line shouldnt be culled

    // View clipping plane corrections
    runningSum += clipWithinPlane(clippingPlanes[0], &p1Transformed, &p2Transformed); // near
    runningSum += clipWithinPlane(clippingPlanes[1], &p1Transformed, &p2Transformed); // far

    // Project to screen
    Vec3 p1Projected, p2Projected;
    projectVec3ToScreen(&p1Projected, projectionMatrix, p1Transformed, width, height);
    projectVec3ToScreen(&p2Projected, projectionMatrix, p2Transformed, width, height);

    /*
    These clipping plane algorithms are by far the slowest part of the entire rendering. 
    Without them, upwards of 100,000 lines could be drawn easily.
    With them, it struggles at 10,000 lines.

    Simple solution is don't use a software renderer, more complex solution is implement
    a more sophisticated clipping algorithm instead of hacking together the 3d one to work
    within screen coordinates. I, however, will do neither of those things.
    */
    // View clipping plane corrections
    if (p1Projected.x < 0 || p1Projected.x > width || p1Projected.y < 0 || p1Projected.y > height ||
        p2Projected.x < 0 || p2Projected.x > width || p2Projected.y < 0 || p2Projected.y > height) {
            runningSum += clipWithinPlane(clippingPlanes[2], &p1Projected, &p2Projected); // left
            runningSum += clipWithinPlane(clippingPlanes[3], &p1Projected, &p2Projected); // top
            runningSum += clipWithinPlane(clippingPlanes[4], &p1Projected, &p2Projected); // right
            runningSum += clipWithinPlane(clippingPlanes[5], &p1Projected, &p2Projected); // bottom
            runningSum += 0;
        } else {
            runningSum += 4;
        }

    // Final draw
    if (runningSum == 6) {
        SDL_RenderDrawLine(renderer, p1Projected.x, p1Projected.y, p2Projected.x, p2Projected.y);
    }
}

void drawPoint3D(SDL_Renderer* renderer, int width, int height, const Vec3 point, const Mat4 transformationMatrix, const Mat4 projectionMatrix, const Plane clippingPlanes[6], int radius) {
    // Transform to desired space
    Vec3 pointTransformed;
    Mat4MultiplyVec3(&pointTransformed, transformationMatrix, point);

    int runningSum = 0; // Adds one for every check to make sure the line shouldnt be culled

    // View clipping plane corrections
    runningSum += isWithinPlane(clippingPlanes[0], pointTransformed); // near
    runningSum += isWithinPlane(clippingPlanes[1], pointTransformed); // far

    // Project to screen
    Vec3 pointProjected;
    projectVec3ToScreen(&pointProjected, projectionMatrix, pointTransformed, width, height);

    // View clipping plane corrections
    if (pointProjected.x < 0 || pointProjected.x > width || pointProjected.y < 0 || pointProjected.y > height) {
        runningSum += isWithinPlane(clippingPlanes[2], pointProjected); // left
        runningSum += isWithinPlane(clippingPlanes[3], pointProjected); // top
        runningSum += isWithinPlane(clippingPlanes[4], pointProjected); // right
        runningSum += isWithinPlane(clippingPlanes[5], pointProjected); // bottom
    } else {
        runningSum += 4;
    }

    // Final draw
    if (runningSum == 6) {
        SDL_RenderFillRect(renderer, 
        &(SDL_Rect)
        {
            pointProjected.x - radius, pointProjected.y - radius,
            radius * 2, radius * 2
        });
    }
}

// Debug, draws origin and X Y and Z axis as red green and blue lines
void drawOriginAxis(SDL_Renderer* renderer, int width, int height, const Mat4 transformationMatrix, const Mat4 projectionMatrix, const Plane clippingPlanes[6], double unitLength) {
    // Mini plane grid
    int i;
    for (i = -3; i < 4; i++) {
        Vec3 p1 = {i * unitLength, 0, 3 * unitLength};
        Vec3 p2 = {i * unitLength, 0, -3 * unitLength};
        SDL_SetRenderDrawColor(renderer, 150, 150, 150, 35);
        drawLine3D(renderer, width, height, p1, p2, transformationMatrix, projectionMatrix, clippingPlanes);
    }
    for (i = -3; i < 4; i++) {
        Vec3 p1 = {3 * unitLength, 0, i * unitLength};
        Vec3 p2 = {-3 * unitLength, 0, i * unitLength};
        SDL_SetRenderDrawColor(renderer, 150, 150, 150, 35);
        drawLine3D(renderer, width, height, p1, p2, transformationMatrix, projectionMatrix, clippingPlanes);
    }

    // Draw x axis
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 35);
    drawLine3D(renderer, width, height, (Vec3){0, 0, 0}, (Vec3){unitLength, 0, 0}, transformationMatrix, projectionMatrix, clippingPlanes);

    // Draw y axis
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 35);
    drawLine3D(renderer, width, height, (Vec3){0, 0, 0}, (Vec3){0, unitLength, 0}, transformationMatrix, projectionMatrix, clippingPlanes);

    // Draw z axis
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 35);
    drawLine3D(renderer, width, height, (Vec3){0, 0, 0}, (Vec3){0, 0, unitLength}, transformationMatrix, projectionMatrix, clippingPlanes);

    // Draw origin
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 50);
    drawPoint3D(renderer, width, height, (Vec3){0, 0, 0}, transformationMatrix, projectionMatrix, clippingPlanes, 1);
}


// Main
int main( int argc, char* argv[] ) {
    // -- SDL init --
    if (SDL_Init( SDL_INIT_EVERYTHING )) {
        printf("Initializtaion failed: %s\n", SDL_GetError());
        return 1;
    };
    if (TTF_Init()) {
        printf("Initializtaion failed: %s\n", TTF_GetError());
        return 1;
    }

    // Dynamic window settings
    char windowTitle[37] = "Lorenz System Viewer   |   FPS:    ";
    int width = WIDTH;
    int height = HEIGHT;
    int skipFrames = 0;

    // Window
    SDL_Window* window = SDL_CreateWindow(
        windowTitle, 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        width, 
        height, 
        SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE
    );
    if (!window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, 
        -1, 
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (!renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_ADD);

    SDL_Event event;

    // -- Non-SDL init --
    // Frame settings
    int active = 1;
    const int msPerFrame = 1000 / TARGETFPS;
    const int frameSumTime = 50; // the number of frames sampled to get FPS
    int msPerFrameSum = 0;
    int frameCount = 0;
    int deltaTime = msPerFrame;
    double scaledDeltaTime = (double) deltaTime / 10.0;

    // Mouse/key trackers
    int mouseX = 0; int mouseY = 0;
    int previousMouseX = 0; int previousMouseY = 0;
    int mouseDX = 0; int mouseDY = 0;
    int mouseDown = 0;
    int scrollDY = 0;

    // Local delta variable
    double localDelta = DELTA;

    // Initialize points
    int trailLength = 25;
    int pointCount = 500;
    Vec3 lorenzParams = {10, 28, 8.0/3.0};
    Vec3 points[MAXPOINTS];
    VecQueue pointQueues[MAXPOINTS];

    int i, j;
    for (i = 0; i < MAXPOINTS; i++) {
        points[i].x = (double)(rand() % 100) / 100 + 0.01;
        points[i].y = (double)(rand() % 100) / 100 + 0.01;
        points[i].z = (double)(rand() % 100) / 100 + 25.01;

        pointQueues[i].count = 0;
    }

    // Camera
    Vec3 cameraPosition = {0, 0, -35};
    Vec3 cameraRotation = {0, 0, 0}; // Its actual rotation
    Vec3 cameraPositionVelocity = {0, 0, 0};
    Vec3 cameraAngularVelocity = {0, 0, 0};

    Vec3 cameraDirection = {0, 0, 1}; // Where its pointing
    Vec3 cameraForwad = {0, 0, 1}; // Constant forward
    Vec3 cameraUp = {0, 1, 0};
    Vec3 cameraRight;
    Vec3Cross(&cameraRight, cameraUp, cameraDirection);

    int cameraMode = ORBIT;

    // Matrices
    double aspectRatio = (double) height / (double) width;
    Mat4 projectionMatrix = makeProjectionMatrix(90.0, 0.1, 100, aspectRatio);
    Mat4 worldMatrix = makeIdentityMatrix();

    // View space clipping planes
    Plane nearPlane = {
        {0, 0, 1},
        {0, 0, 1},
    };
    Plane farPlane = {
        {0, 0, 100},
        {0, 0, -1},
    };
    // Screen space clipping planes
    Plane leftPlane = {
        {0, 0, 0},
        {1, 0, 0},
    };
    Plane topPlane = {
        {0, 0, 0},
        {0, 1, 0},
    };
    Plane rightPlane = {
        {width, 0, 0},
        {-1, 0, 0},
    };
    Plane bottomPlane = {
        {0, height, 0},
        {0, -1, 0},
    };
    Plane clippingPlanes[6] = {
        nearPlane,
        farPlane,
        leftPlane,
        topPlane,
        rightPlane,
        bottomPlane,
    };

    // -- GUI setup -- (there has got to be some better way of doing this)
    TTF_Font* proggyClean = TTF_OpenFont("simplegui/ProggyClean.ttf", 24);

    SDL_Color white = {255, 255, 255, 255};
    SDL_Color buttonColor1 = {73, 79, 79, 255};
    SDL_Color buttonColor2 = {38, 117, 117, 255};
    SDL_Color sliderColor1 = {50, 50, 50, 255};
    SDL_Color sliderColor2 = {35, 35, 35, 155};

    // Watermark
    Text watermarkText;
    initializeText(renderer, proggyClean, white, "Lorenz Attractor Simulation by Andrew Combs 2022", &watermarkText);
    SDL_Rect watermarkTextRect = {0, height - 16, 49 * 10, 16};

    // Main panel
    SDL_Rect panelHeader = {width - 150, 0, 150, 18};
    SDL_Rect panelBody = {width - 150, 18, 150, 200};
    Text panelHeaderText;
    initializeText(renderer, proggyClean, white, "Settings", &panelHeaderText);
    SDL_Rect panelHeaderTextRect = {width - 115, 2, 9 * 10, 16};

    // Reset particles button
    Button resetParticlesButton;
    SDL_Rect resetParticlesButtonRect = {width - 139, 190, 130, 18};
    Text resetParticlesButtonText;
    initializeText(renderer, proggyClean, white, "Reset Particles", &resetParticlesButtonText);
    SDL_Rect resetParticlesButtonTextRect = {width - 130, 192, 16 * 7, 15};
    initializeButton(&resetParticlesButtonText, &resetParticlesButtonRect, buttonColor1, buttonColor2, &resetParticlesButton);

    // Reset camera button
    Button resetCameraButton;
    SDL_Rect resetCameraButtonRect = {width - 139, 164, 130, 18};
    Text resetCameraButtonText;
    initializeText(renderer, proggyClean, white, "Reset Camera", &resetCameraButtonText);
    SDL_Rect resetCameraButtonTextRect = {width - 130, 166, 13 * 7, 15};
    initializeButton(&resetCameraButtonText, &resetCameraButtonRect, buttonColor1, buttonColor2, &resetCameraButton);

    // Render tip button
    Button renderTipButton;
    SDL_Rect renderTipButtonRect = {width - 139, 138, 18, 18};
    Text renderTipButtonText;
    initializeText(renderer, proggyClean, white, "Tip", &renderTipButtonText);
    SDL_Rect renderTipButtonTextRect = {width - 115, 141, 3 * 7, 15};
    initializeButton(&renderTipButtonText, &renderTipButtonRect, buttonColor1, buttonColor2, &renderTipButton);
    int usingRenderTip = 1;

    // Render trail button
    Button renderTrailButton;
    SDL_Rect renderTrailButtonRect = {width - 80, 138, 18, 18};
    Text renderTrailButtonText;
    initializeText(renderer, proggyClean, white, "Trail", &renderTrailButtonText);
    SDL_Rect renderTrailButtonTextRect = {width - 55, 141, 5 * 7, 15};
    initializeButton(&renderTrailButtonText, &renderTrailButtonRect, buttonColor1, buttonColor2, &renderTrailButton);
    int usingRenderTrail = 1;

    // Show origin button
    Button showOriginButton;
    SDL_Rect showOriginButtonRect = {width - 139, 112, 18, 18};
    Text showOriginButtonText;
    initializeText(renderer, proggyClean, white, "Origin", &showOriginButtonText);
    SDL_Rect showOriginButtonTextRect = {width - 115, 115, 7 * 7, 15};
    initializeButton(&showOriginButtonText, &showOriginButtonRect, buttonColor1, buttonColor2, &showOriginButton);
    int usingShowOrigin = 1;

    // Show velocities button
    Button showVelocityButton;
    SDL_Rect showVelocityButtonRect = {width - 55, 112, 18, 18};
    Text showVelocityButtonText;
    initializeText(renderer, proggyClean, white, "Vel", &showVelocityButtonText);
    SDL_Rect showVelocityButtonTextRect = {width - 30, 115, 4 * 7, 15};
    initializeButton(&showVelocityButtonText, &showVelocityButtonRect, buttonColor1, buttonColor2, &showVelocityButton);
    int usingShowVelocity = 0;

    // Delta slider
    SDL_Rect deltaSliderTrack = {width - 139, 92, 130, 12};
    SDL_Rect deltaSliderBar = {width - 80, 92, 12, 12};
    Text deltaSliderText;
    initializeText(renderer, proggyClean, white, "Delta", &deltaSliderText);
    SDL_Rect deltaSliderTextRect = {width - 139, 79, 5 * 7, 15};
    int deltaSliderActive = 0;
    double deltaSliderValue = 59;
    Slider deltaSlider = {
        &deltaSliderBar,
        &deltaSliderTrack,
        sliderColor1,
        sliderColor2,
    };

    // Points slider
    SDL_Rect pointsSliderTrack = {width - 139, 66, 130, 12};
    SDL_Rect pointsSliderBar = {width - 57, 66, 12, 12};
    Text pointsSliderText;
    initializeText(renderer, proggyClean, white, "Points", &pointsSliderText);
    SDL_Rect pointsSliderTextRect = {width - 139, 52, 7 * 7, 15};
    int pointsSliderActive = 0;
    double pointsSliderValue = 39;
    Slider pointsSlider = {
        &pointsSliderBar,
        &pointsSliderTrack,
        sliderColor1,
        sliderColor2,
    };

    // Trail slider
    SDL_Rect trailsSliderTrack = {width - 139, 40, 130, 12};
    SDL_Rect trailsSliderBar = {width - 80, 40, 12, 12};
    Text trailsSliderText;
    initializeText(renderer, proggyClean, white, "Trails", &trailsSliderText);
    SDL_Rect trailsSliderTextRect = {width - 139, 26, 7 * 7, 15};
    int trailsSliderActive = 0;
    double trailsSliderValue = 59;
    Slider trailsSlider = {
        &trailsSliderBar,
        &trailsSliderTrack,
        sliderColor1,
        sliderColor2,
    };

    // Main loop
    while (active) {
        // Frame updates
        frameCount++;
        int tickStart = SDL_GetTicks();
        if (deltaTime >= 1000) {
            scaledDeltaTime = 0;
        }

        // -- Event handling -- 
        // Event poll
        while (SDL_PollEvent(&event)) {
            // Exit
            if (event.type == SDL_QUIT) {
                active = 0;
                break; 
            }

            // -- General event handling --
            // Keyboard
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    // WASD-QE movement
                    case SDLK_w:
                        cameraPositionVelocity = cameraDirection;
                        break;
                    case SDLK_a:
                        cameraPositionVelocity = cameraRight;
                        Vec3Negative(&cameraPositionVelocity);
                        break;
                    case SDLK_s:
                        cameraPositionVelocity = cameraDirection;
                        Vec3Negative(&cameraPositionVelocity);
                        break;
                    case SDLK_d:
                        cameraPositionVelocity = cameraRight;
                        break;
                    case SDLK_q:
                        cameraPositionVelocity = cameraUp;
                        break;
                    case SDLK_e:
                        cameraPositionVelocity = cameraUp;
                        Vec3Negative(&cameraPositionVelocity);
                        break;

                    // Reset
                    case SDLK_r:
                        // todo
                        break;

                    // Arrow key looking
                    case SDLK_LEFT:
                        cameraAngularVelocity.y = 0.05;
                        break;

                    case SDLK_RIGHT:
                        cameraAngularVelocity.y = -0.05;
                        break;

                    case SDLK_UP:
                        cameraAngularVelocity.x = 0.05;
                        break;

                    case SDLK_DOWN:
                        cameraAngularVelocity.x = -0.05;
                        break;
                    
                    default:
                        break;
                }
            } else if (event.type == SDL_KEYUP) {
                switch (event.key.keysym.sym) {
                    // WASD-QE movement
                    case SDLK_w:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;
                    case SDLK_a:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;
                    case SDLK_s:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;
                    case SDLK_d:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;
                    case SDLK_q:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;
                    case SDLK_e:
                        cameraPositionVelocity = (Vec3){0, 0, 0};
                        break;

                    // Arrow key looking
                    case SDLK_LEFT:
                        cameraAngularVelocity.y = 0;
                        break;

                    case SDLK_RIGHT:
                        cameraAngularVelocity.y = 0;
                        break;

                    case SDLK_UP:
                        cameraAngularVelocity.x = 0;
                        break;

                    case SDLK_DOWN:
                        cameraAngularVelocity.x = 0;
                        break;
                    
                    default:
                        break;
                }
            // Mouse
            } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                switch (event.button.button) {
                    case (SDL_BUTTON_LEFT): {
                        mouseDown = 1;
                        // Buttons
                        if (isMouseOverRect(resetParticlesButton.rect, mouseX, mouseY)) {
                            int i;
                            for (i = 0; i < MAXPOINTS; i++) {
                                points[i].x = (double)(rand() % 100) / 100 + 0.01;
                                points[i].y = (double)(rand() % 100) / 100 + 0.01;
                                points[i].z = (double)(rand() % 100) / 100 + 25.01;

                                pointQueues[i].count = 1;
                            }
                        }
                        if (isMouseOverRect(resetCameraButton.rect, mouseX, mouseY)) {
                            cameraRotation.x = 0;
                            cameraRotation.y = 0;
                            cameraRotation.z = 0;

                            cameraPosition.x = 0;
                            cameraPosition.y = 0;
                            cameraPosition.z = -35;
                        }
                        if (isMouseOverRect(renderTipButton.rect, mouseX, mouseY)) {
                            usingRenderTip = !usingRenderTip;
                        }
                        if (isMouseOverRect(renderTrailButton.rect, mouseX, mouseY)) {
                            usingRenderTrail = !usingRenderTrail;
                        }
                        if (isMouseOverRect(showOriginButton.rect, mouseX, mouseY)) {
                            usingShowOrigin = !usingShowOrigin;
                        }
                        if (isMouseOverRect(showVelocityButton.rect, mouseX, mouseY)) {
                            usingShowVelocity = !usingShowVelocity;
                        }
                        if (isMouseOverRect(deltaSlider.track, mouseX, mouseY)) {
                            deltaSliderActive = 1;
                        }
                        if (isMouseOverRect(pointsSlider.track, mouseX, mouseY)) {
                            pointsSliderActive = 1;
                        }
                        if (isMouseOverRect(trailsSlider.track, mouseX, mouseY)) {
                            trailsSliderActive = 1;
                        }
                        break;
                    }
                    default:
                        break;
                }
            } else if (event.type == SDL_MOUSEBUTTONUP) {
                switch (event.button.button) {
                    case (SDL_BUTTON_LEFT):
                        mouseDown = 0;
                        deltaSliderActive = 0;
                        pointsSliderActive = 0;
                        trailsSliderActive = 0;
                        break;
                    default:
                        break;
                }
            } else if (event.type == SDL_MOUSEMOTION) {
                mouseX = event.button.x;
                mouseY = event.button.y;
                mouseDX = previousMouseX - mouseX;
                mouseDY = previousMouseY - mouseY;
                previousMouseX = mouseX;
                previousMouseY = mouseY;

                if (deltaSliderActive) {
                    deltaSliderValue = min(118, max(0, mouseX - (width - 139)));
                    deltaSliderBar.x = (width - 139) + deltaSliderValue;
                    localDelta = 0.01 + (deltaSliderValue / 118.0) * 0.1;
                }
                if (pointsSliderActive) {
                    pointsSliderValue = min(118, max(0, mouseX - (width - 139)));
                    pointsSliderBar.x = (width - 139) + pointsSliderValue;
                    pointCount = floor(pow(pointsSliderValue / 118.0, 3)) * MAXPOINTS;
                }
                if (trailsSliderActive) {
                    trailsSliderValue = min(118, max(0, mouseX - (width - 139)));
                    trailsSliderBar.x = (width - 139) + trailsSliderValue;
                    trailLength = floor(trailsSliderValue / 118.0) * MAXTRAIL;
                }


            } else if (event.type == SDL_MOUSEWHEEL) {
                scrollDY = event.wheel.y;
            // Window stuff
            } else if (event.type == SDL_WINDOWEVENT) {
                switch (event.window.event) {
                    case (SDL_WINDOWEVENT_SIZE_CHANGED):
                        skipFrames = 2;

                        // Rebuilding projection matrix and clipping planes
                        width = (int) event.window.data1;
                        height = (int) event.window.data2;
                        double aspectRatio = (double) height / (double) width;
                        projectionMatrix = makeProjectionMatrix(90.0, 0.1, 100, aspectRatio);
                        rightPlane.position.x = width;
                        clippingPlanes[4].position.x = width;
                        clippingPlanes[5].position.y = height;

                        // Updating gui placements (legitimately awful)
                        panelHeader.x = width - 150;
                        panelBody.x = width - 150;
                        watermarkTextRect.y = height - 16;
                        panelHeaderTextRect.x = width - 115;
                        resetParticlesButtonRect.x = width - 139;
                        resetParticlesButtonTextRect.x = width - 130;
                        resetCameraButtonRect.x = width - 139;
                        resetCameraButtonTextRect.x = width - 130;
                        renderTipButtonRect.x = width - 139;
                        renderTipButtonTextRect.x = width - 115;
                        renderTrailButtonRect.x = width - 80;
                        renderTrailButtonTextRect.x = width - 55;
                        showOriginButtonRect.x = width - 139;
                        showOriginButtonTextRect.x = width - 115;
                        showVelocityButtonRect.x = width - 55;
                        showVelocityButtonTextRect.x = width - 30;
                        deltaSliderTrack.x = width - 139;
                        deltaSliderTextRect.x = width - 139;
                        pointsSliderTrack.x = width - 139;
                        pointsSliderTextRect.x = width - 139;
                        trailsSliderTrack.x = width - 139;
                        trailsSliderTextRect.x = width - 139;

                        // Slider positioning
                        deltaSliderBar.x = (width - 139) + deltaSliderValue;
                        pointsSliderBar.x = (width - 139) + pointsSliderValue;
                        trailsSliderBar.x = (width - 139) + trailsSliderValue;

                        break;
                    case (SDL_WINDOWEVENT_MOVED):
                        skipFrames = 2;
                    default:
                        break;
                }
            }
        }

        // Janky calculation skipping to prevent explosions when resizing (delta time scaling)
        if (skipFrames > 0) {
            skipFrames--;
            continue;
        }

        // Non-particle dynamics
        Mat4 cameraMatrix;

        // Walk mode, FPS-like
        if (cameraMode == WALK) { 
            Vec3Add(&cameraPosition, cameraPosition, cameraPositionVelocity);
            Vec3Add(&cameraRotation, cameraRotation, cameraAngularVelocity);

            Mat4 cameraXRotationMatrix = makeXRotationMatrix(cameraRotation.x);
            Mat4MultiplyVec3(&cameraDirection, cameraXRotationMatrix, cameraForwad);
            Vec3Cross(&cameraUp, cameraDirection, cameraRight);

            Mat4 cameraYRotationMatrix = makeYRotationMatrix(cameraRotation.y);
            Mat4MultiplyVec3(&cameraDirection, cameraYRotationMatrix, cameraDirection);
            Vec3Cross(&cameraRight, cameraUp, cameraDirection);

            Vec3 target;
            Vec3Add(&target, cameraPosition, cameraDirection);
            cameraMatrix = makePointAtMatrix(cameraPosition, target, cameraUp);  // Maybe make constants? 

        // Orbit mode, much more intuitive, rotate with mouse
        } else if (!(deltaSliderActive || pointsSliderActive || trailsSliderActive)) { 
            cameraPosition.z += (double) scrollDY * scaledDeltaTime;
            cameraPosition.z = clamp(cameraPosition.z, -75, -1);
            scrollDY = 0;

            Vec3 cameraNewPosition = cameraPosition;
            
            if (mouseDown) {
                cameraRotation.x += ((double) mouseDX / 1000) * scaledDeltaTime;
                cameraRotation.y += ((double) mouseDY / 1000) * scaledDeltaTime;
                cameraRotation.y = clamp(cameraRotation.y, -1.55, 1.55);
            }
            mouseDX = 0;
            mouseDY = 0;

            Mat4 cameraXRotationMatrix = makeXRotationMatrix(cameraRotation.y);
            Mat4MultiplyVec3(&cameraNewPosition, cameraXRotationMatrix, cameraNewPosition);

            Mat4 cameraYRotationMatrix = makeYRotationMatrix(cameraRotation.x);
            Mat4MultiplyVec3(&cameraNewPosition, cameraYRotationMatrix, cameraNewPosition);

            cameraMatrix = makePointAtMatrix(cameraNewPosition, (Vec3){0, 0, 0}, cameraUp);
        }
        
        Mat4 viewMatrix = quickMatrixInverse(cameraMatrix);

        // Any transformatiosn that should be applied to the particles
        Mat4 transformationMatrix, worldToViewMatrix, objectToViewMatrix;
        Mat4 rotationMatrix = makeYRotationMatrix(0);
        Mat4 translationMatrix = makeTranslationMatrix((Vec3){0, 0, -25});
        Mat4 scalingMatrix = makeScalingMatrix((Vec3){0.7, 0.7, 0.7});
        Mat4MultiplyMat4(&transformationMatrix, rotationMatrix, translationMatrix);
        Mat4MultiplyMat4(&transformationMatrix, scalingMatrix, transformationMatrix);
        
        Mat4MultiplyMat4(&worldToViewMatrix, viewMatrix, worldMatrix); // Transforms from world to view
        Mat4MultiplyMat4(&objectToViewMatrix, worldToViewMatrix, transformationMatrix); // Transforms from any particle transformations to viewspace

        // Main draw cycle
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Particle Handling
        for (i = 0; i < pointCount; i++) {
            // Apply the attractor
            Vec3 color;
            Vec3 velocity = {0, 0, 0};
            for (j = 0; j < STEPS; j++) {
                rk4LorenzAttractor(&points[i], &velocity, lorenzParams, (localDelta / STEPS) * (scaledDeltaTime / 10));
                // eulerLorenzAttractor(&points[i], lorenzParams, (localDelta / STEPS) * (scaledDeltaTime / 10));
            }
            // printf("{ %5lf, %5lf, %5lf }\n", velocity.x, velocity.y, velocity.z);
            if (usingShowVelocity) {
                color.x = ((velocity.x + 4) * 32);
                color.y = ((velocity.y + 4) * 32);
                color.z = ((velocity.z + 4) * 32);
            } else {
                color.x = 255;
                color.y = 255;
                color.z = 255;
            }
            

            // Drawing the trails
            int trueTrailLength = min(pointQueues[i].count, trailLength); // Bad naming, but is the actual length of the trail (to account for when there are less particles than trail length)
            int offset = max(0, pointQueues[i].count - trailLength);
            if (usingRenderTrail && trueTrailLength) {
                for (j = 0; j < trueTrailLength - 1; j++) {
                    Vec3 p1 = pointQueues[i].queue[offset + j];
                    Vec3 p2 = pointQueues[i].queue[offset + j + 1];

                    int opacity = powf((float)j / trueTrailLength, 5) * 255;
                    SDL_SetRenderDrawColor(renderer, 
                        color.x, 
                        color.y, 
                        color.z, 
                        opacity
                        );

                    drawLine3D(renderer, width, height, p1, p2, objectToViewMatrix, projectionMatrix, clippingPlanes);
                }
                drawLine3D(renderer, width, height, pointQueues[i].queue[offset + trueTrailLength - 1], points[i], objectToViewMatrix, projectionMatrix, clippingPlanes); // Final line to connect last point in trail with current
            }
            if (usingRenderTip) {
                // Tip rendering
                SDL_SetRenderDrawColor(renderer, 
                    color.x, 
                    color.y, 
                    color.z,
                    255
                );
                drawPoint3D(renderer, width, height, points[i], objectToViewMatrix, projectionMatrix, clippingPlanes, 1);
            }
            

            // Add new position to queue
            queueAdd(&pointQueues[i], points[i]);
        }
        
        // Origin
        if (usingShowOrigin) {
            drawOriginAxis(renderer, width, height, worldToViewMatrix, projectionMatrix, clippingPlanes, 5);
        }

        // Static GUI
        renderText(renderer, &watermarkTextRect, &watermarkText);
        renderText(renderer, &panelHeaderTextRect, &panelHeaderText);
        SDL_SetRenderDrawColor(renderer, 15, 15, 15, 150); SDL_RenderFillRect(renderer, &panelBody);
        SDL_SetRenderDrawColor(renderer, 25, 25, 25, 150); SDL_RenderFillRect(renderer, &panelHeader);
        
        // Dynamic GUI
        int buttonDown;
        buttonDown = mouseDown * isMouseOverRect(resetParticlesButton.rect, mouseX, mouseY);
        renderButton(renderer, &resetParticlesButtonTextRect, &resetParticlesButton, buttonDown);
        buttonDown = mouseDown * isMouseOverRect(resetCameraButton.rect, mouseX, mouseY);
        renderButton(renderer, &resetCameraButtonTextRect, &resetCameraButton, buttonDown);
        renderButton(renderer, &renderTipButtonTextRect, &renderTipButton, usingRenderTip);
        renderButton(renderer, &renderTrailButtonTextRect, &renderTrailButton, usingRenderTrail);
        renderButton(renderer, &showOriginButtonTextRect, &showOriginButton, usingShowOrigin);
        renderButton(renderer, &showVelocityButtonTextRect, &showVelocityButton, usingShowVelocity);
        
        renderText(renderer, &deltaSliderTextRect, &deltaSliderText);
        renderSlider(renderer, &deltaSlider);
        renderText(renderer, &pointsSliderTextRect, &pointsSliderText);
        renderSlider(renderer, &pointsSlider);
        renderText(renderer, &trailsSliderTextRect, &trailsSliderText);
        renderSlider(renderer, &trailsSlider);

        SDL_RenderPresent(renderer);

        // Delta time correction
        deltaTime = SDL_GetTicks() - tickStart;

        // Delta time correction
        if (deltaTime < msPerFrame) {
            SDL_Delay(msPerFrame - deltaTime);
        }

        // Setting window title to include fps
        int newDeltaTime = SDL_GetTicks() - tickStart;
        msPerFrameSum += newDeltaTime;
        if (frameCount >= frameSumTime) {
            double framesPerSecond = 1.0 / (double)(msPerFrameSum / (1000.0 * frameSumTime));
            msPerFrameSum = 0;
            frameCount = 0;
            if (framesPerSecond > 1) {
                // String number joining hell
                double fractionalPart, integerPart;
                fractionalPart = modf(framesPerSecond, &integerPart);
                windowTitle[33] = '0' + (int) integerPart / 10;
                windowTitle[34] = '0' + (int) integerPart % 10;
                windowTitle[35] = '.';
                windowTitle[36] = '0' + (int) (fractionalPart * 10);
                SDL_SetWindowTitle(window, windowTitle);
            }
        }
        scaledDeltaTime = (double) newDeltaTime / 10.0;
    
    }

    destroyText(&watermarkText);
    destroyText(&resetParticlesButtonText);
    destroyText(&resetCameraButtonText);
    destroyText(&renderTipButtonText);
    destroyText(&renderTrailButtonText);
    destroyText(&deltaSliderText);
    destroyText(&pointsSliderText);
    destroyText(&trailsSliderText);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}