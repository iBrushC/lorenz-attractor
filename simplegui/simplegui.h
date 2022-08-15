#ifndef LORENZ_SIMPLE_GUI_H
#define LORENZ_SIMPLE_GUI_H

#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL.h>

typedef struct Text {
    SDL_Surface* surface;
    SDL_Texture* texture;
    const char* text;
} Text;

typedef struct Button {
    Text* text;
    SDL_Rect* rect;
    SDL_Color normalColor;
    SDL_Color activeColor;
} Button;

typedef struct Slider {
    SDL_Rect* bar;
    SDL_Rect* track;
    SDL_Color barColor;
    SDL_Color trackColor;
} Slider;

int isMouseOverRect(SDL_Rect* rect, int mouseX, int mouseY);

// Text
void initializeText(SDL_Renderer* renderer, TTF_Font* font, SDL_Color color, const char* contents, Text* text);
void destroyText(Text* text);
void renderText(SDL_Renderer* renderer, SDL_Rect* rect, Text* text);

// Button
void initializeButton(Text* text, SDL_Rect* rect, SDL_Color normalColor, SDL_Color activeColor, Button* button);
void renderButton(SDL_Renderer* renderer, SDL_Rect* textRect, Button* button, int active);

// Slider
void renderSlider(SDL_Renderer* renderer, Slider* slider);

#endif