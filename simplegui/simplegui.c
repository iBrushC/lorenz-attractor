#include "simplegui.h"

void initializeText(SDL_Renderer* renderer, TTF_Font* font, SDL_Color color, const char* contents, Text* text) {
    SDL_Surface* textSurface = TTF_RenderText_Blended(font, contents, color);
    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    text->surface = textSurface;
    text->texture = textTexture;
    text->text = contents;
}

void destroyText(Text* text) {
    SDL_FreeSurface(text->surface);
    SDL_DestroyTexture(text->texture);
}

void renderText(SDL_Renderer* renderer, SDL_Rect* rect, Text* text) {
    SDL_RenderCopy(renderer, text->texture, NULL, rect);
}

void initializeButton(Text* text, SDL_Rect* rect, SDL_Color normalColor, SDL_Color activeColor, Button* button) {
    button->text = text;
    button->rect = rect;
    button->normalColor = normalColor;
    button->activeColor = activeColor;
}

int isMouseOverRect(SDL_Rect* rect, int mouseX, int mouseY) {
    int overX = (mouseX > rect->x) && (mouseX < rect->x + rect->w);
    int overY = (mouseY > rect->y) && (mouseY < rect->y + rect->h);
    return overX && overY;
}

void renderButton(SDL_Renderer* renderer, SDL_Rect* textRect, Button* button, int active) {
    if (active) {
        SDL_SetRenderDrawColor(renderer, button->activeColor.r, button->activeColor.g, button->activeColor.b, button->activeColor.a);
    } else {
        SDL_SetRenderDrawColor(renderer, button->normalColor.r, button->normalColor.g, button->normalColor.b, button->normalColor.a);
    }
    SDL_RenderFillRect(renderer, button->rect);
    renderText(renderer, textRect, button->text);
}

void renderSlider(SDL_Renderer* renderer, Slider* slider) {
    SDL_SetRenderDrawColor(renderer, slider->barColor.r, slider->barColor.g, slider->barColor.b, slider->barColor.a);
    SDL_RenderFillRect(renderer, slider->bar);
    SDL_SetRenderDrawColor(renderer, slider->trackColor.r, slider->trackColor.g, slider->trackColor.b, slider->trackColor.a);
    SDL_RenderFillRect(renderer, slider->track);
}