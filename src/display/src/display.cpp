/*
 * display.cpp
 * Created on: Feb 14, 2022 21:40
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "display/display.hpp"

namespace pllee4 {
Display::Display()
    : screen_width_(0),
      screen_height_(0),
      window_(nullptr),
      renderer_(nullptr),
      main_front_(nullptr) {}

Display::~Display() { DestroyRenderer(); }

bool Display::CreateRenderer(std::string title, int screen_width,
                             int screen_height) {
  DestroyRenderer();

  screen_width_ = screen_width;
  screen_height_ = screen_height;

  view_width_ = screen_width_;
  view_height_ = screen_height_;
  view_x_offset_ = 0.0;
  view_y_offset_ = 0.0;

  // Create window
  window_ =
      SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, screen_width, screen_height, 0);
  if (window_ == nullptr) {
    std::cout << "Window could not be created! SDL_Error: " << SDL_GetError()
              << std::endl;
    DestroyRenderer();
    return false;
  }

  // Create Renderer
  renderer_ = SDL_CreateRenderer(
      window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer_ == nullptr) {
    std::cout << "Renderer could not be created! SDL Error: " << SDL_GetError()
              << std::endl;
    DestroyRenderer();
    return false;
  }

  // Create Font
  std::string font_path{ASSET_PATH};
  font_path += "/font/Roboto-Regular.ttf";

  main_front_ = TTF_OpenFont(font_path.c_str(), 18);
  if (main_front_ == nullptr) {
    std::cout << "Failed to load font! SDL_ttf Error: " << TTF_GetError()
              << std::endl;
    DestroyRenderer();
    return false;
  }

  return true;
}

void Display::DestroyRenderer() {
  // Destroy Renderer
  if (renderer_ != nullptr) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }

  // Destroy Window
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }

  // Destroy Font
  if (main_front_ != nullptr) {
    TTF_CloseFont(main_front_);
    main_front_ = nullptr;
  }
}

void Display::ClearScreen() {
  if (renderer_ != nullptr) {
    // Clear screen
    SDL_SetRenderDrawColor(renderer_, 0x00, 0x00, 0x00, 0xFF);
    SDL_RenderClear(renderer_);
  }
}

void Display::ShowScreen() {
  if (renderer_ != nullptr) {
    SDL_RenderPresent(renderer_);
  }
}

void Display::SetView(double width, double height, double x_offset,
                      double y_offset) {
  view_width_ = fabs(width);
  view_height_ = fabs(height);
  view_x_offset_ = x_offset - view_height_ / 2.0;
  view_y_offset_ = y_offset - view_width_ / 2.0;
}

void Display::SetView(double x_offset, double y_offset) {
  view_x_offset_ = x_offset - view_height_ / 2.0;
  view_y_offset_ = y_offset - view_width_ / 2.0;
}

void Display::SetDrawColour(uint8_t red, uint8_t green, uint8_t blue,
                            uint8_t alpha) {
  SDL_SetRenderDrawColor(renderer_, red, green, blue, alpha);
}

void Display::DrawLine(const Vector2& start_pos, const Vector2& end_pos) {
  Vector2 p1 = TransformPoint(start_pos);
  Vector2 p2 = TransformPoint(end_pos);
  SDL_RenderDrawLine(renderer_, p1.x, p1.y, p2.x, p2.y);
}

void Display::DrawLines(const std::vector<Vector2>& points) {
  for (unsigned int i = 1; i < points.size(); ++i) {
    DrawLine(points[i - 1], points[i]);
  }
}
void Display::DrawLines(const std::vector<std::vector<Vector2>>& dataset) {
  for (const std::vector<Vector2>& points : dataset) {
    DrawLines(points);
  }
}

Vector2 Display::TransformPoint(const Vector2& point) {
  auto dx = point.x - view_x_offset_;
  auto dy = point.y - view_y_offset_;
  auto y = screen_height_ - (dx / view_height_) * screen_height_;
  auto x = (dy / view_width_) * screen_width_;
  return Vector2(x, y);
}

void Display::DrawText(const std::string text, const Vector2 pos,
                       const double scale, const SDL_Color color,
                       bool centered) {
  SDL_Surface* surface =
      TTF_RenderText_Blended(main_front_, text.c_str(), color);
  if (surface == nullptr) {
    std::cout << "Unable to render text surface! SDL_ttf Error: "
              << TTF_GetError() << std::endl;
    return;
  }

  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
  if (texture == nullptr) {
    SDL_FreeSurface(surface);
    std::cout << "Unable to create texture from rendered text! SDL Error: "
              << SDL_GetError() << std::endl;
    return;
  }

  int width = surface->w * scale;
  int height = surface->h * scale;
  int x = pos.x;
  int y = pos.y;

  if (centered) {
    x -= width / 2;
    y -= height / 2;
  }

  SDL_Rect renderQuad = {x, y, width, height};
  SDL_RenderCopy(renderer_, texture, nullptr, &renderQuad);

  SDL_DestroyTexture(texture);
  SDL_FreeSurface(surface);
}
}  // namespace pllee4