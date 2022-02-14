/*
 * display.hpp
 * Created on: Feb 14, 2022 21:40
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "types/types.hpp"

namespace pllee4 {
template <typename... Args>
std::string string_format(const std::string& format, Args... args) {
  size_t size = snprintf(nullptr, 0, format.c_str(), args...) +
                1;  // Extra space for '\0'
  if (size <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  std::unique_ptr<char[]> buf(new char[size]);
  snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(),
                     buf.get() + size - 1);  // We don't want the '\0' inside
}

class Display {
 public:
  Display();
  ~Display();

  bool CreateRenderer(std::string title, int screen_width, int screen_height);
  void DestroyRenderer();

  void ShowScreen();
  void ClearScreen();

  double GetScreenWidth() const { return screen_width_; }
  double GetScreenHeight() const { return screen_height_; }
  double GetScreenAspectRatio() const {
    return GetScreenWidth() / GetScreenHeight();
  }

  void DrawText(const std::string text, const Vector2 pos,
                const double scale = 1, const SDL_Color color = {0, 0, 0},
                bool centered = false);

  void SetDrawColour(uint8_t red, uint8_t green, uint8_t blue,
                     uint8_t alpha = 0xFF);
  void SetView(double width, double height, double x_offset, double y_offset);
  void SetView(double x_offset, double y_offset);
  void DrawLine(const Vector2& start_pos, const Vector2& end_pos);
  void DrawLines(const std::vector<Vector2>& points);
  void DrawLines(const std::vector<std::vector<Vector2>>& points);

 private:
  int screen_width_;
  int screen_height_;

  double view_width_;
  double view_height_;
  double view_x_offset_;
  double view_y_offset_;

  SDL_Window* window_;
  SDL_Renderer* renderer_;
  TTF_Font* main_front_;

  Vector2 TransformPoint(const Vector2& point);
};
}  // namespace pllee4
#endif /* DISPLAY_HPP */
