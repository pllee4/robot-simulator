/* 
 * app_kalman_filter.cpp
 * 
 * Created on: Feb 19, 2022 16:35
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <iostream>
#include <string>

#include "display/display.hpp"
#include "simulation/profile/simulation_profile.hpp"
#include "simulation/simulation.hpp"

// Screen dimension constants
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

using namespace pllee4;

// Main Loop
int main(int argc, char* args[]) {
  Display display;
  Simulation simulation;

  // Start Graphics
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "SDL could not initialize! SDL_Error: " << SDL_GetError()
              << std::endl;
    return -1;
  }
  if (TTF_Init() == -1) {
    std::cout << "SDL_ttf could not initialize! SDL_ttf Error: "
              << TTF_GetError() << std::endl;
    return -1;
  }

  // Create Display
  if (!display.CreateRenderer("AKFSF Simulations", SCREEN_WIDTH,
                              SCREEN_HEIGHT)) {
    return false;
  }

  // Main Simulation Loop
  simulation.Reset(LoadSimulation1Parameters());
  // simulation.SetTimeMultiplier(10);
  bool running = true;

  while (running) {
    // Update Simulation
    simulation.Update();

    // Update Display
    display.ClearScreen();

    // Draw Background Grid
    display.SetDrawColour(101, 101, 101);
    for (int x = -GRID_SIZE; x <= GRID_SIZE; x += GRID_SPACEING) {
      display.DrawLine(Vector2(x, -GRID_SIZE), Vector2(x, GRID_SIZE));
    }
    for (int y = -GRID_SIZE; y <= GRID_SIZE; y += GRID_SPACEING) {
      display.DrawLine(Vector2(-GRID_SIZE, y), Vector2(GRID_SIZE, y));
    }

    // Draw Simulation
    simulation.Render(display);

    display.ShowScreen();

    // Handle Events
    SDL_Event event;
  
    while (SDL_PollEvent(&event) != 0) {
      if (event.type == SDL_QUIT) {
        running = false;
      } else if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
          case SDLK_SPACE:
            simulation.TogglePauseSimulation();
            break;
          case SDLK_ESCAPE:
            running = false;
            break;
          case SDLK_KP_PLUS:
            simulation.IncreaseZoom();
            break;
          case SDLK_KP_MINUS:
            simulation.DecreaseZoom();
            break;
          case SDLK_RIGHTBRACKET:
            simulation.IncreaseTimeMultiplier();
            break;
          case SDLK_LEFTBRACKET:
            simulation.DecreaseTimeMultiplier();
            break;
          case SDLK_r:
            simulation.Reset();
            break;
          case SDLK_1:
            simulation.Reset(LoadSimulation1Parameters());
            break;
          case SDLK_2:
            simulation.Reset(LoadSimulation2Parameters());
            break;
          case SDLK_3:
            simulation.Reset(LoadSimulation3Parameters());
            break;
          case SDLK_4:
            simulation.Reset(LoadSimulation4Parameters());
            break;
          case SDLK_5:
            simulation.Reset(LoadSimulation5Parameters());
            break;
          case SDLK_6:
            simulation.Reset(LoadSimulation6Parameters());
            break;
          case SDLK_7:
            simulation.Reset(LoadSimulation7Parameters());
            break;
          case SDLK_8:
            simulation.Reset(LoadSimulation8Parameters());
            break;
          case SDLK_9:
            simulation.Reset(LoadSimulation9Parameters());
            break;
          case SDLK_0:
            simulation.Reset(LoadSimulation0Parameters());
            break;
        }
      }
    }
  }

  // Destroy Renderer
  display.DestroyRenderer();

  // Unload SDL
  TTF_Quit();
  SDL_Quit();

  return 0;
}