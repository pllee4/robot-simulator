# Robot-simulator
[![Build](https://github.com/pllee4/robot-simulator/actions/workflows/build.yml/badge.svg)](https://github.com/pllee4/robot-simulator/actions/workflows/build.yml)

- Repository for 2D robot simulator

## Dependencies

```
$ sudo apt install libeigen3-dev libsdl2-dev libsdl2-ttf-dev
```

## Run

```
$ mkdir build && cd build
$ cmake ..
$ make -j
$./bin/app_kalman_filter
```

![simulator](https://user-images.githubusercontent.com/42335542/158397356-4ca7bac2-5a2e-4032-a3b1-4f81ed95f986.gif)