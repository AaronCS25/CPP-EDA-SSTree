# README

## Dependencias (on linux)
```cpp
sudo apt-get install build-essential
sudo apt-get install libeigen3-dev
```

## Ejecución
```bash
g++ -std=c++17 main.cpp Point.cpp SSTree.cpp -o main -Wall -g -I/usr/include/eigen3
```