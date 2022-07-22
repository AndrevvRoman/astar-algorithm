# A* search algorithm implementation

Simple implementation of search using A* algorithm allows user to see how search steps is preformed

## Table of Contents

- [Building](#building)
- [Running](#running)
- [Testing](#testing)

## Building

Clone project and build it with CMake:

```sh
git clone https://github.com/AndrevvRoman/astar-algorithm.git
```

You also need SFML intalled to your system:

```sh
sudo apt update
sudo apt install libsfml-dev
```

Then you can build project:

```sh
cd astar-algorithm
mkdir build && cd build
cmake ..
cmake --build .
```

## Running

After build you can run application with the following command:

```sh
./astar-algorithm
```

## Testing

If you made changes you can run unit tests after build to bu sure that search is working fine:

```sh
cd build/tests
./astar-algorithm-tests
```
