# Planning in Dynamic Environments

A MATLAB animated simulation package for various path planning techniques in changing environments included a new proposed technique.

<p align="center">
  <img src="https://user-images.githubusercontent.com/37188590/154152987-3c84f6b9-1232-461b-a6d3-b26a36ce390f.gif">
</p>
<p align="center">
  <i>the blue agent is constantly replanning to reach its goal without colliding with emerging agents.</i>
</p>

Used in the research paper [Static and Dynamic Path Planning Using Incremental Heuristic Search](https://arxiv.org/abs/1804.07276).

written in DLR, Braunschweig, Germany

## Included Files:

### search problem formation function

- createEmptySearchProblem

### search problem modification functions

- addAgent
- updateAgentsBounce
- updateAgentsRepeat

### search problem inspection functions

- getAgentPosition
- isChanged

### grid formation functions

- createEmptyGrid
- createGridMap

### grids

- roadGrid
- oneRoadGrid
- oneRoadGrid2

### obstacle checking functions

- isFree
- isFreeR
- isFreeD

### Successor functions

- neighbors8F
- succRight5S
- succRightCS
- succRightCSV

### search functions

- AStar
- AStarGraph
- ARAStar
- tracePath

### heuristics

- hEuclideanFixedV
- hEuclideanMaxV
- hEuclideanMaxA

### graphing function

- plotD

### executable test scripts

- roadTests
- oneRoadTests
