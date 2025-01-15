# Interrail Planner

The **Interrail Planner** is a C++ project that simulates a travel planning system across Europe. It uses a graph representation where nodes are cities, and edges represent the travel time (in hours) between cities. This tool provides functionality to perform various graph-based operations and helps recommend the most cost-effective Interrail pass for a journey.

## Features

1. **Graph Representation**:
   - Cities are nodes, and edges represent travel time between them.

2. **Extract Subgraph**:
   - Extract a subgraph of selected cities from the main graph.

3. **Minimum Spanning Tree (MST)**:
   - Calculate the MST using Prim's algorithm to find a set of connections that links all cities with the minimum total travel time.

4. **Shortest Path Calculation**:
   - Use Dijkstra's algorithm to find the shortest path between two cities.

5. **Traveling Salesman Problem (TSP)**:
   - Approximate a TSP solution using the nearest neighbor algorithm to generate an efficient tour.

6. **Interrail Pass Recommendation**:
   - Suggest the most cost-effective Interrail pass based on the travel plan.
   - Pass options are illustrative and may include fictional entries for functionality demonstration.

## How It Works

1. **Initialize the Graph**:
   - The cities and connections are pre-defined in the code.

2. **Run Functionalities**:
   - Extract subgraphs for a group of cities.
   - Calculate the MST for minimal connection costs.
   - Find the shortest path between cities.
   - Solve the TSP for a complete journey starting and ending in a city.
   - Recommend an Interrail pass for the planned tour.

3. **Output**:
   - The program outputs results such as the MST, shortest path, TSP tour, and Interrail pass recommendations directly to the console.

## Sample Cities and Connections

- **Rome** connects to:
  - Florence: 1.5 hours
  - Venice: 4.0 hours
  - Paris: 13.0 hours
  - Munich: 12.0 hours
- **Zurich** connects to:
  - Paris: 4.05 hours
  - Munich: 4.0 hours
  - Lyon: 5.5 hours

## How to Run

1. Clone the repository to your local machine:
   ```bash
   git clone https://github.com/<your-username>/interrail-planner.git
   cd interrail-planner
   ```

2. Compile the code using a C++ compiler:
   ```bash
   g++ -o interrail_planner interrail_planner.cpp
   ```

3. Run the program:
   ```bash
   ./interrail_planner
   ```

## Key Algorithms

- **Prim's Algorithm**: Used for finding the MST.
- **Dijkstra's Algorithm**: Used for calculating the shortest path.
- **Nearest Neighbor Heuristic**: Used for approximating the TSP solution.

## Limitations

- The Interrail pass options may not reflect the latest real-world offerings.
- Some pass options are fictional, included for functionality demonstration purposes.
- The TSP solution is an approximation and may not yield the optimal tour.

## Future Improvements

- Integrate real-world data for Interrail pass options.
- Enhance the TSP algorithm for improved accuracy.
- Add a user interface for better interaction.

## Author

Collin Graff

---

Feel free to fork, modify, and share this project. Contributions are welcome!
```

You can edit the file to include the actual URL for your GitHub repository and other details. Let me know if you'd like adjustments or further help!
