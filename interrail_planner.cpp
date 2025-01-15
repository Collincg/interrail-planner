// interrail_planner.cpp
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <queue>
#include <limits>
#include <tuple>
#include <stack>
#include <algorithm>

using namespace std;

// Type definitions
using Graph = unordered_map<string, unordered_map<string, double>>;
using Edge = tuple<double, string, string>; // (distance, cityA, cityB)
using EdgeDijkstra = pair<double, string>; // (distance from start, city)


// step 1: parse data. 
void initializeGraph(Graph &graph) {
    // Manually adding the parsed data into the C++ graph
    graph["Rome"] = {{"Florence", 1.5}, {"Venice", 4.0}, {"Bucharest", 10.0}, {"Paris", 13.0}, {"Munich", 12.0}, {"Monaco", 4.5}};
    graph["Zurich"] = {{"Paris", 4.05}, {"Munich", 4.0}, {"Lyon", 5.5}};
    graph["Sarajevo"] = {{"Budapest", 8.0}};
    graph["Florence"] = {{"Rome", 1.5}, {"Venice", 3.0}, {"Bucharest", 18.0}};
    graph["Venice"] = {{"Rome", 4.0}, {"Florence", 3.0}};
    graph["Bucharest"] = {{"Rome", 10.0}, {"Florence", 18.0}, {"Budapest", 12.0}, {"Istanbul", 8.0}, {"Kiev", 15.0}};
    graph["Paris"] = {{"Rome", 13.0}, {"Zurich", 4.05}, {"London", 2.5}, {"Lyon", 2.0}, {"Marseille", 3.5}};
    graph["Lisbon"] = {{"Barcelona", 12.0}, {"Madrid", 1.0}};
    graph["Barcelona"] = {{"Lisbon", 12.0}, {"Madrid", 3.0}, {"Marseille", 4.0}};
    graph["Madrid"] = {{"Lisbon", 1.0}, {"Barcelona", 3.0}};
    graph["Budapest"] = {{"Sarajevo", 8.0}, {"Bucharest", 12.0}, {"Warsaw", 10.0}};
    graph["Minsk"] = {{"Moscow", 8.0}, {"Warsaw", 7.0}, {"Kiev", 7.0}};
    graph["Gothenburg"] = {{"Oslo", 3.0}, {"Stockholm", 3.0}};
    graph["Athens"] = {{"Munich", 6.0}, {"Istanbul", 10.0}};
    graph["Riga"] = {{"Helsinki", 1.0}, {"Warsaw", 6.0}};
    graph["Manchester"] = {{"London", 2.0}};
    graph["Oslo"] = {{"Gothenburg", 3.0}};
    graph["Malmö"] = {{"Stockholm", 0.5}};
    graph["Stockholm"] = {{"Gothenburg", 3.0}, {"Malmö", 0.5}, {"Helsinki", 1.0}};
    graph["Cologne"] = {{"Munich", 4.5}, {"Amsterdam", 2.5}};
    graph["Helsinki"] = {{"Riga", 1.0}, {"Stockholm", 1.0}, {"Moscow", 3.0}};
    graph["Moscow"] = {{"Minsk", 8.0}, {"Helsinki", 3.0}, {"Kiev", 9.0}};
    graph["Munich"] = {{"Rome", 12.0}, {"Zurich", 4.0}, {"Athens", 6.0}, {"Cologne", 4.5}};
    graph["Warsaw"] = {{"Budapest", 10.0}, {"Minsk", 7.0}, {"Riga", 6.0}, {"Kiev", 9.0}};
    graph["Istanbul"] = {{"Bucharest", 8.0}, {"Athens", 10.0}};
    graph["London"] = {{"Paris", 2.5}, {"Manchester", 2.0}, {"Amsterdam", 1.0}};
    graph["Kiev"] = {{"Bucharest", 15.0}, {"Minsk", 7.0}, {"Moscow", 9.0}, {"Warsaw", 9.0}};
    graph["Amsterdam"] = {{"Cologne", 2.5}, {"London", 1.0}};
    graph["Lyon"] = {{"Zurich", 5.5}, {"Paris", 2.0}, {"Marseille", 3.0}};
    graph["Monaco"] = {{"Rome", 4.5}, {"Marseille", 2.5}};
    graph["Marseille"] = {{"Paris", 3.5}, {"Barcelona", 4.0}, {"Lyon", 3.0}, {"Monaco", 2.5}};
}


// Step 2: Extract a subgraph
Graph extractSubgraph(const Graph &graph, const vector<string> &selectedCities) {
    Graph subgraph;
    
    // loop through each city in the list of selected cities
    for (const string &city : selectedCities){

        // If the city exists in the main graph, add it to the subgraph and initialize it empty.
        if (graph.find(city) != graph.end()){
            subgraph[city] = {};
        }

        // copy connections to other selected cities
        for (const auto &[neighbor, distance] : graph.at(city)){
            if (find(selectedCities.begin(), selectedCities.end(), neighbor) != selectedCities.end()){
                subgraph[city][neighbor] = distance;
            }
        }
    }
    return subgraph;
}


// Step 3: Calculate the Minimum Spanning Tree (MST)
vector<Edge> primMST(Graph &graph, const string &startCity) {
    unordered_set<string> visitedCities;
    vector<Edge> mstEdges;
    priority_queue<Edge, vector<Edge>, greater<Edge>> minHeap; // greater<Edge> tells it prioritize smaller items first, making it a min-heap
    visitedCities.insert(startCity);

    // Add all edges from the starting city to the priority queue
    for (const auto &[neighbor, distance] : graph[startCity]){
        minHeap.push(std::make_tuple(distance, startCity, neighbor));
    }

    // Expand the MST
    while (!minHeap.empty()){
        auto [distance, cityA, cityB] = minHeap.top();
        minHeap.pop();
        
        // skip this edge if cityB is already visited
        if (visitedCities.find(cityB) != visitedCities.end()){
            continue;
        }

        // Add an unvisited edge to the MST
        mstEdges.push_back({distance, cityA, cityB});
        visitedCities.insert(cityB);

        // Add all edges from cityB to the priority queue if the neighbor is not visited
        for (const auto &[neighbor, distance] : graph[cityB]){
            if (visitedCities.find(neighbor) == visitedCities.end()){
                minHeap.push({distance, cityB, neighbor});
            }
        }

        // stop looping when MST is complete (edges = nodes-1)
        if (mstEdges.size() == graph.size() - 1){
            break;
        }
    }
    return mstEdges;
}

// Function to print the MST
void printMST(const vector<Edge> &mstEdges) {
    cout << "Minimum Spanning Tree (MST):" << endl;
    for (const auto &[distance, cityA, cityB] : mstEdges) {
        cout << cityA << " --(" << distance << ")-- " << cityB << endl;
    }
}

// Step 4: Find the shortest path using Dijkstra's algorithm
vector<string> dijkstra(const Graph &graph, const string &startCity, const string &endCity) {
    unordered_map<string, double> distanceMap;
    unordered_map<string, string> previousMap;

    // initializing distance map with all nodes being at a distance of infinity
    for (const auto &[city, _] : graph){
        distanceMap[city] = numeric_limits<double>::infinity();
    } 
    distanceMap[startCity] = 0;

    // Min-heap priority queue (distance, city)
    priority_queue<EdgeDijkstra, vector<EdgeDijkstra>, greater<EdgeDijkstra>> minHeap;
    minHeap.push({0, startCity});

    while (!minHeap.empty()){
        auto [currDistance, currCity] = minHeap.top();
        minHeap.pop();

        if (currCity == endCity){ // stopping when end city is reached
            break;
        }

        // Update the distance to neighboring cities
        for (const auto &[neighbor, distance] : graph.at(currCity)){
            double newDistance = currDistance + distance;
            if (newDistance < distanceMap[neighbor]){
                distanceMap[neighbor] = newDistance;
                minHeap.push({newDistance, neighbor});
                previousMap[neighbor] = currCity;
            }
        }
    }

    // Reconstruct the path from previous map
    vector<string> path;
    for (string at = endCity; at != ""; at = previousMap[at]){
        path.push_back(at);
        if (at == startCity) break;
    }
    reverse(path.begin(), path.end());

    return path;
}

// Function to print the shortest path
void printPath(const vector<string> &path) {
    if (path.empty()) {
        cout << "No path found." << endl;
        return;
    }

    cout << "Shortest path: ";
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i];
        if (i < path.size() - 1) cout << " -> ";
    }
    cout << endl;
}


// Step 5 rework (Traveling Salesman Problem)
pair<vector<string>, double> nearestNeighborTSP(const Graph &graph, const string &startCity) {
    unordered_set<string> visitedCities;
    vector<string> tour;
    stack<string> cityStack;
    string currentCity = startCity;
    double totalDistance = 0.0;

    tour.push_back(startCity);
    visitedCities.insert(startCity);
    cityStack.push(startCity);

    // Main loop to build the tour
    while (visitedCities.size() < graph.size()) {
        string nearestNeighbor;
        double minDistance = numeric_limits<double>::infinity();

        // Find the nearest unvisited neighbor
        for (const auto &[neighbor, distance] : graph.at(currentCity)) {
            if (visitedCities.find(neighbor) == visitedCities.end() && distance < minDistance) {
                minDistance = distance;
                nearestNeighbor = neighbor;
            }
        }

        if (!nearestNeighbor.empty()) {
            // Found an unvisited neighbor, add it to the tour
            tour.push_back(nearestNeighbor);
            visitedCities.insert(nearestNeighbor);
            cityStack.push(nearestNeighbor);
            totalDistance += minDistance;
            currentCity = nearestNeighbor;
        } else {
            // No unvisited neighbors found, backtrack
            cout << "No unvisited neighbors found for " << currentCity << ". Backtracking." << endl;

            string previousCity = cityStack.top();
            cityStack.pop();

            // If the stack is empty, we have backtracked to the start and there are no unvisited neighbors left
            if (cityStack.empty()) {
                cout << "All cities visited or no valid tour found." << endl;
                break;
            }

            // Backtrack to the previous city
            currentCity = cityStack.top();
            double backtrackDistance = graph.at(previousCity).at(currentCity);
            tour.push_back(currentCity);
            totalDistance += backtrackDistance;
        }
    }

    // Check if a direct edge exists from the last city to the startCity
    const string &lastCity = tour.back();
    if (graph.at(lastCity).find(startCity) != graph.at(lastCity).end()) {
        // Direct connection exists, add it to the tour
        tour.push_back(startCity);
        totalDistance += graph.at(lastCity).at(startCity);
    } else {
        // No direct connection, use dijkstra() to find the shortest path back
        vector<string> pathToStart = dijkstra(graph, lastCity, startCity);
        if (!pathToStart.empty()) {
            // Add the path back to the starting city and update the total distance
            for (size_t i = 1; i < pathToStart.size(); ++i) {
                const string &from = pathToStart[i - 1];
                const string &to = pathToStart[i];
                tour.push_back(to);
                totalDistance += graph.at(from).at(to);
            }
        } else {
            cout << "Failed to find a path back to the starting city." << endl;
        }
    }

    return {tour, totalDistance};
}

// Function to print the TSP tour
void printTour(const vector<string> &tour) {
    cout << "TSP Tour: ";
    for (size_t i = 0; i < tour.size(); ++i) {
        cout << tour[i];
        if (i < tour.size() - 1) cout << " -> ";
    }
    cout << endl;
}

// Step 6: Suggest the most cost-effective Interrail pass
void suggestInterrailPass(const vector<string> &tour, const double totalDistance) {
    // Sample pass options (prices are illustrative, not actual values)
    struct PassOption {
        string name;
        int travelDays;
        double price;
    };

    vector<PassOption> passOptions = {
        {"5 travel days in 1 month", 5, 200.0},
        {"7 travel days in 1 month", 7, 250.0},
        {"10 travel days in 1 month", 10, 300.0},
        {"Unlimited travel for 1 month", 30, 500.0},
        {"Unlimited travel for 2 months", 60, 900.0}
    };

    // Count the number of travel days (each transition between cities is a travel day)
    int travelDays = tour.size() - 1;

    // Suggest a pass based on the number of travel days
    PassOption bestOption = passOptions.back(); // Start with the most extensive option
    for (const auto &option : passOptions) {
        if (travelDays <= option.travelDays) {
            bestOption = option;
            break;
        }
    }

    cout << "Recommended Interrail Pass: " << bestOption.name << " (Price: $" << bestOption.price << ")" << endl;
    cout << "Total travel days: " << travelDays << endl;
    cout << "Total distance covered: " << totalDistance << " km" << endl;
}

void makeUndirected(Graph &graph) { // function to make a graph undirected
    for (const auto &[city, neighbors] : graph) {
        for (const auto &[neighbor, distance] : neighbors) {
            if (graph[neighbor].find(city) == graph[neighbor].end()) {
                graph[neighbor][city] = distance;
            }
        }
    }
}


// Main function for testing
int main() {
    Graph cityGraph;
    initializeGraph(cityGraph);

    // TEST STEP 1
    string city = "Rome";
    cout << city << " connects to:\n";
    for (const auto &[neighbor, distance] : cityGraph[city]) {
        cout << "  " << neighbor << " with distance " << distance << "\n";
    }


    // TEST STEP 2
    // Define a list of selected cities
    vector<string> selectedCities = {"Rome", "Venice", "Paris"};

    // Extract the subgraph
    Graph subgraph = extractSubgraph(cityGraph, selectedCities);

    // Print the subgraph
    cout << "\nSubgraph is: \n\n";
    for (const auto &[city, neighbors] : subgraph) {
        cout << city << " connects to:\n";
        for (const auto &[neighbor, distance] : neighbors) {
            cout << "  " << neighbor << " with distance " << distance << "\n";
        }
    }
    cout << "\n";


    // TEST STEP 3

    Graph graph;
    graph["Rome"] = {{"Florence", 1.5}, {"Venice", 4.0}, {"Paris", 13.0}};
    graph["Florence"] = {{"Rome", 1.5}, {"Venice", 3.0}};
    graph["Venice"] = {{"Rome", 4.0}, {"Florence", 3.0}, {"Paris", 3.5}};
    graph["Paris"] = {{"Rome", 13.0}, {"Zurich", 4.05}};
    graph["Zurich"] = {{"Paris", 4.05}, {"Munich", 4.0}};
    graph["Munich"] = {{"Zurich", 4.0}, {"Florence", 18.0}};

    // Adding reverse connections to make the graph undirected
    makeUndirected(graph);

    // vector<Edge> mstEdges = primMST(graph, "Rome");
    vector<Edge> mstEdges = primMST(cityGraph, "Rome");
    printMST(mstEdges);


    // TEST STEP 4

    // Run Dijkstra's algorithm from "Rome" to "Munich"
    // vector<string> shortestPath = dijkstra(graph, "Rome", "Munich");
    vector<string> shortestPath = dijkstra(cityGraph, "Madrid", "Stockholm");

    // Print the shortest path
    printPath(shortestPath);


    // TEST STEP 5 REWORK
    auto [tour, totalDistance] = nearestNeighborTSP(cityGraph, "Rome");
    printTour(tour);
    cout << "Total tour distance: " << totalDistance << "\n" << endl;


    // TEST STEP 6
    suggestInterrailPass(tour, totalDistance);

    return 0;
}

