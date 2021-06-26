#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Populate the current_node.neighbors vector with all the neighbors
    current_node -> FindNeighbors();

    // For each node in current_node.neighbors
    for (auto neighbor : current_node -> neighbors) {

        // Calculate h_value
        neighbor->h_value = CalculateHValue(neighbor);

        // Set the parent
        neighbor->parent = current_node;

        // Set the g_value
        neighbor->g_value = distance + neighbor->distance(*current_node);

        // Add the neighbor to the open_list
        open_list.emplace_back(neighbor);

        // Indicate the neighbor has been visited
        neighbor->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

// Provide the comparison needed to sort the open list in the RoutePlanner::NextNode function
bool Compare(const RouteModel::Node *i, const RouteModel::Node *j) {
    float f1 = i->h_value + i->g_value;
    float f2 = j->h_value + j->g_value;
    return f1 < f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    
    // Sort the open_list according to the sum of the h value and g value
    std::sort(open_list.begin(), open_list.end(), Compare);

    // Create a pointer to the node in the list with the lowest sum, remove the node from the open list, and return the pointer
    RouteModel::Node *ptr = open_list.front();
    open_list.erase(open_list.begin());
    return ptr;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    // Push the current node onto the back of the path_found vector
    path_found.emplace_back(*current_node);

    // Initialize node iterator to the current node
    RouteModel::Node *node = current_node;

    // Loop through the chain of nodes until the start_node is found
    while (node != start_node) {

        // Add the distance from the node to its parent to the distance variable
        distance += node->distance(*(node->parent));

        std::cout << "distance = " << distance << "\n";
 
        // Push the parent node onto the back of the path_found vector
        path_found.emplace_back(*(node->parent));

        // Change the node iterator to the parent node
        node = node->parent;
    }

    // Reverse the order of the path_found vector so that the start_node is first and the end_node is last
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // Initialize the current node to the start node and add it to the open list
    current_node = start_node;
    current_node->visited = true;
    open_list.emplace_back(current_node);

    std::cout << "Start node x = " << start_node->x << "   ";
    std::cout << "Start node y = " << start_node->y << "\n";

    // While the end node has not been reached and theopen list is not empty
    while ((current_node != end_node) && (!open_list.empty())) {

        std::cout << "Current node x = " << current_node->x << "   ";
        std::cout << "Current node y = " << current_node->y << "\n";

        // Find the neighbors of the current node
        AddNeighbors(current_node);

        // Sort the open list and return the next node as the new current node
        current_node = NextNode();
    }
    
    std::cout << "Current node x = " << current_node->x << "   ";
    std::cout << "Current node y = " << current_node->y << "\n";
    std::cout << "End node x = " << end_node->x << "   ";
    std::cout << "End node y = " << end_node->y << "\n";

    // Return the final path found
    if (current_node == end_node) {
        m_Model.path = ConstructFinalPath(current_node);
    }
    else std::cout << "End node never reached!" << "\n";
}