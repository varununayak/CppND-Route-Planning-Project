#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        if (neighbor->visited) {
            auto tentativeGscore = current_node->g_value + current_node->distance(*neighbor);
            // If shorter than when visited before
            if (tentativeGscore < neighbor->g_value) {
                neighbor->parent = current_node;
                neighbor->h_value = CalculateHValue(neighbor);
                neighbor->g_value = tentativeGscore;
            }
        } else {
            neighbor->visited = true;
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            open_list.push_back(neighbor);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(),
    [](const RouteModel::Node* a, const RouteModel::Node* b) -> bool
        { 
            return (a->g_value + a->h_value) < (b->g_value + b->h_value); 
        });
    auto nextNode = open_list.front();
    open_list.erase(open_list.begin(), open_list.begin() + 1);
    return nextNode;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    auto childNode = current_node;
    while (childNode != start_node) {
        path_found.push_back(*childNode);
        distance += childNode->distance(*(childNode->parent));
        childNode = childNode->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    auto current_node = start_node;
    start_node->visited = true;
    open_list.push_back(start_node);
    while (open_list.size()) {
        auto current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}