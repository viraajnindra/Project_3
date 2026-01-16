#include <iostream>
#include <cmath>
#include <queue>
#include <stack>
#include <functional>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/
 
// *** Task: Implement this function if completing the advanced extensions *** //
std::vector<Cell> depthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /* BEGIN STUDENT CODE. */
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    std::stack<int> s;
    s.push(start_idx);

    graph.nodes[start_idx].visited = true;
    graph.nodes[start_idx].parent = -1;

    while (!s.empty())
    {
        int current = s.top();
        s.pop();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            path = tracePath(goal_idx, graph);
            return path;
        }
        std::vector<int> neighbors = findNeighbors(current, graph);
        for (int neighbor : neighbors)
        {
            if (checkCollision(neighbor, graph))
                continue;
            if (graph.nodes[neighbor].visited)
                continue;

            graph.nodes[neighbor].parent = current;
            //graph.nodes[neighbor].dist = graph.nodes[current].dist + sqrt(pow(graph.nodes[current].i-graph.nodes[neighbor].i,2)+pow(graph.nodes[current].j-graph.nodes[neighbor].j,2));
            graph.nodes[neighbor].visited = true;
            s.push(neighbor);
        }
    }
    /* END STUDENT CODE. */
    

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /* BEGIN STUDENT CODE. */
    std::queue<int> q;
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    graph.nodes[start_idx].parent = -1;
    graph.nodes[start_idx].dist = 0;
    graph.nodes[start_idx].visited = true;
    q.push(start_idx);
    graph.visited_cells.push_back(start);
    while (q.empty() == false) {
        int t = q.front();
        q.pop();
        if (t == goal_idx) {
            path = tracePath(t, graph);
            break;
        }
        std::vector<int> mandem = findNeighbors(t, graph);
        for (int i = 0; i < mandem.size(); i++) {
            int innit = mandem[i];
            if (checkCollision(innit, graph) == true)
                continue;
            if (graph.nodes[innit].visited == false) {
                graph.nodes[innit].parent = t;
                graph.nodes[innit].dist = graph.nodes[t].dist + 1;
                graph.nodes[innit].visited = true;
                q.push(innit);
                graph.visited_cells.push_back(idxToCell(innit, graph));
            }
        }
    }
    if (path.empty() && graph.nodes[goal_idx].visited) {
        path = tracePath(goal_idx, graph);
    }
    /* END STUDENT CODE. */

    return path;
}

// *** Task: Implement this function if completing the advanced extensions *** //
std::vector<Cell> iterativeDeepeningSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /* BEGIN STUDENT CODE. */
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    graph.nodes[start_idx].dist = 0.0;
    graph.nodes[start_idx].parent = -1;

    int num_nodes = graph.nodes.size();

    for (int i = 0; i < num_nodes; i++)
    {
        float min_dist = HIGH;
        int current = -1;

        for (int j = 0; j < num_nodes; j++)
        {
            if (!graph.nodes[j].visited && graph.nodes[j].dist < min_dist)
            {
                min_dist = graph.nodes[j].dist;
                current = j;
            }
        }

        if (current == -1)
            break;

        graph.nodes[current].visited = true;
        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
            return tracePath(goal_idx, graph);

        std::vector<int> neighbors = findNeighbors(current, graph);

        for (int neighbor : neighbors)
        {
            if (checkCollision(neighbor, graph))
                continue;

            float new_dist = graph.nodes[current].dist + 1;

            if (new_dist < graph.nodes[neighbor].dist)
            {
                graph.nodes[neighbor].dist = new_dist;
                graph.nodes[neighbor].parent = current;
            }
        }
    }
    /* END STUDENT CODE. */

    return path;
}

// *** Task: Implement this function if completing the advanced extensions *** //
std::vector<Cell> aStarSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /* BEGIN STUDENT CODE. */
    std::vector<int> open;
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    graph.nodes[start_idx].parent = -1;
    graph.nodes[start_idx].dist = 0;
    graph.nodes[start_idx].visited = false;
    open.push_back(start_idx);
    std::vector<int> used;
    for (int i = 0; i < graph.nodes.size(); i++) {
        used.push_back(0);
    }
    used[start_idx] = 1;
    graph.visited_cells.push_back(start);
    while (!open.empty()) {
        int low_idx = -1;
        int low_pos = 0;
        float low_f = HIGH;
        for (int k = 0; k < open.size(); k++) {
            int idx = open[k];
            Cell c = idxToCell(idx, graph);
            int dist_i = c.i - goal.i;
            int dist_j = c.j - goal.j;
            if (dist_i < 0) {
                dist_i = -dist_i;
            }
            if (dist_j < 0) {
                dist_j = -dist_j;
            }
            float g = graph.nodes[idx].dist;
            float h = dist_i + dist_j;
            float f = g + h;
            if (f < low_f) {
                low_f = f;
                low_idx = idx;
                low_pos = k;
            }
        }
        int t = low_idx;
        for (int i = low_pos; i < open.size() - 1; i++) {
            open[i] = open[i + 1];
        }
        open.pop_back();
        used[t] = 0;
        graph.nodes[t].visited = true;
        graph.visited_cells.push_back(idxToCell(t, graph));
        if (t == goal_idx) {
            path = tracePath(t, graph);
            break;
        }
        std::vector<int> mandem = findNeighbors(t, graph);
        for (int i = 0; i < mandem.size(); i++) {
            int innit = mandem[i];
            if (checkCollision(innit, graph) == true) {
                continue;
            }
            if (graph.nodes[innit].visited == true) {
                continue;
            }
            float g2 = graph.nodes[t].dist + 1;
            if (used[innit] == 0 || g2 < graph.nodes[innit].dist) {
                graph.nodes[innit].dist = g2;
                graph.nodes[innit].parent = t;
                if (used[innit] == 0) {
                    open.push_back(innit);
                    used[innit] = 1;
                    graph.visited_cells.push_back(idxToCell(innit, graph));
                }
            }
        }
    }
    if (path.size() == 0 && graph.nodes[goal_idx].visited == true) {
        path = tracePath(goal_idx, graph);
    }
    /* END STUDENT CODE. */

    return path;
}
