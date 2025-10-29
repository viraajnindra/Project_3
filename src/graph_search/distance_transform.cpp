#include <iostream>
#include <vector>
#include <cmath>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/distance_transform.h>


void distanceTransformSlow(GridGraph& graph)
{
    // *** Task: Implement this function if completing the advanced extensions *** //
    //BEGIN STUDENT CODE
    //END STUDENT CODE
    return;
}

void distanceTransformManhattan(GridGraph& graph)
{
    // *** Task: Implement this function if completing the advanced extensions *** //
    // BEGIN STUDENT CODE
    // END STUDENT CODE.
    return;
}

std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt)
{
    std::vector<float> dt(init_dt.begin(), init_dt.end());

    std::vector<int> paras = {0};
    std::vector<float> ranges = {-HIGH, HIGH};

    // *** Task: Implement this function if completing the advanced extensions *** //
    // BEGIN STUDENT CODE.
    // END STUDENT CODE.
    return dt;
}

void distanceTransformEuclidean2D(GridGraph& graph)
{
    // *** Task: Implement this function if completing the advanced extensions *** //
    // BEGIN STUDENT CODE.
    // END STUDENT CODE.
    return;
}
