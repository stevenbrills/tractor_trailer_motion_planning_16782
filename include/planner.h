#ifndef PLANNER_TRACTOR_TRAILER_H
#define PLANNER_TRACTOR_TRAILER_H

struct leaf_node{
    std::vector<float> state{0,0,0,0,0,0}; //[x,y,theta,beta,xt,yt];
	std::vector<int> connected_vertices;
    std::vector<float> associated_point{0,0,0}; //[x,y,theta]
};

#endif