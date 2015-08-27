#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>

class Parameters
{
public:
    Parameters();
    ~Parameters();

    bool load(const std::string& filename);
    bool save(const std::string& filename);

    inline int getStartFrame() { return start_frame_; }
    inline int getEndFrame() { return end_frame_; }
    inline int getKeyFrame() { return key_frame_; }

    inline int getIterNum() { return iter_num_; }
    inline double getEps() { return eps_; }
    inline double getBoundaryFitting() { return boundary_fitting_; }
    inline double getInnerFitting() { return inner_fitting_; }
    inline double getSkelSmooth() { return skel_smooth_; }
    inline double getCollision() { return collision_; }
    inline double getNoiseP() { return noise_p_; }
    inline float getMovingRatio() { return moving_ratio_; }


    inline int getBinNum() { return bin_num_; }
    inline float getKnnRadius() { return knn_radius_; }
    inline int getNoiseK() { return noise_k_; }


    inline double getSegmentRatio() { return segment_ratio_; }
    inline int getCompletionDegree() { return completion_degree_; }

private:

    // frame paras
    int start_frame_;
    int end_frame_;
    int key_frame_;

    // solver paras
    int iter_num_;
    double eps_;
    double boundary_fitting_;
    double inner_fitting_;
    double skel_smooth_;
    double collision_;
    double noise_p_;
    float moving_ratio_;

    // boundary paras
    int bin_num_;
    float knn_radius_;
    int noise_k_;

    // segment paras
    double segment_ratio_;
    int completion_degree_;
};

#endif