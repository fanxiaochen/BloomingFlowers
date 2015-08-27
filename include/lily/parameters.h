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