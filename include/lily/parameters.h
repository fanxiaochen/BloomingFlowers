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

    inline int& getStartFrame() { return start_frame_; }
    inline int& getEndFrame() { return end_frame_; }
    inline int& getKeyFrame() { return key_frame_; }

    inline int& getIterNum() { return iter_num_; }
    inline double& getEps() { return eps_; }
    inline double& getBoundaryFitting() { return boundary_fitting_; }
    inline double& getTipFitting() { return tip_fitting_; }
    inline double& getInnerFitting() { return inner_fitting_; }
    inline double& getSkelSmooth() { return skel_smooth_; }
    inline double& getCollision() { return collision_; }
    inline double& getARAP() { return arap_; }
    inline double& getClosure() { return closure_; }
    inline double& getInterpolate() { return interpolate_; }
    inline double& getNoiseP() { return noise_p_; }
    inline float& getMovingRatio() { return moving_ratio_; }
    inline std::string& getClosureIds() { return closure_ids_; }
    inline int& getClosureStartFrame() { return closure_start_frame_; }


    inline int& getBinNum() { return bin_num_; }
    inline float& getKnnRadius() { return knn_radius_; }
    inline int& getNoiseK() { return noise_k_; }
    inline int& getMinBoundary() { return min_boundary_; }
    inline float& getTipRadius() { return tip_radius_; }

    inline double& getSegmentRatio() { return segment_ratio_; }
    inline int& getCompletionDegree() { return completion_degree_; }

    inline std::string& getPetalOrder() { return petal_order_; }
    inline int& getPetalNum() { return petal_num_; }

    inline float& getCameraZoffset() { return z_offset_; }

    Eigen::MatrixXi getPetalRelation();

private:

    // frame paras
    int start_frame_;
    int end_frame_;
    int key_frame_;

    // solver paras
    int iter_num_;
    double eps_;
    double tip_fitting_;
    double boundary_fitting_;
    double inner_fitting_;
    double skel_smooth_;
    double collision_;
    double arap_;
    double closure_;
    double interpolate_;
    double noise_p_;
    float moving_ratio_;
    std::string closure_ids_;
    int closure_start_frame_;

    // boundary paras
    int bin_num_;
    float knn_radius_;
    int noise_k_;
    int min_boundary_;
    float tip_radius_;

    // segment paras
    double segment_ratio_;
    int completion_degree_;

    // petal order
    std::string petal_order_;
    int petal_num_;

    // camera
    float z_offset_;
};

#endif