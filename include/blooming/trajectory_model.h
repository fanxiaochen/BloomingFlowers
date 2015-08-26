#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H
#include <vector>

#include "point_cloud.h"
#include "renderable.h"

#include "open_curve.h"

typedef struct  
{
    int hdl_idx_;
    std::vector<Point> pos_; 

    ON_NurbsCurve fitting_curve_;
    Eigen::VectorXd t_;

    void resize(int num)
    {
        pos_.resize(num);
    }

    void push_back(Point p)
    {
        pos_.push_back(p);
    }

    Point& operator [] (int i)
    {
        return pos_[i];
    }

    int size() { return pos_.size(); }

    void reverse()
    {
        int left = 0;
        int right = pos_.size()-1;
        while (left < right)
        {
            Point tp = pos_[left];
            pos_[left] = pos_[right];
            pos_[right] = tp;

            left ++;
            right --;
        }
    }

    void curve_fitting()
    {
        int dims = 3;         // dimension of points & B-spline curve
        int cps = 5;         // number of control points
        int order = 4;        // polynomial order of B-spline curve
        bool clamp = false;   // clamp curve at ends, or leave them open

        nurbsfit::FitOpenCurve::Domain range(0,1);

        Eigen::VectorXd params; // parameter values in B-spline domain, corresponding to points
        Eigen::VectorXd points; // data points B-spline is fitted to (params.size * dims == points.size)

        // create parameters and data points
        params.resize(size());
        points.resize(3*size());
        for (int i = 0, i_end = size(); i < i_end; ++i)
        {
            params(i) = range.width() * (double(i)/(size()-1));
            Point& p = pos_.at(i);
            points(3*i) = p.x;
            points(3*i+1) = p.y;
            points(3*i+2) = p.z;
        }

        nurbsfit::FitOpenCurve fit;
        fit.initCurve(dims, order, cps, range, clamp);
        fit.initSolver(params);
        fit.solve(points);

        fitting_curve_ = fit.getCurve();
        t_ = params;
    }

    ON_NurbsCurve& getFittingCurve() { return fitting_curve_; }
    Eigen::VectorXd& getParas() { return t_; }

}Trajectory;

typedef struct 
{
    int petal_id_;
    std::vector<Trajectory> trajs_;

    void resize(int num)
    {
        trajs_.resize(num);
    }

    void push_back(Trajectory t)
    {
        trajs_.push_back(t);
    }

    Trajectory& operator [] (int i)
    {
        return trajs_[i];
    }

    int size() { return trajs_.size(); }

}Trajectories;


class TrajectoryModel: public Renderable
{
public:
    TrajectoryModel();

    void show();
    void hide();
    void update();

    void init(Flower* flower);

    /*inline Trajectories& getTrajectories(int petal_id){ return trajs_set_[petal_id]; }
    inline Trajectory& getTrajectory(int petal_id, int hdl_idx){ return trajs_set_[petal_id].trajs_[hdl_idx]; }*/

    void addFlowerPosition(Flower* flower);

    void reverseAll();

    void fittingAll();

    void recoverFromFlowerViewer(FlowersViewer* flower_viewer);

    inline std::vector<Trajectories>& getTrajsSet() { return trajs_set_; }

    inline std::vector<int>& getPetalOrder() { return petal_order_; }

protected:
    virtual void updateImpl(void);
    virtual void visualizeTrajectory(void);

private:
    std::vector<Trajectories>    trajs_set_;  // the size should be the same as petal number

    bool show_traj_;

    std::vector<std::vector<int>>  trajs_indices_; // based on flower

    std::vector<int>    petal_order_; // based on flower

};

#endif