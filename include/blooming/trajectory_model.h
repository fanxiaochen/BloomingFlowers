#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H
#include <vector>

#include "point_cloud.h"
#include "renderable.h"


typedef struct  
{
    int hdl_idx_;
    std::vector<Point> pos_; 

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

protected:
    virtual void updateImpl(void);
    virtual void visualizeTrajectory(void);

private:
    std::vector<Trajectories>    trajs_set_;  // the size should be the same as petal number

    bool show_traj_;
};

#endif