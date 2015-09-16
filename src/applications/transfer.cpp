
#include "transfer.h"
#include "flower.h"

Transfer::Transfer(FlowersViewer* flower_viewer)
{
    flower_viewer_ = flower_viewer;
    key_flower_ = new Flower;
    current_flower_ = new Flower;
}

void Transfer::loadFlower(const std::string& flower, int key_frame)
{
    key_flower_->load(flower);
    key_frame_ = key_frame;

    *current_flower_ = *key_flower_; // deep copy
    current_frame_ = key_frame_;
}

void Transfer::setFlowerFolder(const std::string& flower_folder)
{
    flower_folder_ = flower_folder;
}

void Transfer::transfer(bool is_forward)
{
    int start_frame = flower_viewer_->getStartFrame();
    int end_frame = flower_viewer_->getEndFrame();

    if (is_forward)
    {
        for (int i = key_frame_; i < end_frame; ++ i)
        {
            //update
            update(i, is_forward);
        }
    }

    else 
    {
        for (int i = key_frame_; i > start_frame; -- i)
        {
            //update
            update(i, is_forward);
        }
    }

}

void Transfer::update(int frame, bool is_forward)
{
    Petals& petals = current_flower_->getPetals();

    osg::ref_ptr<Flower> t_flower = flower_viewer_->flower(frame);

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        int handle_num = petal.getSkeleton()->getJointNumber();

        Petal& t_petal = t_flower->getPetals()[i]; // for loading transforms of current frame

        // get T
        Eigen::MatrixXd T;

        if (is_forward)
            T = t_petal.getSkeleton()->getForwardTransforms();
        else
            T = t_petal.getSkeleton()->getBackwardTransforms();

        // get biharmonic weights
        Eigen::MatrixXd W = petal.getBiharmonicWeights();

        // get current vertices
        int petal_size = petal.getVertices()->size();
        Eigen::Matrix3Xd V(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            V.col(j) << petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z();
        }

        // get M
        Eigen::MatrixXd M(petal_size, 4*handle_num);

        Eigen::MatrixXd petal_matrix(petal_size, 4);
        petal_matrix.setOnes();
        petal_matrix.block(0, 0, petal_size, 3) = V.transpose();

        for (size_t j = 0, j_end = handle_num; j < j_end; ++ j)
        {
            M.block(0, 4*j, petal_size, 4) = W.col(j).asDiagonal() * petal_matrix;
        }

        // update new vertices / lbs
        Eigen::Matrix3Xd new_V = (M * T).transpose();

        for (size_t j = 0, j_end = petal.getVertices()->size(); j < j_end; ++ j)
        {
            petal.getVertices()->at(j).x() = new_V(0, j);
            petal.getVertices()->at(j).y() = new_V(1, j);
            petal.getVertices()->at(j).z() = new_V(2, j);
        }
        petal.updateNormals();

        // update new skeleton
        osg::ref_ptr<Skeleton> skeleton = petal.getSkeleton();
        Skeleton::Joints& joints = skeleton->getJoints();
        for (size_t i = 0; i < skeleton->getJointNumber(); ++ i)
        {
            auto& joint = joints[i];
            Eigen::MatrixXd t = T.block<4,3>(i*4,0);
            Eigen::Vector3d np = t.transpose() * Eigen::Vector4d(joint.x, joint.y, joint.z, 1);
            joint.x = np(0);
            joint.y = np(1);
            joint.z = np(2);
        }

        if (is_forward)
            current_flower_->save(flower_folder_, frame+1);
        else 
            current_flower_->save(flower_folder_, frame-1);
    }
}