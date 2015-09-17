
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

Transfer::Transfer(const std::string& transform_folder)
{
    transform_folder_ = transform_folder;
    key_flower_ = new Flower;
    current_flower_ = new Flower;
}

void Transfer::loadFlower(const std::string& template_flower, int key_frame, std::vector<int>& order)
{
    loadFlower(template_flower, key_frame);

    order_ = order;
}

Eigen::MatrixXd Transfer::getTransform(int petal_id, int frame, bool is_forward)
{
    Eigen::MatrixXd T;

    std::string petal_folder = transform_folder_ + "/" + QString("petal-%1").arg(petal_id).toStdString();
    std::string transform_file;
    if (is_forward)
        transform_file = petal_folder + "/" + QString("%1.ft").arg(frame).toStdString();
    else
        transform_file = petal_folder + "/" + QString("%1.bt").arg(frame).toStdString();

    std::ifstream infile;
    infile.open( transform_file.c_str());

    std::stringstream sem; 
    sem << infile.rdbuf(); 

    int handle_num;
    sem >> handle_num;

    T.resize(handle_num * 4, 4);

    double x, y, z;
    int row_num = 0, count = 0;
    

    for (size_t i = 0; i < handle_num; ++ i)
    {
        Eigen::MatrixXd t(4, 4);
        for (size_t j = 0; j < 4; ++ j)
        {
            sem >> x >> y >> z;
            if (j != 3)
                t.row(j) << x, y, z, 0;
            else 
                t.row(j) << x, y, z, 1;
        }

        T.block(i*4, 0, 4, 4) = t.transpose();
    }

    infile.close();

    return T;
}


// hide backward process, all is in order
void Transfer::transfer(int start_frame, int end_frame)
{
    if (key_frame_ < start_frame || key_frame_ > end_frame)
    {
        std::cout << "frame number should be right!!" << std::endl;
        return;
    }
    
    // backward
    for (int i = start_frame+1; i <= key_frame_; ++ i)
    {
        updateBackward(i);
    }

    // forward
    for (int i = key_frame_; i < end_frame; ++ i)
    {
        updateForward(i);
    }
}

void Transfer::updateForward(int frame)
{
    Petals& petals = current_flower_->getPetals();

    assert(petals.size() == order_.size());

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        int handle_num = petal.getSkeleton()->getJointNumber();

        // get T
        Eigen::MatrixXd T;
        T = getTransform(order_[i], frame, true);


        // get biharmonic weights
        Eigen::MatrixXd W = petal.getBiharmonicWeights();

        // get current vertices
        int petal_size = petal.getVertices()->size();
        Eigen::MatrixXd V(4, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            V.col(j) << petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z(), 1;
        }

        // update new vertices
        Eigen::MatrixXd new_V(4, petal_size);

        for (size_t i = 0, i_end = V.cols(); i < i_end; ++ i)
        {
            Eigen::MatrixXd W_i(4, 4*handle_num);
            for (size_t j = 0; j < handle_num; ++ j)
            {
                W_i.block(0, j*4, 4, 4) = W(i, j) * Eigen::MatrixXd::Identity(4, 4);
            }

            Eigen::MatrixXd M_i = W_i * T;
            
            new_V.col(i) = M_i * V.col(i);
        }

        // write back to petal
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
            Eigen::MatrixXd t = T.block<4,4>(i*4,0);
            Eigen::Vector4d np = t * Eigen::Vector4d(joint.x, joint.y, joint.z, 1);
            joint.x = np(0);
            joint.y = np(1);
            joint.z = np(2);
        }

        current_flower_->save(flower_folder_, frame+1);

    }
}


void Transfer::updateBackward(int frame)
{
    Petals& petals = current_flower_->getPetals();

    assert(petals.size() == order_.size());

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        int handle_num = petal.getSkeleton()->getJointNumber();

        // get T
        Eigen::MatrixXd T;
        T = getTransform(order_[i], frame, false);


        // get biharmonic weights
        Eigen::MatrixXd W = petal.getBiharmonicWeights();

        // get current vertices
        int petal_size = petal.getVertices()->size();
        Eigen::MatrixXd V(4, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            V.col(j) << petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z(), 1;
        }

        // update new vertices
        Eigen::MatrixXd new_V(4, petal_size);

        for (size_t i = 0, i_end = V.cols(); i < i_end; ++ i)
        {
            Eigen::MatrixXd W_i(4, 4*handle_num);
            for (size_t j = 0; j < handle_num; ++ j)
            {
                W_i.block(0, j*4, 4, 4) = W(i, j) * Eigen::MatrixXd::Identity(4, 4);
            }

            Eigen::MatrixXd M_i = W_i * T;

            new_V.col(i) = M_i.inverse() * V.col(i);
        }

        // write back to petal
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
            Eigen::MatrixXd t = T.block<4,4>(i*4,0);
            Eigen::Vector4d np = t * Eigen::Vector4d(joint.x, joint.y, joint.z, 1);
            joint.x = np(0);
            joint.y = np(1);
            joint.z = np(2);
        }

        current_flower_->save(flower_folder_, frame);

    }
}



