#ifndef TRANSFER_H
#define TRANSFER_H

#include <string>

class FlowersViewer;

// two modes including for whole flower and each petal
class Transfer
{
public:
    // for whole flower
    Transfer(FlowersViewer* flower_viewer);
    void loadFlower(const std::string& flower, int key_frame);
    void setFlowerFolder(const std::string& flower_folder);
    void transfer(bool is_forward);

    // for each petal
    Transfer(const std::string& transform_folder);
    void loadFlower(const std::string& template_flower, int key_frame, std::vector<int>& order);
    Eigen::MatrixXd getTransform(int petal_id, int frame, bool is_forward);
    void transfer(int start_frame, int end_frame);

    // motion transfer
    Transfer(const std::string& transform_folder, Flower* current_flower, int current_frame);
    Flower* update();
    Flower* updateBackward();
    Flower* updateForward();

protected:
    void update(int frame, bool is_forward);

    void updateBackward(int frame);
    void updateForward(int frame);


private:
    FlowersViewer* flower_viewer_;     // to get the flower sequence

    Flower* current_flower_;  // the new flower which have similar pose as key flower
	std::vector<Eigen::MatrixXd>  petal_transformations_;  //each petal of the new flower may need a global transform
    int current_frame_;	

    Flower* key_flower_;      // the old flower
    int key_frame_;

    std::string flower_folder_;

    std::string transform_folder_;
    std::vector<int> order_;
};

#endif