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
    void transfer(int start_frame, int end_frame, bool is_forward);

protected:
    void update(int frame, bool is_forward);

    void updateWithOrder(int frame, bool is_forward);

private:
    FlowersViewer* flower_viewer_;

    Flower* current_flower_;
    int current_frame_;

    Flower* key_flower_;
    int key_frame_;

    std::string flower_folder_;

    std::string transform_folder_;
    std::vector<int> order_;
};

#endif