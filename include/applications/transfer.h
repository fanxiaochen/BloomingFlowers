#ifndef TRANSFER_H
#define TRANSFER_H

#include <string>

class FlowersViewer;

class Transfer
{
public:
    Transfer(FlowersViewer* flower_viewer);
    void loadFlower(const std::string& flower, int key_frame);

    void getTransform(int frame);

    void transfer(bool is_forward);

private:
    FlowersViewer* flower_viewer_;
    Flower* current_flower_;

    Flower* key_flower_;
    int key_frame_;

    // Transform
};

#endif