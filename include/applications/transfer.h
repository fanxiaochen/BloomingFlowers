#ifndef TRANSFER_H
#define TRANSFER_H

#include <string>

class FlowersViewer;

class Transfer
{
public:
    Transfer(FlowersViewer* flower_viewer);
    void loadFlower(const std::string& flower, int key_frame);

    void setFlowerFolder(const std::string& flower_folder);

    void transfer(bool is_forward);

protected:
    void update(int frame, bool is_forward);

private:
    FlowersViewer* flower_viewer_;

    Flower* current_flower_;
    int current_frame_;

    Flower* key_flower_;
    int key_frame_;

    std::string flower_folder_;
};

#endif