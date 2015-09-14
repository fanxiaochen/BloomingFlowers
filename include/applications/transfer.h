#ifndef TRANSFER_H
#define TRANSFER_H

#include <string>

class FlowerViewer;

class Transfer
{
public:
    Transfer(FlowerViewer* flower_viewer);
    void loadFlower(const std::string& flower);
    void transfer();

private:
    FlowerViewer* flower_viewer_;
    Flower* flower_;
};

#endif