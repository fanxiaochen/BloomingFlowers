
#include "transfer.h"
#include "flower.h"

Transfer::Transfer(FlowersViewer* flower_viewer)
{
    flower_viewer_ = flower_viewer;
    key_flower_ = new Flower;
}

void Transfer::loadFlower(const std::string& flower, int key_frame)
{
    key_flower_->load(flower);
    key_frame_ = key_frame;
}

void Transfer::getTransform(int frame)
{
    
}

void Transfer::transfer(bool is_forward)
{
    if (is_forward)
    {
        
    }

    else 
    {

    }
    current_flower_ = flower_viewer_->flower(key_frame_);

}
