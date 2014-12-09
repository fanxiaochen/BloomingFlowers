#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>

class Parameters
{
public:
    Parameters();
    ~Parameters();

    bool load(const std::string& filename);
    bool save(const std::string& filename);

    inline int getStartFrame() { return start_frame_; }
    inline int getEndFrame() { return end_frame_; }
    inline int getKeyFrame() { return key_frame_; }

private:
    int start_frame_;
    int end_frame_;
    int key_frame_;
};

#endif