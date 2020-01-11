#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

template <typename T, int SZ>
class RingBuffer {
public:
    RingBuffer();
    void reset();
    bool empty() const;
    bool full() const;
    size_t capacity() const;
    size_t size() const;
    void add(T item);
    T* getLatest();
    T* getSecondLatest();

private:
    size_t read_ = 0;
    size_t write_ = 0;
    const size_t max_size_;
    bool full_ = 0;
    T buffer[SZ];
};

template <typename T,int SZ>
RingBuffer<T,SZ>::RingBuffer() :
        max_size_(SZ)
{ /*empty*/ }

template <typename T,int SZ>
void RingBuffer<T,SZ>::reset() {
    read_ = write_;
    full_ = false;
}

template <typename T,int SZ>
bool RingBuffer<T,SZ>::empty() const {
    //if head and tail are equal, we are empty
    return (!full_ && (read_ == write_));
}

template <typename T,int SZ>
bool RingBuffer<T,SZ>::full() const {
    //If tail is ahead the head by 1, we are full
    return full_;
}

template <typename T,int SZ>
size_t RingBuffer<T,SZ>::capacity() const {
    return max_size_;
}

template <typename T,int SZ>
size_t RingBuffer<T,SZ>::size() const {
    size_t size = max_size_;

    if (!full_) {
        if (read_ >= write_) {
            size = read_ - write_;
        } else {
            size = max_size_ + read_ - write_;
        }
    }
    std::cout << "Size: " << size << std::endl;
    return size;
}

template <typename T,int SZ>
void RingBuffer<T,SZ>::add(T item) {

    buffer[write_] = item;
    //move add position to next slot
    write_ = (write_ + 1) % max_size_;

    //if buffer was already full, move read position to next slot since it got overwritten
    if (full_) {
        read_ = (read_ + 1) % max_size_;
    }
    //check if buffer got fully occupied with this add action
    full_ = write_ == read_;
}

template <typename T,int SZ>
T* RingBuffer<T,SZ>::getLatest()
{
    //taking care of negative wrap around
    if(write_ == 0)
    {
        return &buffer[max_size_-1];
    }

    //return last written element
    return &buffer[write_-1];
}

template <typename T,int SZ>
T* RingBuffer<T,SZ>::getSecondLatest()
{
    //taking care of negative wrap around
    if(write_ == 0)
    {
        return &buffer[max_size_-2];
    }
    if(write_ == 1)
    {
        return &buffer[max_size_-1];
    }

    //return last written element
    return &buffer[write_-2];
}

//TODO function for getting last/oldest element, deleting elements or consuming read

#endif /* dataStructures_h */
