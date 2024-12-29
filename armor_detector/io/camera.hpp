#ifndef IO_CAMERA__HPP
#define IO_CAMERA__HPP

#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

namespace io
{
    class Camera
    {
    public:
        Camera(int cameraIndex, int width, int height, int fps);
        ~Camera();
        bool read(cv::Mat &img);

    private:
        int fd; // 文件描述符
        struct v4l2_format fmt;
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        void* buffer; // 映射的缓冲区
        unsigned int n_buffers;
    };
} // namespace io

#endif // IO_CAMERA__HPP