#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

class V4L2Camera {
public:
    V4L2Camera(const std::string& devicePath, int width, int height, int fps) {
        // 打开设备
        fd = open(devicePath.c_str(), O_RDWR);
        if (fd == -1) {
            throw std::runtime_error("Failed to open camera device.");
        }

        // 设置格式和分辨率
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // 或其他支持的格式
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
            throw std::runtime_error("Failed to set camera format.");
        }

        // 请求缓冲区
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
            throw std::runtime_error("Failed to request buffers.");
        }

        // 映射缓冲区
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            throw std::runtime_error("Failed to query buffer.");
        }

        void* buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffer == MAP_FAILED) {
            throw std::runtime_error("Failed to map buffer.");
        }

        // 开始捕获
        if (ioctl(fd, VIDIOC_STREAMON, &buf.type) == -1) {
            throw std::runtime_error("Failed to start capture.");
        }
    }

    void read(cv::Mat& img) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            throw std::runtime_error("Failed to dequeue buffer.");
        }

        // 处理图像数据
        // ...

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            throw std::runtime_error("Failed to queue buffer.");
        }
    }

    ~V4L2Camera() {
        // 停止捕获
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_STREAMOFF, &buf.type) == -1) {
            std::cerr << "Failed to stop capture." << std::endl;
        }

        // 释放映射的缓冲区
        munmap(buffer, buf.length);

        // 关闭设备
        close(fd);
    }

private:
    int fd;
    void* buffer;
};