#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <iostream>

class Timer {
public:
    Timer() : fd(-1) {}
    ~Timer() {
        if (fd != -1) {
            close(fd);
        }
    }

    bool start(int interval) {
        fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if (fd == -1) {
            std::cerr << "Failed to create timer" << std::endl;
            return false;
        }

        struct itimerspec ts;
        ts.it_interval.tv_sec = 0;
        ts.it_interval.tv_nsec = interval;
        ts.it_value.tv_sec = 1;
        ts.it_value.tv_nsec = 0;

        if (timerfd_settime(fd, 0, &ts, nullptr) == -1) {
            std::cerr << "Failed to set timer" << std::endl;
            close(fd);
            fd = -1;
            return false;
        }

        epoll_fd = epoll_create1(0);
        if (epoll_fd == -1) {
            std::cerr << "Failed to create epoll" << std::endl;
            close(fd);
            fd = -1;
            return false;
        }

        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = fd;

        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) == -1) {
            std::cerr << "Failed to add timer to epoll" << std::endl;
            close(epoll_fd);
            close(fd);
            fd = -1;
            return false;
        }

        return true;
    }

    bool wait() {
        if (fd == -1) {
            return false;
        }

        struct epoll_event events[1];
        int n = epoll_wait(epoll_fd, events, 1, -1);
        if (n == -1) {
            std::cerr << "Failed to wait for timer event" << std::endl;
            return false;
        }

        uint64_t expirations = 0;
        if (read(fd, &expirations, sizeof(expirations)) != sizeof(expirations)) {
            std::cerr << "Failed to read timer expirations" << std::endl;
            return false;
        }

        return true;
    }

private:
    int fd;
    int epoll_fd;
};
