#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>

namespace aurora {

class ThreadPool {
public:
    explicit ThreadPool(std::size_t n_threads = std::thread::hardware_concurrency()) {
        for (std::size_t i = 0; i < n_threads; ++i) {
            workers_.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mu_);
                        cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                        if (stop_ && tasks_.empty()) return;
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                }
            });
        }
    }

    // 提交任务，返回 std::future
    template <class F, class... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>
    {
        using R = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<R()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        std::future<R> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(mu_);
            if (stop_) throw std::runtime_error("ThreadPool: submit after shutdown");
            tasks_.emplace([task] { (*task)(); });
        }
        cv_.notify_one();
        return res;
    }

    std::size_t size() const { return workers_.size(); }

    ~ThreadPool() {
        { std::unique_lock<std::mutex> lock(mu_); stop_ = true; }
        cv_.notify_all();
        for (auto& w : workers_) w.join();
    }

    // 禁止拷贝
    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

private:
    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex                        mu_;
    std::condition_variable           cv_;
    bool                              stop_ = false;
};

// 全局单例，按需初始化
inline ThreadPool& global_thread_pool(std::size_t n = 0) {
    static ThreadPool pool(n ? n : std::thread::hardware_concurrency());
    return pool;
}

} // namespace aurora
