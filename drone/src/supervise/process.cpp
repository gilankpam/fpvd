#include "supervise/process.hpp"
#include <cstring>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace fpvd {

Process::Process(std::vector<std::string> argv, std::map<std::string, std::string> env)
    : argv_(std::move(argv)), env_(std::move(env)) {}

void Process::start() {
    pid_t p = fork();
    if (p < 0) {
        state_ = ProcState::Failed;
        return;
    }
    if (p == 0) {
        // Child: new process group so we can signal all descendants.
        setpgid(0, 0);
        for (auto& kv : env_) {
            setenv(kv.first.c_str(), kv.second.c_str(), 1);
        }
        std::vector<char*> cargs;
        for (auto& s : argv_)
            cargs.push_back(const_cast<char*>(s.c_str()));
        cargs.push_back(nullptr);
        execvp(cargs[0], cargs.data());
        _exit(127);
    }
    pid_ = p;
    state_ = ProcState::Running;
}

void Process::stop(std::chrono::milliseconds grace) {
    if (state_ != ProcState::Running)
        return;
    ::killpg(pid_, SIGTERM);
    auto deadline = std::chrono::steady_clock::now() + grace;
    while (std::chrono::steady_clock::now() < deadline) {
        if (reapIfReady())
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ::killpg(pid_, SIGKILL);
}

bool Process::reapIfReady() {
    if (pid_ <= 0)
        return false;
    int status = 0;
    pid_t r = ::waitpid(pid_, &status, WNOHANG);
    if (r == 0)
        return false;
    if (r < 0) {
        state_ = ProcState::Exited;
        return true;
    }
    if (WIFEXITED(status))
        lastExitCode_ = WEXITSTATUS(status);
    else if (WIFSIGNALED(status))
        lastExitCode_ = 128 + WTERMSIG(status);
    state_ = ProcState::Exited;
    return true;
}

bool Process::waitFor(std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (reapIfReady())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

} // namespace fpvd
