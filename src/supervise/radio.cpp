#include "supervise/radio.hpp"
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <vector>

namespace fpvd {

static void readAll(int fd, std::string& dst) {
    char buf[1024];
    ssize_t n;
    while ((n = ::read(fd, buf, sizeof(buf))) > 0) {
        dst.append(buf, buf + n);
    }
}

RadioResult bringUpRadio(const std::string& scriptPath, const Config& c) {
    RadioResult r{};
    int outPipe[2], errPipe[2];
    if (::pipe(outPipe) < 0 || ::pipe(errPipe) < 0) {
        r.ok = false; r.exitCode = -1; return r;
    }
    pid_t pid = ::fork();
    if (pid == 0) {
        ::dup2(outPipe[1], 1);
        ::dup2(errPipe[1], 2);
        ::close(outPipe[0]); ::close(outPipe[1]);
        ::close(errPipe[0]); ::close(errPipe[1]);
        setenv("FPVD_CHANNEL", std::to_string(c.link.channel).c_str(), 1);
        setenv("FPVD_WIDTH",   std::to_string(c.link.width).c_str(),   1);
        setenv("FPVD_TXPOWER", std::to_string(c.link.txpower).c_str(), 1);
        setenv("FPVD_MTU",     std::to_string(c.link.mtu).c_str(),     1);
        if (c.link.wlanAdapter)
            setenv("FPVD_WLAN_ADAPTER", c.link.wlanAdapter->c_str(), 1);
        ::execl(scriptPath.c_str(), scriptPath.c_str(), nullptr);
        _exit(127);
    }
    ::close(outPipe[1]); ::close(errPipe[1]);

    std::string stdoutBuf;
    readAll(outPipe[0], stdoutBuf);
    readAll(errPipe[0], r.stderrText);
    ::close(outPipe[0]); ::close(errPipe[0]);

    int status = 0;
    ::waitpid(pid, &status, 0);
    r.exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : 128;
    r.ok = (r.exitCode == 0);

    std::istringstream is(stdoutBuf);
    std::string line;
    while (std::getline(is, line)) {
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        auto k = line.substr(0, eq);
        auto v = line.substr(eq + 1);
        if (k == "driver") r.driver = v;
        else if (k == "iface") r.iface = v;
        else if (k == "adapter_id" && !v.empty()) r.adapterId = v;
    }
    return r;
}

} // namespace fpvd
