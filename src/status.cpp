#include "status.hpp"
#include <chrono>

namespace fpvd {

static const char* stateName(ProcState s) {
    switch (s) {
        case ProcState::Stopped:  return "stopped";
        case ProcState::Starting: return "starting";
        case ProcState::Running:  return "running";
        case ProcState::Exited:   return "exited";
        case ProcState::Failed:   return "failed";
    }
    return "unknown";
}

nlohmann::json buildStatus(Daemon& d) {
    using namespace std::chrono;
    auto uptimeSec = duration_cast<seconds>(
        steady_clock::now() - d.startedAt()).count();

    nlohmann::json procs = nlohmann::json::array();
    for (auto& name : d.orchestrator().names()) {
        auto* s = d.orchestrator().get(name);
        if (!s) continue;
        nlohmann::json p = {
            {"name", name},
            {"pid", s->pid()},
            {"state", stateName(s->state())},
            {"restarts", s->restartCount()},
            {"lastExitCode", s->lastExitCode().has_value()
                              ? nlohmann::json(s->lastExitCode().value())
                              : nlohmann::json(nullptr)}
        };
        procs.push_back(p);
    }

    nlohmann::json last;
    if (d.lastApply().at.empty()) {
        last = nullptr;
    } else {
        last = {
            {"at", d.lastApply().at},
            {"ok", d.lastApply().ok},
            {"restarted", d.lastApply().restarted},
            {"error", d.lastApply().error.has_value()
                       ? nlohmann::json(d.lastApply().error.value())
                       : nlohmann::json(nullptr)}
        };
    }

    return {
        {"uptime", uptimeSec},
        {"version", d.version()},
        {"lastApply", last},
        {"radio", {
            {"driver", d.radio().driver},
            {"iface", d.radio().iface},
            {"adapterId", d.radio().adapterId.has_value()
                           ? nlohmann::json(d.radio().adapterId.value())
                           : nlohmann::json(nullptr)}
        }},
        {"processes", procs}
    };
}

} // namespace fpvd
