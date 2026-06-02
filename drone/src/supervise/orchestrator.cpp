#include "supervise/orchestrator.hpp"
#include <algorithm>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace fpvd {

void Orchestrator::add(SupervisedSpec spec) {
    auto name = spec.name;
    specs_[name] = spec;
    sups_[name] = std::make_unique<Supervisor>(std::move(spec));
}

void Orchestrator::remove(const std::string& name) {
    auto it = sups_.find(name);
    if (it == sups_.end()) return;
    it->second->shutdown();
    sups_.erase(it);
    specs_.erase(name);
}

void Orchestrator::restart(const std::string& name,
                           std::chrono::milliseconds settle) {
    auto it = sups_.find(name);
    if (it == sups_.end()) return;
    it->second->shutdown();   // SIGTERM + join (no reinit flag -> no self-respawn)
    if (settle.count() > 0)   // let the old process's kernel/driver state drain
        std::this_thread::sleep_for(settle);
    it->second->start();      // fresh supervision loop
}

Supervisor* Orchestrator::get(const std::string& name) {
    auto it = sups_.find(name);
    return it == sups_.end() ? nullptr : it->second.get();
}

std::vector<std::string> Orchestrator::startOrder() const {
    // Kahn's algorithm.
    std::unordered_map<std::string, int> indeg;
    std::unordered_map<std::string, std::vector<std::string>> revDeps;
    for (auto& [n, s] : specs_) indeg[n] = 0;
    for (auto& [n, s] : specs_) {
        for (auto& dep : s.startAfter) {
            if (specs_.count(dep)) {
                indeg[n]++;
                revDeps[dep].push_back(n);
            }
        }
    }
    std::vector<std::string> queue;
    for (auto& [n, d] : indeg) if (d == 0) queue.push_back(n);
    std::sort(queue.begin(), queue.end());  // stable order across runs
    std::vector<std::string> out;
    for (size_t i = 0; i < queue.size(); ++i) {
        auto& n = queue[i];
        out.push_back(n);
        for (auto& m : revDeps[n]) {
            if (--indeg[m] == 0) queue.push_back(m);
        }
    }
    if (out.size() != specs_.size())
        throw OrchestrationError("startAfter cycle detected");
    return out;
}

std::vector<std::string> Orchestrator::stopOrder() const {
    auto o = startOrder();
    std::reverse(o.begin(), o.end());
    return o;
}

void Orchestrator::startAll() {
    for (auto& n : startOrder()) sups_[n]->start();
}

void Orchestrator::stopAll() {
    for (auto& n : stopOrder()) sups_[n]->shutdown();
}

std::vector<std::string> Orchestrator::names() const {
    std::vector<std::string> out;
    for (auto& [n, _] : sups_) out.push_back(n);
    return out;
}

} // namespace fpvd
