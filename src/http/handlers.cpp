#include "http/handlers.hpp"

namespace fpvd {

static nlohmann::json errBody(const std::string& code, const std::string& msg,
                               const nlohmann::json& details = nullptr) {
    nlohmann::json j = {{"error", code}, {"message", msg}};
    if (!details.is_null()) j["details"] = details;
    return j;
}

void registerHandlers(HttpServer& srv, Daemon& d, bool /*reallyRestart*/) {
    srv.get("/healthz", [](const httplib::Request&, httplib::Response& res){
        res.status = 200; res.set_content("{}", "application/json");
    });

    srv.get("/config", [&](const httplib::Request& req, httplib::Response& res){
        bool pending = req.has_param("pending") && req.get_param_value("pending") == "true";
        nlohmann::json j = pending ? nlohmann::json(d.pending())
                                    : nlohmann::json(d.effective());
        res.set_content(j.dump(), "application/json");
    });

    srv.get("/defaults", [&](const httplib::Request&, httplib::Response& res){
        res.set_content(d.defaultsJson().dump(), "application/json");
    });

    srv.patch("/config", [&](const httplib::Request& req, httplib::Response& res){
        nlohmann::json body;
        try { body = nlohmann::json::parse(req.body); }
        catch (const nlohmann::json::exception&) {
            res.status = 400;
            res.set_content(errBody("bad_json", "request body not valid JSON").dump(),
                            "application/json");
            return;
        }
        auto pr = d.patchPending(body);
        if (!pr.ok) {
            nlohmann::json details = nlohmann::json::array();
            for (auto& e : pr.errors)
                details.push_back({{"path", e.path}, {"message", e.message}});
            res.status = 400;
            res.set_content(errBody("validation", "schema validation failed", details).dump(),
                            "application/json");
            return;
        }
        res.set_content(nlohmann::json(d.pending()).dump(), "application/json");
    });
}

} // namespace fpvd
