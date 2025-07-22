// TebDebugLogger.h
#pragma once
#include <fstream>
#include <mutex>
#include <string>
#include <sstream>
#include <iomanip>

class TebDebugLogger {
public:
  explicit TebDebugLogger(const std::string& path)
  : ofs_(path, std::ios::out | std::ios::trunc) {}

  bool good() const { return ofs_.good(); }

  void logRawJson(const std::string& json_line) {
    std::lock_guard<std::mutex> lk(m_);
    ofs_ << json_line << "\n";
  }

  // convenience: build {"k":"v",...}
  template <typename Fn>
  void logObject(Fn fn) {
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    auto add = [&](const std::string& k, const std::string& v, bool quote=true){
      if(!first) oss << ","; first=false;
      oss << "\"" << k << "\":";
      if (quote) oss << "\"" << v << "\"";
      else oss << v;
    };
    fn(add);
    oss << "}";
    logRawJson(oss.str());
  }

private:
  std::ofstream ofs_;
  std::mutex m_;
};
