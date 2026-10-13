// vNext M2.6 test harness -- a production executable run as a CHILD PROCESS (trainit_run_bt,
// scene_manager_node), the way a launch file starts it: its own process, a params file, an
// optional `-r __ns:=` remap. The child inherits the private domain and ROS_LOCALHOST_ONLY from
// the environment the parent sets IN CODE (m26::runAll, re-set here explicitly), writes stdout
// and stderr to a file in the build tree, keeps its ROS log there too, and dies with the parent
// (PR_SET_PDEATHSIG) so a killed test never leaves a node behind.
#ifndef TRAINIT_MOTION_RUNTIME_TEST_M2_6_CHILD_PROCESS_HPP_
#define TRAINIT_MOTION_RUNTIME_TEST_M2_6_CHILD_PROCESS_HPP_

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "capture26.hpp"

extern char** environ;

namespace m26
{

class ChildProcess
{
public:
  ChildProcess(const std::string& exe, const std::vector<std::string>& args,
               const std::map<std::string, std::string>& env, const std::string& output_file)
  : output_(output_file)
  {
    // everything the child needs is built BEFORE fork(): after it only async-signal-safe calls
    std::vector<std::string> argv_s{exe};
    argv_s.insert(argv_s.end(), args.begin(), args.end());
    std::vector<std::string> env_s;
    for (char** e = environ; e && *e; ++e)
    {
      const std::string kv(*e);
      if (env.count(kv.substr(0, kv.find('='))) == 0) env_s.push_back(kv);
    }
    for (const auto& [k, v] : env) env_s.push_back(k + "=" + v);
    std::vector<char*> argv, envp;
    for (auto& a : argv_s) argv.push_back(a.data());
    argv.push_back(nullptr);
    for (auto& a : env_s) envp.push_back(a.data());
    envp.push_back(nullptr);
    const pid_t parent = ::getpid();
    pid_ = ::fork();
    if (pid_ == 0)
    {
      ::prctl(PR_SET_PDEATHSIG, SIGKILL);
      if (::getppid() != parent) ::_exit(127);
      const int fd = ::open(output_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (fd >= 0) { ::dup2(fd, STDOUT_FILENO); ::dup2(fd, STDERR_FILENO); ::close(fd); }
      const int in = ::open("/dev/null", O_RDONLY);
      if (in >= 0) { ::dup2(in, STDIN_FILENO); ::close(in); }
      ::execve(argv[0], argv.data(), envp.data());
      ::_exit(127);
    }
  }
  ~ChildProcess()
  {
    if (running())
    {
      ::kill(pid_, SIGKILL);
      ::waitpid(pid_, &status_, 0);
    }
  }
  ChildProcess(const ChildProcess&) = delete;
  ChildProcess& operator=(const ChildProcess&) = delete;

  bool started() const { return pid_ > 0; }
  bool running()
  {
    if (done_ || pid_ <= 0) return false;
    int st = 0;
    const pid_t r = ::waitpid(pid_, &st, WNOHANG);
    if (r == pid_) { done_ = true; status_ = st; return false; }
    return r == 0;
  }
  // Waits up to `budget` for the child to exit; returns whether it did.
  bool wait(milliseconds budget) { return waitUntil([this] { return !running(); }, budget); }
  void signal(int sig)
  {
    if (running()) ::kill(pid_, sig);
  }
  // Ends it the way a launch does on Ctrl-C: SIGINT, then SIGKILL if it hangs.
  void stop(milliseconds grace = milliseconds(10000))
  {
    if (!running()) return;
    signal(SIGINT);
    if (!wait(grace))
    {
      hung_ = true;
      signal(SIGKILL);
      wait(milliseconds(5000));
    }
  }
  std::string exitText() const
  {
    if (!done_) return "still running";
    std::string s = WIFEXITED(status_) ? "exit code " + std::to_string(WEXITSTATUS(status_))
                    : WIFSIGNALED(status_) ? "killed by signal " + std::to_string(WTERMSIG(status_))
                                           : std::string("?");
    return hung_ ? s + " (did not exit within the grace period after SIGINT)" : s;
  }
  // The documented last line of trainit_run_bt ("BT finished: <status>"); "" when absent.
  std::string btFinished() const
  {
    std::ifstream in(output_);
    std::string line, found;
    const std::string key = "BT finished: ";
    while (std::getline(in, line))
    {
      const auto at = line.find(key);
      if (at != std::string::npos) found = line.substr(at + key.size());
    }
    while (!found.empty() && (found.back() == '\r' || found.back() == ' ')) found.pop_back();
    return found;
  }
  const std::string& output() const { return output_; }

private:
  std::string output_;
  pid_t pid_{-1};
  int status_{0};
  bool done_{false};
  bool hung_{false};
};

// The environment of every child: the private domain and localhost-only (the parent set them in
// code; set again here so nothing inherited can override them), the rmw, and the ROS log directory
// in the build tree (no file under ~/.ros/log).
inline std::map<std::string, std::string> childEnvironment(std::size_t domain, const std::string& log_dir)
{
  return {{"ROS_DOMAIN_ID", std::to_string(domain)},
          {"ROS_LOCALHOST_ONLY", "1"},
          {"RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"},
          {"ROS_LOG_DIR", log_dir},
          {"RCUTILS_COLORIZED_OUTPUT", "0"}};
}

inline void writeFile(const std::string& path, const std::string& text)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
}

// Indents every line of `text` by `n` spaces (a YAML literal block scalar).
inline std::string indentBlock(const std::string& text, int n)
{
  std::istringstream in(text);
  std::ostringstream out;
  std::string line;
  const std::string pad(static_cast<std::size_t>(n), ' ');
  while (std::getline(in, line)) out << pad << line << "\n";
  return out.str();
}

}  // namespace m26

#endif  // TRAINIT_MOTION_RUNTIME_TEST_M2_6_CHILD_PROCESS_HPP_
