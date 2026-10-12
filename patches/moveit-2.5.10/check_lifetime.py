"""Exercise the actual patched cleanup and upstream callback-lock methods without ROS/Qt."""
from pathlib import Path
import subprocess
import resource
import sys

root = Path(sys.argv[1]).resolve()
# The unpatched negative control intentionally aborts; do not create a core dump.
resource.setrlimit(resource.RLIMIT_CORE, (0, resource.getrlimit(resource.RLIMIT_CORE)[1]))
source = root / 'moveit2-2.5.10/moveit_ros'
display = (source / 'visualization/planning_scene_rviz_plugin/src/planning_scene_display.cpp').read_text()
monitor = (source / 'planning/planning_scene_monitor/src/planning_scene_monitor.cpp').read_text()

def method(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

cleanup = method(display, 'void PlanningSceneDisplay::clearRobotModel()')
destructor = method(display, 'PlanningSceneDisplay::~PlanningSceneDisplay()')
# Both destruction paths must synchronize before resetting the stored monitor.
assert destructor.index('clearUpdateCallbacks()') < destructor.index('planning_scene_monitor_.reset()')
callbacks = '\n'.join(method(monitor, signature) for signature in (
    'void PlanningSceneMonitor::clearUpdateCallbacks()',
    'void PlanningSceneMonitor::triggerSceneUpdateEvent(SceneUpdateType update_type)',
))
harness = r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
using namespace std::chrono_literals;

// Instrument the lock boundary so the test controls the conflicting interleaving.
struct ObservedMutex {
  std::recursive_mutex mutex;
  std::atomic<bool> observe{false};
  std::promise<void> attempted;
  void lock() {
    if (observe.exchange(false)) attempted.set_value();
    mutex.lock();
  }
  void unlock() { mutex.unlock(); }
};
struct PlanningSceneMonitor {
  enum SceneUpdateType { UPDATE_NONE = 0, UPDATE_STATE = 1 };
  ObservedMutex update_lock_;
  std::vector<std::function<void(SceneUpdateType)>> update_callbacks_;
  SceneUpdateType new_scene_update_ = UPDATE_NONE;
  std::condition_variable_any new_scene_update_condition_;
  void clearUpdateCallbacks();
  void triggerSceneUpdateEvent(SceneUpdateType update_type);
};
struct PlanningSceneDisplay {
  std::shared_ptr<int> planning_scene_render_ = std::make_shared<int>(1);
  std::shared_ptr<PlanningSceneMonitor> planning_scene_monitor_;
  void clearRobotModel();
};
'''
main = r'''
int main() {
  for (int iteration = 0; iteration < 100; ++iteration) {
    PlanningSceneDisplay display;
    auto monitor = std::make_shared<PlanningSceneMonitor>();
    display.planning_scene_monitor_ = monitor;
    std::promise<void> entered, release;
    auto released = release.get_future();
    std::atomic<int> calls{0};
    monitor->update_callbacks_.push_back([&](auto) {
      ++calls;
      entered.set_value();
      released.wait();
      // This is the stored pointer dereferenced by RViz's scene update callback.
      assert(display.planning_scene_monitor_ == monitor);
      assert(display.planning_scene_render_);
    });
    std::thread incoming([&] { monitor->triggerSceneUpdateEvent(PlanningSceneMonitor::UPDATE_STATE); });
    entered.get_future().wait();
    monitor->update_lock_.observe = true;
    auto clearing = std::async(std::launch::async, [&] { display.clearRobotModel(); });
    assert(monitor->update_lock_.attempted.get_future().wait_for(2s) == std::future_status::ready);
    assert(clearing.wait_for(0s) == std::future_status::timeout);
    release.set_value();
    incoming.join();
    clearing.get();
    assert(!display.planning_scene_monitor_ && !display.planning_scene_render_);
    // A late update is harmless after cleanup, including an already queued message.
    monitor->triggerSceneUpdateEvent(PlanningSceneMonitor::UPDATE_STATE);
    assert(calls == 1);
    display.clearRobotModel();  // Empty cleanup is safe too.
  }
}
'''
for name, code in [('patched', cleanup), ('original', cleanup.replace(
    '  if (planning_scene_monitor_)\n    planning_scene_monitor_->clearUpdateCallbacks();\n', ''))]:
    cpp = root / f'lifetime_{name}.cpp'
    exe = root / f'lifetime_{name}'
    cpp.write_text(harness + callbacks + '\n' + code + '\n' + main)
    subprocess.run(['g++', '-std=c++17', '-O1', '-g', '-pthread', str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], timeout=15, capture_output=True)
    if name == 'patched':
        assert result.returncode == 0, result.stderr.decode()
        print('Patched cleanup: 100 controlled callback/reload interleavings passed')
    else:
        assert result.returncode != 0
        assert b'Assertion' in result.stderr
        print('Original cleanup: regression harness detects missing callback synchronization')
