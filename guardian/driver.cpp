// driver_guardian: SteamVR driver that runs camserver's loop (camserver.cpp) on a thread inside vrserver. vrserver is
// XRService's parent, so Yama (ptrace_scope=1) lets it pidfd_getfd XRService's camera fds without root or
// CAP_SYS_PTRACE. Adds no devices. Log: vrserver.txt. guardian.py registers it with vrpathreg on the app's first run.
#include <openvr_driver.h>

#include <cstring>
#include <filesystem>
#include <string>
#include <thread>

#include "shm.h"

int camserver_run(const std::string (&nodes)[shm::kNumCams]);
void camserver_stop();
extern void (*camserver_log)(const char* line);

namespace {

class Provider : public vr::IServerTrackedDeviceProvider {
  public:
    vr::EVRInitError Init(vr::IVRDriverContext* ctx) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
        camserver_log = [](const char* line) {
            vr::VRDriverLog()->Log(line);
        };
        // guardian runs in the Steam Runtime container, which doesn't see /persist: copy the factory calibration next
        // to the frames, for calib.py.
        std::error_code ec;
        std::filesystem::create_directories(shm::kCalibDir, ec);
        for (const char* f : {"xrservice.json", "device_config.json"}) {
            std::filesystem::copy_file(std::string("/persist/") + f, std::string(shm::kCalibDir) + "/" + f,
                                       std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) {
                vr::VRDriverLog()->Log(("guardian: copy /persist/" + std::string(f) + ": " + ec.message()).c_str());
            }
        }
        thread_ = std::thread([] {
            camserver_run({"/dev/video3", "/dev/video0"});
        });
        return vr::VRInitError_None;
    }
    void Cleanup() override {
        camserver_stop();
        if (thread_.joinable()) {
            thread_.join();
        }
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }
    const char* const* GetInterfaceVersions() override {
        return vr::k_InterfaceVersions;
    }
    void RunFrame() override {
    }
    bool ShouldBlockStandbyMode() override {
        return false;
    }
    void EnterStandby() override {
    }
    void LeaveStandby() override {
    }

  private:
    std::thread thread_;
};

Provider g_provider;

}  // namespace

extern "C" __attribute__((visibility("default"))) void* HmdDriverFactory(const char* iface, int* err) {
    if (!strcmp(iface, vr::IServerTrackedDeviceProvider_Version)) {
        return &g_provider;
    }
    if (err) {
        *err = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
