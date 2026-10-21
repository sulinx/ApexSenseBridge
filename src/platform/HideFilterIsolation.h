#pragma once

// Client for the HideFilter kernel lower-filter driver described by
// VykosX/rewasd-xinput-hide-reverse. Unlike HidHide, which is a HIDClass
// upper filter and therefore never reaches the XnaComposite/xusb22 stack that
// provides the physical XInput node, this driver is registered as a
// LowerFilter on the device classes themselves and exposes one device interface
// per filtered physical device (one per PDO). Hiding therefore removes the node
// from every user-mode enumeration at once: XInput, GameInput, DirectInput,
// RawInput and HID.
//
// The wire contract is taken verbatim from the project's reference/protocol.md
// and src/driver/HideFilter.h so a stock HideFilter build can be driven without
// modifying it.

#include "core/DeviceInfo.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace asb::platform {

// One filter instance discovered on the system. Every entry corresponds to a
// single physical device stack, so `parentInstanceId` names the device that
// would be hidden by sending the IOCTL against `interfacePath`.
struct HideFilterInstance {
    std::wstring interfacePath;
    std::wstring parentInstanceId;
    std::wstring parentService;
    std::wstring parentClass;
    bool opened = false;
    std::string error;
};

// RAII handle that keeps one physical device hidden for as long as it lives.
//
// Lifetime semantics come from the driver's per-file-handle reference count:
// destroying this object closes the handle, and the kernel decrements the count
// itself. A crashed bridge therefore cannot leave the controller hidden, which
// is exactly the guarantee the pnputil-based path could not provide.
class HideFilterIsolation {
public:
    HideFilterIsolation();
    ~HideFilterIsolation();

    HideFilterIsolation(const HideFilterIsolation&) = delete;
    HideFilterIsolation& operator=(const HideFilterIsolation&) = delete;

    // Enumerates every filter instance currently attached, with the parent
    // device identity for each. Read-only; safe to call at any time.
    static std::vector<HideFilterInstance> enumerate();

    // Reports whether any HideFilter instance is present, i.e. whether the
    // driver is installed and attached to at least one device stack.
    static bool available();

    // Opens the filter instance whose underlying physical device matches
    // `target` (by device-interface path or PnP instance id, including the
    // parent of that node) and hides it. The handle stays open until restore().
    bool activate(const HidDeviceInfo& target, std::string& error);

    // Re-sends an explicit "show" and then closes the handle. Idempotent.
    bool restore(std::string& error) noexcept;

    [[nodiscard]] bool active() const noexcept;
    [[nodiscard]] std::wstring hiddenInstanceId() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

namespace detail {

// Interface GUID published by every HideFilter instance, verbatim from
// HideFilter.h. The upstream file marks this value as a placeholder to be
// replaced before shipping; a downstream build that regenerates it must supply
// the same GUID here.
[[nodiscard]] const void* hideFilterInterfaceGuid() noexcept;

// CTL_CODE(0x002A, 0xA78, METHOD_BUFFERED, FILE_READ_DATA|FILE_WRITE_DATA)
// == 0x002AE9E0.
[[nodiscard]] std::uint32_t hideFilterTogglePresenceIoctl() noexcept;

// The driver's input convention: a single byte, 0 hides and any other value
// shows. Kept separate so the polarity can be unit-tested without a driver.
[[nodiscard]] std::uint8_t hideFilterPresenceByte(bool hide) noexcept;

}  // namespace detail

}  // namespace asb::platform
