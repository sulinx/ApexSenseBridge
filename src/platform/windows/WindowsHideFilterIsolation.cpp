// Windows client for the HideFilter lower-filter driver.
//
// The enumeration order below mirrors src/client/hide-example.c from
// VykosX/rewasd-xinput-hide-reverse:
//   SetupDiGetClassDevsW(GUID_DEVINTERFACE_HIDEFILTER, DIGCF_PRESENT|DIGCF_DEVICEINTERFACE)
//     -> SetupDiEnumDeviceInterfaces
//     -> SetupDiGetDeviceInterfaceDetailW
//     -> CM_Get_Parent + CM_Get_Device_IDW        (which device this filter sits on)
//     -> CreateFileW(GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE)
//     -> DeviceIoControl(IOCTL_HIDEFILTER_TOGGLE_PRESENCE, {0})
//
// Two properties of that contract drive the design here:
//   * One interface per filtered physical device. Selecting "the controller"
//     means matching the parent instance id, never the interface path.
//   * The reference count is per file handle, so simply closing our handle
//     un-hides the device. There is no persistent configuration to restore and
//     nothing that survives a crash.

#include "platform/HideFilterIsolation.h"

#include <windows.h>

#include <cfgmgr32.h>
#include <devpkey.h>
#include <setupapi.h>
#include <initguid.h>

#include <algorithm>
#include <cwctype>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "cfgmgr32.lib")

namespace asb::platform {
namespace {

// {A1B2C3D4-5E6F-7081-92A3-B4C5D6E7F809} -- the placeholder published by
// HideFilter.h. Replace together with a rebuilt driver if the vendor GUID is
// rotated.
DEFINE_GUID(GUID_DEVINTERFACE_HIDEFILTER_LOCAL,
            0xA1B2C3D4, 0x5E6F, 0x7081,
            0x92, 0xA3, 0xB4, 0xC5, 0xD6, 0xE7, 0xF8, 0x09);

constexpr std::uint16_t kHideFilterDeviceType = 0x002A;
constexpr std::uint16_t kHideFilterFunction = 0xA78;

std::string windowsErrorText(std::string_view operation, DWORD code) {
    char* buffer = nullptr;
    const DWORD chars = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, code, 0, reinterpret_cast<char*>(&buffer), 0, nullptr);
    std::string message;
    if (chars != 0 && buffer != nullptr) {
        message.assign(buffer, chars);
        while (!message.empty() &&
               (message.back() == '\r' || message.back() == '\n')) {
            message.pop_back();
        }
    }
    if (buffer != nullptr) {
        LocalFree(buffer);
    }
    std::string text(operation);
    text += " failed with Win32 error " + std::to_string(code);
    if (!message.empty()) {
        text += ": " + message;
    }
    return text;
}

std::wstring wide(std::string_view value) {
    return std::wstring(value.begin(), value.end());
}

// PnP instance ids are ASCII by specification, so the error text does not need a
// full UTF-8 conversion; anything outside that range is reported as '?'.
std::string narrow(const std::wstring& value) {
    std::string text;
    text.reserve(value.size());
    for (const wchar_t character : value) {
        text.push_back(character > 0x20 && character < 0x7f
                          ? static_cast<char>(character)
                          : '?');
    }
    return text;
}

bool equalsIgnoreCase(const std::wstring& left, const std::wstring& right) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t i = 0; i < left.size(); ++i) {
        if (std::towlower(left[i]) != std::towlower(right[i])) {
            return false;
        }
    }
    return true;
}

std::wstring nodeInstanceId(DEVINST node) {
    ULONG length = 0;
    if (CM_Get_Device_ID_Size(&length, node, 0) != CR_SUCCESS) {
        return {};
    }
    std::vector<wchar_t> buffer(length + 1, L'\0');
    if (CM_Get_Device_IDW(node, buffer.data(), length + 1, 0) != CR_SUCCESS) {
        return {};
    }
    return std::wstring(buffer.data());
}

std::wstring nodeStringProperty(DEVINST node, const DEVPROPKEY& property) {
    ULONG bytes = 0;
    if (CM_Get_DevNode_PropertyW(node, &property, nullptr, nullptr, &bytes, 0) !=
        CR_SUCCESS) {
        return {};
    }
    std::vector<wchar_t> buffer(bytes / sizeof(wchar_t) + 1, L'\0');
    DEVPROPTYPE type = 0;
    if (CM_Get_DevNode_PropertyW(node, &property, &type,
                                 reinterpret_cast<BYTE*>(buffer.data()), &bytes,
                                 0) != CR_SUCCESS) {
        return {};
    }
    return std::wstring(buffer.data());
}

// Reads the device-interface detail for one enumerated filter instance and
// resolves the device that filter is attached to.
bool describeFilterInstance(HDEVINFO devices, SP_DEVICE_INTERFACE_DATA& interfaceData,
                            HideFilterInstance& out) {
    DWORD required = 0;
    SetupDiGetDeviceInterfaceDetailW(devices, &interfaceData, nullptr, 0,
                                    &required, nullptr);
    const DWORD detailError = GetLastError();
    if (required == 0 && detailError != ERROR_INSUFFICIENT_BUFFER) {
        out.error = "Sizing a HideFilter interface failed with Win32 error " +
                    std::to_string(detailError);
        return false;
    }
    std::vector<BYTE> storage(required, 0);
    auto* detail =
        reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(storage.data());
    detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
    SP_DEVINFO_DATA info = {};
    info.cbSize = sizeof(info);
    if (!SetupDiGetDeviceInterfaceDetailW(devices, &interfaceData, detail,
                                          required, &required, &info)) {
        out.error = windowsErrorText("Reading a HideFilter interface",
                                     GetLastError());
        return false;
    }
    out.interfacePath = detail->DevicePath;

    // The filter's own devnode is a child of the device it sits below, exactly
    // as in the reference client. That parent is what HidNotifyPresence targets.
    DEVINST parent = 0;
    if (CM_Get_Parent(&parent, info.DevInst, 0) == CR_SUCCESS) {
        out.parentInstanceId = nodeInstanceId(parent);
        out.parentService = nodeStringProperty(parent, DEVPKEY_Device_Service);
        out.parentClass = nodeStringProperty(parent, DEVPKEY_Device_Class);
    }
    return true;
}

struct FilterInstanceList {
    HDEVINFO value = INVALID_HANDLE_VALUE;
    std::vector<SP_DEVICE_INTERFACE_DATA> entries;

    ~FilterInstanceList() {
        if (value != INVALID_HANDLE_VALUE) {
            SetupDiDestroyDeviceInfoList(value);
        }
    }
};

}  // namespace

namespace detail {

const void* hideFilterInterfaceGuid() noexcept {
    return &GUID_DEVINTERFACE_HIDEFILTER_LOCAL;
}

std::uint32_t hideFilterTogglePresenceIoctl() noexcept {
    // CTL_CODE(DeviceType, Function, METHOD_BUFFERED,
    //          FILE_READ_DATA | FILE_WRITE_DATA) == 0x002AE9E0
    return (static_cast<std::uint32_t>(kHideFilterDeviceType) << 16) |
           (static_cast<std::uint32_t>(FILE_READ_DATA | FILE_WRITE_DATA) << 14) |
           (static_cast<std::uint32_t>(kHideFilterFunction) << 2) |
           0 /* METHOD_BUFFERED */;
}

std::uint8_t hideFilterPresenceByte(bool hide) noexcept {
    // Documented input polarity: 0 hides, any other value shows.
    return hide ? 0 : 1;
}

}  // namespace detail

struct HideFilterIsolation::Impl {
    HANDLE handle = INVALID_HANDLE_VALUE;
    std::wstring instanceId;

    bool send(std::uint8_t value, std::string& error) {
        DWORD returned = 0;
        if (!DeviceIoControl(handle,
                            detail::hideFilterTogglePresenceIoctl(), &value,
                            sizeof(value), nullptr, 0, &returned, nullptr)) {
            const DWORD code = GetLastError();
            std::string text =
                windowsErrorText("Sending IOCTL_HIDEFILTER_TOGGLE_PRESENCE", code);
            if (code == ERROR_INVALID_FUNCTION) {
                text += "; the target device is not a HideFilter instance.";
            } else if (code == ERROR_INSUFFICIENT_BUFFER) {
                text += "; the driver requires a one-byte input buffer.";
            } else if (code == ERROR_NOT_SUPPORTED) {
                text += "; the driver could not resolve HIDCLASS!HidNotifyPresence, "
                        "which means its export-table resolver failed at load time.";
            }
            error = std::move(text);
            return false;
        }
        return true;
    }
};

HideFilterIsolation::HideFilterIsolation() : impl_(std::make_unique<Impl>()) {}

HideFilterIsolation::~HideFilterIsolation() {
    std::string ignored;
    restore(ignored);
}

std::vector<HideFilterInstance> HideFilterIsolation::enumerate() {
    std::vector<HideFilterInstance> found;
    FilterInstanceList list;
    list.value = SetupDiGetClassDevsW(
        reinterpret_cast<const GUID*>(&GUID_DEVINTERFACE_HIDEFILTER_LOCAL),
        nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (list.value == INVALID_HANDLE_VALUE) {
        HideFilterInstance entry;
        entry.error = "SetupDiGetClassDevs for the HideFilter interface failed "
                      "with Win32 error " +
                      std::to_string(GetLastError()) +
                      "; the driver is probably not installed.";
        found.push_back(std::move(entry));
        return found;
    }
    for (DWORD index = 0;; ++index) {
        SP_DEVICE_INTERFACE_DATA data = {};
        data.cbSize = sizeof(data);
        if (!SetupDiEnumDeviceInterfaces(list.value, nullptr,
                                         &GUID_DEVINTERFACE_HIDEFILTER_LOCAL,
                                         index, &data)) {
            break;
        }
        HideFilterInstance instance;
        if (describeFilterInstance(list.value, data, instance)) {
            found.push_back(std::move(instance));
        } else {
            found.push_back(std::move(instance));
        }
    }
    return found;
}

bool HideFilterIsolation::available() {
    for (const auto& instance : enumerate()) {
        if (!instance.interfacePath.empty()) {
            return true;
        }
    }
    return false;
}

bool HideFilterIsolation::activate(const HidDeviceInfo& target, std::string& error) {
    if (impl_->handle != INVALID_HANDLE_VALUE) {
        error = "A HideFilter isolation is already active.";
        return false;
    }
    if (target.path.empty() && target.instanceId.empty()) {
        error = "The target device has no path or instance id to match.";
        return false;
    }

    const auto instances = enumerate();
    if (instances.empty()) {
        error = "No HideFilter instances are present, so the lower-filter driver "
                "is not installed or is not attached to any device stack.";
        return false;
    }

    // Build the identity set for the requested device. The bridge normally knows
    // the vendor HID path, while the filter sits on the function node, so both
    // the node itself and its parent are accepted, matching how
    // TemporaryPhysicalControllerIsolation already collects MI_xx and the
    // composite parent together.
    std::vector<std::wstring> wanted;
    wanted.push_back(target.instanceId);
    wanted.push_back(target.parentInstanceId);
    wanted.erase(std::remove_if(wanted.begin(), wanted.end(),
                                [](const std::wstring& value) {
                                    return value.empty();
                                }),
                 wanted.end());
    if (wanted.empty()) {
        error = "The target device exposes no PnP instance id to match.";
        return false;
    }

    for (const auto& instance : instances) {
        if (instance.interfacePath.empty()) {
            continue;
        }
        const bool matched = std::any_of(
            wanted.begin(), wanted.end(), [&](const std::wstring& value) {
                return equalsIgnoreCase(instance.parentInstanceId, value);
            });
        if (!matched) {
            continue;
        }
        HANDLE handle = CreateFileW(instance.interfacePath.c_str(),
                                    GENERIC_READ | GENERIC_WRITE,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                                    OPEN_EXISTING, 0, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            error = "Opening the HideFilter instance for " +
                    narrow(instance.parentInstanceId) + " failed with Win32 error " +
                    std::to_string(GetLastError());
            continue;
        }
        impl_->handle = handle;
        if (!impl_->send(detail::hideFilterPresenceByte(true), error)) {
            CloseHandle(handle);
            impl_->handle = INVALID_HANDLE_VALUE;
            return false;
        }
        impl_->instanceId = instance.parentInstanceId;
        return true;
    }

    std::wstring known;
    for (const auto& instance : instances) {
        if (instance.parentInstanceId.empty()) {
            continue;
        }
        if (!known.empty()) {
            known += L", ";
        }
        known += instance.parentInstanceId;
    }
    error = "No HideFilter instance matches this controller. Filtered devices: " +
            (known.empty() ? std::string("none reported an identity") : narrow(known));
    return false;
}

bool HideFilterIsolation::restore(std::string& error) noexcept {
    if (impl_->handle == INVALID_HANDLE_VALUE) {
        return true;
    }
    // Explicit show first so a later failure to close is still visible, then
    // close the handle regardless: the kernel decrements the per-handle count
    // on close, which is what guarantees no leak of hidden state.
    const bool shown = impl_->send(detail::hideFilterPresenceByte(false), error);
    CloseHandle(impl_->handle);
    impl_->handle = INVALID_HANDLE_VALUE;
    impl_->instanceId.clear();
    return shown;
}

bool HideFilterIsolation::active() const noexcept {
    return impl_->handle != INVALID_HANDLE_VALUE;
}

std::wstring HideFilterIsolation::hiddenInstanceId() const noexcept {
    return impl_->instanceId;
}

}  // namespace asb::platform
