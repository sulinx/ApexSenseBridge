#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <hidsdi.h>
#include <hidpi.h>

#include "platform/PhysicalInputSource.h"
#include "platform/PhysicalInputFreshnessWatchdog.h"

#include "flydigi/Apex4Input.h"
#include "flydigi/Apex4Protocol.h"
#include "flydigi/Apex5Identity.h"
#include "flydigi/Apex5Input.h"
#include "flydigi/Apex5Protocol.h"
#include "flydigi/Apex6Protocol.h"
#include "platform/Apex6InputFusion.h"
#include "platform/HidTransport.h"
#include "platform/XInputGamepad.h"
#include "platform/XInputMapping.h"
#include "platform/windows/WindowsOverlappedIo.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace asb::platform {
namespace {

constexpr USAGE kUsagePageGenericDesktop = 0x01;
constexpr USAGE kUsagePageButton = 0x09;
constexpr USAGE kUsageX = 0x30;
constexpr USAGE kUsageY = 0x31;
constexpr USAGE kUsageZ = 0x32;
constexpr USAGE kUsageRx = 0x33;
constexpr USAGE kUsageRy = 0x34;
constexpr USAGE kUsageRz = 0x35;
constexpr USAGE kUsageHatSwitch = 0x39;
constexpr USAGE kUsageDpadUp = 0x90;
constexpr USAGE kUsageDpadDown = 0x91;
constexpr USAGE kUsageDpadRight = 0x92;
constexpr USAGE kUsageDpadLeft = 0x93;

std::string win32Error(DWORD code) {
    LPSTR buffer = nullptr;
    const DWORD size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
    std::string message = size && buffer ? std::string(buffer, size)
                                         : "Unknown Win32 error";
    if (buffer) LocalFree(buffer);
    while (!message.empty() && (message.back() == '\r' || message.back() == '\n')) {
        message.pop_back();
    }
    return message;
}

bool containsUsage(const HIDP_VALUE_CAPS& cap, USAGE usage) noexcept {
    if (cap.IsRange) {
        return usage >= cap.Range.UsageMin && usage <= cap.Range.UsageMax;
    }
    return usage == cap.NotRange.Usage;
}

bool containsUsage(const HIDP_BUTTON_CAPS& cap, USAGE usage) noexcept {
    if (cap.IsRange) {
        return usage >= cap.Range.UsageMin && usage <= cap.Range.UsageMax;
    }
    return usage == cap.NotRange.Usage;
}

std::int64_t signedLogicalValue(ULONG raw, const HIDP_VALUE_CAPS& cap) noexcept {
    if (cap.LogicalMin >= 0 || cap.BitSize == 0 || cap.BitSize >= 32) {
        return cap.LogicalMin < 0 ? static_cast<std::int32_t>(raw)
                                  : static_cast<std::int64_t>(raw);
    }
    const ULONG signBit = ULONG{1} << (cap.BitSize - 1);
    const ULONG mask = (ULONG{1} << cap.BitSize) - 1;
    raw &= mask;
    return (raw & signBit) != 0
        ? static_cast<std::int64_t>(raw | ~mask)
        : static_cast<std::int64_t>(raw);
}

std::int64_t logicalMaximum(const HIDP_VALUE_CAPS& cap) noexcept {
    // hid.dll exposes an unsigned all-bits-set logical maximum through the
    // signed LONG field as -1. XUSB descriptors use this for their 16-bit
    // sticks and combined trigger axis.
    if (cap.LogicalMin >= 0 && cap.LogicalMax < cap.LogicalMin &&
        cap.BitSize > 0 && cap.BitSize <= 32) {
        return cap.BitSize == 32
            ? static_cast<std::int64_t>((std::numeric_limits<ULONG>::max)())
            : (std::int64_t{1} << cap.BitSize) - 1;
    }
    return static_cast<std::int64_t>(cap.LogicalMax);
}

std::uint8_t normalizeByte(ULONG raw, const HIDP_VALUE_CAPS& cap) noexcept {
    const auto minimum = static_cast<std::int64_t>(cap.LogicalMin);
    const auto maximum = logicalMaximum(cap);
    if (maximum <= minimum) return 0;
    const auto value = std::clamp(signedLogicalValue(raw, cap), minimum, maximum);
    const auto numerator = static_cast<std::uint64_t>(value - minimum) * 255ULL;
    const auto denominator = static_cast<std::uint64_t>(maximum - minimum);
    return static_cast<std::uint8_t>((numerator + denominator / 2) / denominator);
}

DWORD waitMilliseconds(std::chrono::milliseconds timeout) noexcept {
    if (timeout.count() <= 0) return 0;
    constexpr auto maximum =
        static_cast<long long>((std::numeric_limits<DWORD>::max)() - 1);
    return static_cast<DWORD>((std::min)(timeout.count(), maximum));
}

class HidPhysicalInputSource final : public PhysicalInputSource {
public:
    static std::unique_ptr<HidPhysicalInputSource> open(
        const HidDeviceInfo& vendorInterface, std::string& error) {
        std::string enumerateError;
        auto devices = enumerateHidDevices(enumerateError);
        std::vector<HidDeviceInfo> candidates;
        for (auto& device : devices) {
            if (device.vendorId != vendorInterface.vendorId ||
                device.productId != vendorInterface.productId ||
                device.usagePage != kUsagePageGenericDesktop ||
                device.usage != 0x05 || device.inputReportLength == 0) {
                continue;
            }
            if (!vendorInterface.containerId.empty() &&
                device.containerId != vendorInterface.containerId) {
                continue;
            }
            candidates.push_back(std::move(device));
        }
        if (candidates.empty() && !vendorInterface.containerId.empty()) {
            error = "No APEX game-controller HID collection belongs to the selected "
                    "FORCEADAPT interface container.";
            return {};
        }
        if (candidates.empty()) {
            error = enumerateError.empty()
                ? "No APEX game-controller HID collection was found."
                : "APEX game-controller HID enumeration failed: " + enumerateError;
            return {};
        }
        if (candidates.size() != 1) {
            error = "The selected APEX could not be associated with exactly one HID "
                    "game-controller collection.";
            return {};
        }

        HANDLE handle = CreateFileW(
            candidates.front().path.c_str(), GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            const auto code = GetLastError();
            error = "Could not open the selected APEX HID game-controller collection (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return {};
        }

        PHIDP_PREPARSED_DATA preparsed = nullptr;
        if (!HidD_GetPreparsedData(handle, &preparsed)) {
            const auto code = GetLastError();
            CloseHandle(handle);
            error = "Could not read the APEX HID descriptor (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return {};
        }

        HIDP_CAPS caps{};
        if (HidP_GetCaps(preparsed, &caps) != HIDP_STATUS_SUCCESS ||
            caps.InputReportByteLength == 0) {
            HidD_FreePreparsedData(preparsed);
            CloseHandle(handle);
            error = "The selected APEX HID game-controller descriptor is invalid.";
            return {};
        }

        auto source = std::unique_ptr<HidPhysicalInputSource>(
            new HidPhysicalInputSource(std::move(candidates.front()), handle,
                                       preparsed, caps));
        if (!source->loadCapabilities(error)) return {};
        return source;
    }

    ~HidPhysicalInputSource() override {
        if (handle_ != INVALID_HANDLE_VALUE) {
            if (readPending_ && !cancelOverlappedIo(handle_, overlapped_)) {
                (void)io_.release();
            }
            CloseHandle(handle_);
        }
        if (preparsed_) HidD_FreePreparsedData(preparsed_);
    }

    PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override {
        error.clear();
        if (!prepareRead(error)) return PhysicalInputStatus::Error;

        const DWORD waitResult = WaitForSingleObject(event_, waitMilliseconds(timeout));
        if (waitResult == WAIT_TIMEOUT) {
            ++stats_.timeouts;
            return PhysicalInputStatus::Timeout;
        }
        if (waitResult != WAIT_OBJECT_0) {
            const auto code = GetLastError();
            error = "Waiting for an APEX HID input report failed (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return PhysicalInputStatus::Error;
        }

        return completePreparedRead(state, error);
    }

    bool prepareRead(std::string& error) { return ensureReadPending(error); }
    HANDLE readEvent() const noexcept { return event_; }

    PhysicalInputStatus completePreparedRead(
        dualsense::DualSenseInputState& state, std::string& error) {
        if (!readPending_) {
            error = "The APEX HID input read was not prepared.";
            return PhysicalInputStatus::Error;
        }

        DWORD bytesRead = 0;
        readPending_ = false;
        if (!GetOverlappedResult(handle_, &overlapped_, &bytesRead, FALSE)) {
            const auto code = GetLastError();
            if (code == ERROR_DEVICE_NOT_CONNECTED || code == ERROR_INVALID_HANDLE ||
                code == ERROR_OPERATION_ABORTED) {
                error = "The physical APEX HID input source disconnected.";
                return PhysicalInputStatus::Disconnected;
            }
            error = "Completing the APEX HID input report failed (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return PhysicalInputStatus::Error;
        }
        if (bytesRead == 0 || bytesRead > report_.size()) {
            ++stats_.parseFailures;
            error = "The physical APEX returned an empty or oversized HID input report.";
            return PhysicalInputStatus::Error;
        }

        if (!parseReport(bytesRead, state, error)) {
            ++stats_.parseFailures;
            return PhysicalInputStatus::Error;
        }
        ++stats_.reports;
        return PhysicalInputStatus::State;
    }

    std::string_view backendName() const noexcept override { return "apex-hid-event"; }
    bool eventDriven() const noexcept override { return true; }
    PhysicalInputSourceStats stats() const noexcept override { return stats_; }
    [[nodiscard]] bool hasCombinedTriggerAxis() const noexcept { return combinedTriggerAxis_; }
    void setBatteryState(std::uint8_t batteryPercent,
                         std::uint8_t chargeState) noexcept override {
        currentBatteryPercent_.store(batteryPercent, std::memory_order_relaxed);
        currentChargeState_.store(chargeState, std::memory_order_relaxed);
    }

private:
    HidPhysicalInputSource(HidDeviceInfo info, HANDLE handle,
                           PHIDP_PREPARSED_DATA preparsed, HIDP_CAPS caps)
        : info_(std::move(info)), handle_(handle), preparsed_(preparsed), caps_(caps),
          io_(std::make_unique<OverlappedIo>(caps.InputReportByteLength)),
          event_(io_->event), overlapped_(io_->overlapped), report_(io_->buffer) {}

    bool loadCapabilities(std::string& error) {
        if (!event_) {
            error = "Could not create the APEX HID input event.";
            return false;
        }
        valueCaps_.resize(caps_.NumberInputValueCaps);
        USHORT valueCount = static_cast<USHORT>(valueCaps_.size());
        if (valueCount != 0 && HidP_GetValueCaps(
                HidP_Input, valueCaps_.data(), &valueCount, preparsed_) !=
                HIDP_STATUS_SUCCESS) {
            error = "Could not read the APEX HID axis capabilities.";
            return false;
        }
        valueCaps_.resize(valueCount);

        buttonCaps_.resize(caps_.NumberInputButtonCaps);
        USHORT buttonCount = static_cast<USHORT>(buttonCaps_.size());
        if (buttonCount != 0 && HidP_GetButtonCaps(
                HidP_Input, buttonCaps_.data(), &buttonCount, preparsed_) !=
                HIDP_STATUS_SUCCESS) {
            error = "Could not read the APEX HID button capabilities.";
            return false;
        }
        buttonCaps_.resize(buttonCount);

        const auto hasValue = [this](USAGE usage) {
            return std::any_of(valueCaps_.begin(), valueCaps_.end(),
                               [usage](const auto& cap) {
                                   return cap.UsagePage == kUsagePageGenericDesktop &&
                                          containsUsage(cap, usage);
                               });
        };
        const auto hasCombinedTriggerAxis = [this] {
            return std::any_of(valueCaps_.begin(), valueCaps_.end(),
                               [](const auto& cap) {
                                   return cap.UsagePage == kUsagePageGenericDesktop &&
                                          containsUsage(cap, kUsageZ) &&
                                          cap.BitSize == 16 && cap.ReportCount == 1 &&
                                          cap.LogicalMin == 0 &&
                                          logicalMaximum(cap) == 0xFFFF;
                               });
        };
        if (!hasValue(kUsageX) || !hasValue(kUsageY) || !hasValue(kUsageRx) ||
            !hasValue(kUsageRy) || !hasValue(kUsageZ) ||
            (!hasValue(kUsageRz) && !hasCombinedTriggerAxis())) {
            std::ostringstream details;
            details << "The APEX HID descriptor does not expose complete sticks and either "
                       "separate triggers or an Xbox combined trigger axis. Available value usages:";
            for (const auto& cap : valueCaps_) {
                details << " page=0x" << std::hex << cap.UsagePage << " usage=0x";
                if (cap.IsRange) {
                    details << cap.Range.UsageMin << "-0x" << cap.Range.UsageMax;
                } else {
                    details << cap.NotRange.Usage;
                }
                details << std::dec << " bits=" << cap.BitSize
                        << " count=" << cap.ReportCount
                        << " logical=" << cap.LogicalMin << ".." << cap.LogicalMax;
            }
            error = details.str();
            return false;
        }
        combinedTriggerAxis_ = !hasValue(kUsageRz) && hasCombinedTriggerAxis();
        return true;
    }

    bool ensureReadPending(std::string& error) {
        if (readPending_) return true;
        ResetEvent(event_);
        std::fill(report_.begin(), report_.end(), std::uint8_t{0});
        if (ReadFile(handle_, report_.data(), static_cast<DWORD>(report_.size()),
                     nullptr, &overlapped_)) {
            SetEvent(event_);
            readPending_ = true;
            return true;
        }
        const auto code = GetLastError();
        if (code == ERROR_IO_PENDING) {
            readPending_ = true;
            return true;
        }
        error = "Starting the APEX HID input read failed (" +
                std::to_string(code) + ": " + win32Error(code) + ").";
        return false;
    }

    bool usageValue(USAGE usage, ULONG& raw,
                    const HIDP_VALUE_CAPS*& matched) const noexcept {
        for (const auto& cap : valueCaps_) {
            if (cap.UsagePage != kUsagePageGenericDesktop ||
                !containsUsage(cap, usage)) {
                continue;
            }
            const auto status = HidP_GetUsageValue(
                HidP_Input, kUsagePageGenericDesktop, cap.LinkCollection,
                usage, &raw, preparsed_,
                reinterpret_cast<PCHAR>(const_cast<std::uint8_t*>(report_.data())),
                static_cast<ULONG>(report_.size()));
            if (status == HIDP_STATUS_SUCCESS) {
                matched = &cap;
                return true;
            }
        }
        return false;
    }

    bool buttonPressed(USAGE usage) const noexcept {
        for (const auto& cap : buttonCaps_) {
            if (cap.UsagePage != kUsagePageButton || !containsUsage(cap, usage)) {
                continue;
            }
            std::array<USAGE, 64> active{};
            ULONG count = static_cast<ULONG>(active.size());
            const auto status = HidP_GetUsages(
                HidP_Input, kUsagePageButton, cap.LinkCollection,
                active.data(), &count, preparsed_,
                reinterpret_cast<PCHAR>(const_cast<std::uint8_t*>(report_.data())),
                static_cast<ULONG>(report_.size()));
            if (status != HIDP_STATUS_SUCCESS && status != HIDP_STATUS_BUFFER_TOO_SMALL) {
                continue;
            }
            if (std::find(active.begin(), active.begin() +
                          static_cast<std::ptrdiff_t>((std::min)(count, static_cast<ULONG>(active.size()))),
                          usage) != active.begin() +
                          static_cast<std::ptrdiff_t>((std::min)(count, static_cast<ULONG>(active.size())))) {
                return true;
            }
        }
        return false;
    }

    bool genericDesktopButton(USAGE usage) const noexcept {
        ULONG raw = 0;
        const HIDP_VALUE_CAPS* cap = nullptr;
        return usageValue(usage, raw, cap) && raw != 0;
    }

    bool parseReport(DWORD bytesRead, dualsense::DualSenseInputState& state,
                     std::string& error) {
        (void)bytesRead;
        dualsense::DualSenseInputState decoded{};
        const auto readAxis = [this, &error](USAGE usage, std::uint8_t& target) {
            ULONG raw = 0;
            const HIDP_VALUE_CAPS* cap = nullptr;
            if (!usageValue(usage, raw, cap) || !cap) {
                error = "An expected APEX HID axis was missing from an input report.";
                return false;
            }
            target = normalizeByte(raw, *cap);
            return true;
        };
        if (!readAxis(kUsageX, decoded.lx) || !readAxis(kUsageY, decoded.ly) ||
            !readAxis(kUsageRx, decoded.rx) || !readAxis(kUsageRy, decoded.ry)) {
            return false;
        }

        ULONG combinedTrigger = 0;
        const HIDP_VALUE_CAPS* triggerCap = nullptr;
        if (usageValue(kUsageRz, combinedTrigger, triggerCap) && triggerCap) {
            // Flydigi's Xbox-compatible HID collection exposes LT on Rz and
            // RT on Z (the reverse of the assumption used by the first
            // generic mapper). This ordering is shared by Apex 5 and 6.
            decoded.l2 = normalizeByte(combinedTrigger, *triggerCap);
            if (!readAxis(kUsageZ, decoded.r2)) return false;
        } else if (usageValue(kUsageZ, combinedTrigger, triggerCap) && triggerCap &&
                   triggerCap->BitSize == 16 && triggerCap->ReportCount == 1 &&
                   triggerCap->LogicalMin == 0 &&
                   logicalMaximum(*triggerCap) == 0xFFFF) {
            // The Xbox HID compatibility collection projects LT and RT onto
            // one centered DirectInput-style axis: RT moves toward 0 and LT
            // toward 65535. A one-count center wobble must remain neutral.
            mapCombinedTriggerAxis(
                static_cast<std::uint16_t>(combinedTrigger),
                decoded.l2, decoded.r2);
        } else {
            error = "The APEX HID trigger state was missing from an input report.";
            return false;
        }

        std::uint16_t xinputButtons = 0;
        if (buttonPressed(1)) xinputButtons |= xinputButton::kA;
        if (buttonPressed(2)) xinputButtons |= xinputButton::kB;
        if (buttonPressed(3)) xinputButtons |= xinputButton::kX;
        if (buttonPressed(4)) xinputButtons |= xinputButton::kY;
        if (buttonPressed(5)) xinputButtons |= xinputButton::kLeftShoulder;
        if (buttonPressed(6)) xinputButtons |= xinputButton::kRightShoulder;
        if (buttonPressed(7)) xinputButtons |= xinputButton::kBack;
        if (buttonPressed(8)) xinputButtons |= xinputButton::kStart;
        if (buttonPressed(9)) xinputButtons |= xinputButton::kLeftThumb;
        if (buttonPressed(10)) xinputButtons |= xinputButton::kRightThumb;

        ULONG hat = 0;
        const HIDP_VALUE_CAPS* hatCap = nullptr;
        if (usageValue(kUsageHatSwitch, hat, hatCap) && hatCap) {
            const auto direction = signedLogicalValue(hat, *hatCap) - hatCap->LogicalMin;
            if (direction == 0 || direction == 1 || direction == 7)
                xinputButtons |= xinputButton::kDpadUp;
            if (direction == 3 || direction == 4 || direction == 5)
                xinputButtons |= xinputButton::kDpadDown;
            if (direction == 5 || direction == 6 || direction == 7)
                xinputButtons |= xinputButton::kDpadLeft;
            if (direction == 1 || direction == 2 || direction == 3)
                xinputButtons |= xinputButton::kDpadRight;
        } else {
            if (genericDesktopButton(kUsageDpadUp))
                xinputButtons |= xinputButton::kDpadUp;
            if (genericDesktopButton(kUsageDpadDown))
                xinputButtons |= xinputButton::kDpadDown;
            if (genericDesktopButton(kUsageDpadLeft))
                xinputButtons |= xinputButton::kDpadLeft;
            if (genericDesktopButton(kUsageDpadRight))
                xinputButtons |= xinputButton::kDpadRight;
        }

        mapXInputButtons(xinputButtons, decoded.l2, decoded.r2, decoded);
        if (buttonPressed(11)) decoded.buttons |= dualsense::button::kPs;
        decoded.batteryPercent = currentBatteryPercent_.load(std::memory_order_relaxed);
        decoded.chargeState = currentChargeState_.load(std::memory_order_relaxed);
        state = decoded;
        return true;
    }

    HidDeviceInfo info_;
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    PHIDP_PREPARSED_DATA preparsed_ = nullptr;
    HIDP_CAPS caps_{};
    std::unique_ptr<OverlappedIo> io_;
    HANDLE event_;
    OVERLAPPED& overlapped_;
    bool readPending_ = false;
    std::vector<std::uint8_t>& report_;
    std::vector<HIDP_VALUE_CAPS> valueCaps_;
    std::vector<HIDP_BUTTON_CAPS> buttonCaps_;
    bool combinedTriggerAxis_ = false;
    PhysicalInputSourceStats stats_{};
    std::atomic<std::uint8_t> currentBatteryPercent_{100};
    std::atomic<std::uint8_t> currentChargeState_{0};
};

class Apex6HybridPhysicalInputSource final : public PhysicalInputSource {
public:
    Apex6HybridPhysicalInputSource(
        std::unique_ptr<HidPhysicalInputSource> mappedGamepad,
        std::unique_ptr<XInputGamepad> xinputGamepad)
        : mappedGamepad_(std::move(mappedGamepad)),
          xinputGamepad_(std::move(xinputGamepad)) {}

    PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override {
        constexpr auto kTriggerPollInterval = std::chrono::milliseconds(2);
        const auto startedAt = std::chrono::steady_clock::now();
        const auto deadline = startedAt + (std::max)(timeout, std::chrono::milliseconds(0));

        for (;;) {
            const auto beforeWait = std::chrono::steady_clock::now();
            const auto remaining = deadline - beforeWait;
            const auto waitBudget = (std::min)(
                (std::max)(remaining, std::chrono::steady_clock::duration::zero()),
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    kTriggerPollInterval));
            auto waitSlice = std::chrono::duration_cast<std::chrono::milliseconds>(
                waitBudget);
            if (remaining > std::chrono::steady_clock::duration::zero() &&
                waitSlice <= std::chrono::milliseconds(0)) {
                waitSlice = std::chrono::milliseconds(1);
            }

            dualsense::DualSenseInputState mappedState{};
            std::string mappedError;
            const auto mappedStatus = mappedGamepad_->waitForState(
                mappedState, waitSlice, mappedError);
            if (mappedStatus == PhysicalInputStatus::Disconnected ||
                mappedStatus == PhysicalInputStatus::Error) {
                error = std::move(mappedError);
                return mappedStatus;
            }

            dualsense::DualSenseInputState xinputState{};
            std::string xinputError;
            if (!xinputGamepad_->poll(xinputState, xinputError)) {
                error = "The Apex 6 independent XInput trigger source failed (" +
                        xinputError + ").";
                return PhysicalInputStatus::Disconnected;
            }

            const auto observedAt = std::chrono::steady_clock::now();
            const auto fused = fusion_.observe(
                mappedStatus == PhysicalInputStatus::State
                    ? std::optional<dualsense::DualSenseInputState>(mappedState)
                    : std::nullopt,
                xinputState, observedAt);
            if (fused) {
                state = *fused;
                ++stats_.reports;
                error.clear();
                return PhysicalInputStatus::State;
            }
            if (observedAt >= deadline) {
                ++stats_.timeouts;
                error.clear();
                return PhysicalInputStatus::Timeout;
            }
        }
    }

    std::string_view backendName() const noexcept override {
        return "apex6-hid+xinput-triggers";
    }
    bool eventDriven() const noexcept override { return true; }
    PhysicalInputSourceStats stats() const noexcept override {
        auto result = stats_;
        const auto mappedStats = mappedGamepad_->stats();
        result.parseFailures = mappedStats.parseFailures;
        return result;
    }
    void setBatteryState(std::uint8_t batteryPercent,
                         std::uint8_t chargeState) noexcept override {
        mappedGamepad_->setBatteryState(batteryPercent, chargeState);
    }

private:
    std::unique_ptr<HidPhysicalInputSource> mappedGamepad_;
    std::unique_ptr<XInputGamepad> xinputGamepad_;
    Apex6InputFusion fusion_;
    PhysicalInputSourceStats stats_{};
};

class Apex4PhysicalInputSource final : public PhysicalInputSource {
public:
    static std::unique_ptr<Apex4PhysicalInputSource> open(
        const HidDeviceInfo& vendorInterface, std::string& error) {
        if (!flydigi::isApex4Product(
                vendorInterface.vendorId, vendorInterface.productId) ||
            vendorInterface.usagePage != flydigi::kApex4VendorUsagePage ||
            vendorInterface.inputReportLength < 32) {
            error = "The selected interface is not a complete Apex 4 V1 vendor interface.";
            return {};
        }

        HANDLE handle = CreateFileW(
            vendorInterface.path.c_str(), GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            const auto code = GetLastError();
            error = "Could not open the selected Apex 4 V1 input stream (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return {};
        }

        auto source = std::unique_ptr<Apex4PhysicalInputSource>(
            new Apex4PhysicalInputSource(vendorInterface, handle));
        if (!source->event_) {
            error = "Could not create the Apex 4 V1 input event.";
            return {};
        }
        return source;
    }

    ~Apex4PhysicalInputSource() override {
        if (handle_ != INVALID_HANDLE_VALUE) {
            if (readPending_ && !cancelOverlappedIo(handle_, overlapped_)) {
                (void)io_.release();
            }
            CloseHandle(handle_);
        }
    }

    PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override {
        error.clear();
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        for (;;) {
            if (!ensureReadPending(error)) return PhysicalInputStatus::Error;

            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                ++stats_.timeouts;
                return PhysicalInputStatus::Timeout;
            }
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now);
            if (remaining.count() == 0) remaining = std::chrono::milliseconds(1);

            const DWORD waitResult = WaitForSingleObject(
                event_, waitMilliseconds(remaining));
            if (waitResult == WAIT_TIMEOUT) {
                ++stats_.timeouts;
                return PhysicalInputStatus::Timeout;
            }
            if (waitResult != WAIT_OBJECT_0) {
                const auto code = GetLastError();
                error = "Waiting for an Apex 4 V1 input report failed (" +
                        std::to_string(code) + ": " + win32Error(code) + ").";
                return PhysicalInputStatus::Error;
            }

            DWORD bytesRead = 0;
            readPending_ = false;
            if (!GetOverlappedResult(handle_, &overlapped_, &bytesRead, FALSE)) {
                const auto code = GetLastError();
                if (code == ERROR_DEVICE_NOT_CONNECTED || code == ERROR_INVALID_HANDLE ||
                    code == ERROR_OPERATION_ABORTED) {
                    error = "The physical Apex 4 V1 input stream disconnected.";
                    return PhysicalInputStatus::Disconnected;
                }
                error = "Completing the Apex 4 V1 input report failed (" +
                        std::to_string(code) + ": " + win32Error(code) + ").";
                return PhysicalInputStatus::Error;
            }
            ++stats_.reports;

            const auto decoded = flydigi::decodeApex4InputReport(
                std::span<const std::uint8_t>(report_.data(), bytesRead),
                currentBatteryPercent_.load(std::memory_order_relaxed),
                currentChargeState_.load(std::memory_order_relaxed));
            if (!decoded) {
                // Identity replies and other vendor notifications share this
                // stream. They are valid traffic, just not controller state.
                continue;
            }
            state = *decoded;
            return PhysicalInputStatus::State;
        }
    }

    std::string_view backendName() const noexcept override {
        return "apex4-v1-hid-event";
    }
    bool eventDriven() const noexcept override { return true; }
    PhysicalInputSourceStats stats() const noexcept override { return stats_; }
    void setBatteryState(std::uint8_t batteryPercent, std::uint8_t chargeState) noexcept override {
        currentBatteryPercent_.store(batteryPercent, std::memory_order_relaxed);
        currentChargeState_.store(chargeState, std::memory_order_relaxed);
    }

private:
    Apex4PhysicalInputSource(const HidDeviceInfo& info, HANDLE handle)
        : handle_(handle), io_(std::make_unique<OverlappedIo>(info.inputReportLength)),
          event_(io_->event), overlapped_(io_->overlapped), report_(io_->buffer) {}

    bool ensureReadPending(std::string& error) {
        if (readPending_) return true;
        ResetEvent(event_);
        std::fill(report_.begin(), report_.end(), std::uint8_t{0});
        if (ReadFile(handle_, report_.data(), static_cast<DWORD>(report_.size()),
                     nullptr, &overlapped_)) {
            SetEvent(event_);
            readPending_ = true;
            return true;
        }
        const auto code = GetLastError();
        if (code == ERROR_IO_PENDING) {
            readPending_ = true;
            return true;
        }
        error = "Starting the Apex 4 V1 input read failed (" +
                std::to_string(code) + ": " + win32Error(code) + ").";
        return false;
    }

    HANDLE handle_ = INVALID_HANDLE_VALUE;
    std::unique_ptr<OverlappedIo> io_;
    HANDLE event_;
    OVERLAPPED& overlapped_;
    bool readPending_ = false;
    std::vector<std::uint8_t>& report_;
    PhysicalInputSourceStats stats_{};
    std::atomic<std::uint8_t> currentBatteryPercent_{100};
    std::atomic<std::uint8_t> currentChargeState_{0};
};

class Apex5DualHidPhysicalInputSource final : public PhysicalInputSource {
public:
    static std::unique_ptr<Apex5DualHidPhysicalInputSource> open(
        const HidDeviceInfo& vendorInterface,
        std::string& error,
        bool vaderProduct = false) {
        if (vendorInterface.vendorId != flydigi::kVendorId ||
            !flydigi::isControllerProduct(vendorInterface.productId) ||
            vendorInterface.usagePage != flydigi::kVendorUsagePage ||
            vendorInterface.inputReportLength < 18) {
            error = "The selected interface is not a complete Apex 5 vendor input stream.";
            return {};
        }

        std::string mappedHidError;
        auto mappedGamepad = HidPhysicalInputSource::open(
            vendorInterface, mappedHidError);
        if (!mappedGamepad) {
            error = "Mapped game-controller HID for Apex 5 unavailable (" +
                    mappedHidError + ").";
            return {};
        }

        // Keep mapped HID authoritative for profile-aware controls and use the
        // vendor stream's independent LT/RT bytes when available. Xbox Mode /
        // AnyFSE can expose ordinary XInput with incompatible trigger semantics,
        // so full-state XInput only bootstraps a mapped HID that never reports.
        std::string xinputError;
        auto xinputGamepad = openXInputGamepadForDevice(
            vendorInterface.vendorId, vendorInterface.productId,
            std::nullopt, xinputError);

        HANDLE handle = CreateFileW(
            vendorInterface.path.c_str(), GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            const auto code = GetLastError();
            error = "Could not open the selected Apex 5 vendor input stream (" +
                    std::to_string(code) + ": " + win32Error(code) + ").";
            return {};
        }

        auto source = std::unique_ptr<Apex5DualHidPhysicalInputSource>(
            new Apex5DualHidPhysicalInputSource(
                vendorInterface, handle, std::move(mappedGamepad),
                std::move(xinputGamepad), vaderProduct));
        if (!source->event_) {
            error = "Could not create the Apex 5 vendor input event.";
            return {};
        }
        return source;
    }

    ~Apex5DualHidPhysicalInputSource() override {
        if (handle_ != INVALID_HANDLE_VALUE) {
            if (readPending_ && !cancelOverlappedIo(handle_, overlapped_)) {
                (void)io_.release();
            }
            CloseHandle(handle_);
        }
    }

    PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override {
        error.clear();
        // Standard controls must not depend on the optional vendor stream.
        if (!vendorUnavailable_) {
            std::string vendorError;
            if (!ensureReadPending(vendorError)) {
                vendorUnavailable_ = true;
                ++vendorReadFailures_;
            }
        }

        // LOCAL PATCH: the mapped collection is now purely optional and is never
        // put on the critical path. It is left open (composeState ignores it
        // while a vendor frame is fresh) but it is not armed or waited on here,
        // so its intermittent health cannot throttle or stall the session.
        std::string mappedError;
        (void)mappedError;

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            // LOCAL PATCH: wait on the vendor event ONLY.
            //
            // The vendor 0xEF frame is a complete controller state, so the
            // mapped collection no longer contributes anything the caller needs
            // -- it only served to merge equivalent values back in. Keeping it in
            // the wait set was actively harmful: WaitForMultipleObjects returns
            // the lowest ready index, so a mapped completion always pre-empted a
            // ready vendor completion, and drainVendorReports() then had to guess
            // whether more vendor frames were pending. Measured with the mapped
            // collection in the set: mapped alternated between 0/s and 270/s as
            // Windows re-enumerated it, timeouts climbed to 586, and the vendor
            // rate sagged from 414/s to 214/s in exactly those windows. With a
            // single waiter the vendor stream is serviced the moment its read
            // completes, independent of the mapped collection's health.
            const DWORD eventCount = vendorUnavailable_ ? 0U : 1U;
            std::array<HANDLE, 1> events{event_};
            const auto now = std::chrono::steady_clock::now();
            const auto remaining = now < deadline
                ? std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
                : std::chrono::milliseconds(0);
            const DWORD waitResult = eventCount == 0
                ? WAIT_TIMEOUT
                : WaitForMultipleObjects(eventCount, events.data(), FALSE,
                                         waitMilliseconds(remaining));
            if (waitResult == WAIT_TIMEOUT) {
                if (!mappedInputObserved_ && pollXInputFallback(state)) {
                    ++stats_.reports;
                    return PhysicalInputStatus::State;
                }
                // The vendor wait expired. Collect anything that landed inside
                // the same millisecond before reporting silence, otherwise the
                // caller's freshness watchdog counts pure scheduling jitter as a
                // dead stream.
                if (drainVendorReports() && lastVendorState_) {
                    composeState(state);
                    ++stats_.reports;
                    return PhysicalInputStatus::State;
                }
                ++stats_.timeouts;
                return PhysicalInputStatus::Timeout;
            }
            if (waitResult == WAIT_FAILED) {
                const auto code = GetLastError();
                error = "Waiting for the Apex 5 physical input streams failed (" +
                        std::to_string(code) + ": " + win32Error(code) + ").";
                return PhysicalInputStatus::Error;
            }

            const DWORD selected = waitResult - WAIT_OBJECT_0;
            // LOCAL PATCH: the only waiter is the vendor event, so index 0 is a
            // vendor completion. The mapped collection is maintained out of band
            // by drainVendorReports()/composeState() hints and never gates input.
            (void)selected;

            bool decodedVendorState = consumeVendorReport();
            if (!vendorUnavailable_) {
                std::string pendingError;
                if (!ensureReadPending(pendingError)) {
                    vendorUnavailable_ = true;
                    ++vendorReadFailures_;
                }
            }
            if (drainVendorReports()) decodedVendorState = true;
            if (decodedVendorState && xinputFallbackActive_) {
                if (pollXInputFallback(state)) {
                    ++stats_.reports;
                    return PhysicalInputStatus::State;
                }
                error = "The physical Apex 5 XInput fallback disconnected.";
                return PhysicalInputStatus::Disconnected;
            }
            // LOCAL PATCH: a decoded vendor frame is a complete controller state
            // on its own (sticks, both triggers, dpad and the full button mask all
            // come out of the 0xEF operator report), so it must not be held back
            // waiting for a mapped-HID frame that may never arrive. Previously
            // this branch required lastMappedState_, which meant a degraded
            // mapped collection (23 reports/s measured under load) throttled the
            // whole session even though vendor frames were arriving at 412/s.
            if (decodedVendorState) {
                if (lastMappedState_ || lastVendorState_) {
                    composeState(state);
                    ++stats_.reports;
                    return PhysicalInputStatus::State;
                }
                if (pollXInputFallback(state)) {
                    ++stats_.reports;
                    return PhysicalInputStatus::State;
                }
            }
        }
    }

    std::string_view backendName() const noexcept override {
        return "apex5-v9-mapped+vendor-guarded";
    }

    // LOCAL PATCH: see PhysicalInputSource::recoverStream.
    //
    // Measured stall shape on a Vader 5 Pro over the 2.4 GHz dongle while the
    // controller was in active use: input_vendor_max_gap_ms reached 1625 ms and
    // then 3192/3256 ms, always while decoded-frame counters were healthy on
    // both sides of the gap, with read and parse failure counters at zero for
    // the whole session. That is the device (or its dongle) withholding frames,
    // not a dead handle, so the correct response is to re-arm the read and let
    // the stream resume rather than tear the bridge down.
    bool recoverStream() noexcept override {
        if (vendorUnavailable_) {
            // A previous read failed hard. Assume the device came back and give
            // it one more chance instead of condemning the session.
            vendorUnavailable_ = false;
        }
        std::string recoverError;
        const bool armed = ensureReadArmed(recoverError);
        if (!armed) {
            vendorUnavailable_ = true;
            ++vendorReadFailures_;
            return false;
        }
        // Any frame that landed while the caller was deciding is worth taking
        // now: it re-seeds lastVendorState_ and restarts the gap tracker.
        (void)drainVendorReports();
        return true;
    }
    bool eventDriven() const noexcept override { return true; }
    PhysicalInputSourceStats stats() const noexcept override {
        auto result = stats_;
        const auto mappedStats = mappedGamepad_->stats();
        result.parseFailures += mappedStats.parseFailures;
        result.mappedReports = mappedStats.reports;
        result.vendorReports = vendorReports_;
        result.vendorStates = vendorStates_;
        result.vendorParseFailures = vendorParseFailures_;
        result.vendorReadFailures = vendorReadFailures_;
        result.triggerSourceSwitches = composer_.triggerSourceSwitches();
        result.vendorMaximumGapMs = vendorGaps_.maximumGapMilliseconds();
        result.vendorStaleTriggerGaps = vendorGaps_.staleGaps();
        result.activity = activity_.stats();
        return result;
    }
    bool requiresIndependentTriggers() const noexcept override {
        // LOCAL PATCH: the Vader's LT/RT always come from the vendor 0xEF
        // stream, so the Apex-5 combined-axis hazard never applies and the
        // freshness gate below must not abort the session.
        if (vaderProduct_) return false;
        if (!mappedGamepad_) return false;
        return mappedGamepad_->hasCombinedTriggerAxis();
    }
    bool independentTriggersReady() const noexcept override {
        // LOCAL PATCH: on the Vader the vendor 0xEF stream *is* the independent
        // LT/RT source, so vendor readiness alone is enough. Requiring mapped
        // HID as well aborts the session whenever that collection is quiet.
        if (vaderProduct_) {
            return !vendorUnavailable_ &&
                   independentTriggerFreshness_.ready();
        }
        return mappedInputObserved_ && !vendorUnavailable_ && independentTriggerFreshness_.ready();
    }
    void setBatteryState(std::uint8_t batteryPercent, std::uint8_t chargeState) noexcept override {
        currentBatteryPercent_.store(batteryPercent, std::memory_order_relaxed);
        currentChargeState_.store(chargeState, std::memory_order_relaxed);
        mappedGamepad_->setBatteryState(batteryPercent, chargeState);
    }

private:
    Apex5DualHidPhysicalInputSource(
        const HidDeviceInfo& info, HANDLE handle,
        std::unique_ptr<HidPhysicalInputSource> mappedGamepad,
        std::unique_ptr<XInputGamepad> xinputGamepad,
        bool vaderProduct)
        : handle_(handle), io_(std::make_unique<OverlappedIo>(info.inputReportLength)),
          event_(io_->event), overlapped_(io_->overlapped), report_(io_->buffer),
          mappedGamepad_(std::move(mappedGamepad)),
          xinputGamepad_(std::move(xinputGamepad)),
          // LOCAL PATCH: the Vader 5 Pro shares this vendor interface but not
          // the Apex 5 operator-report layout, so remember which parser to use.
          vaderProduct_(vaderProduct) {}

    bool ensureReadPending(std::string& error) {
        if (readPending_) return true;
        ResetEvent(event_);
        std::fill(report_.begin(), report_.end(), std::uint8_t{0});
        if (ReadFile(handle_, report_.data(), static_cast<DWORD>(report_.size()),
                     nullptr, &overlapped_)) {
            SetEvent(event_);
            readPending_ = true;
            return true;
        }
        const auto code = GetLastError();
        if (code == ERROR_IO_PENDING) {
            readPending_ = true;
            return true;
        }
        error = "Starting the Apex 5 vendor input read failed (" +
                std::to_string(code) + ": " + win32Error(code) + ").";
        return false;
    }

    bool consumeVendorReport() noexcept {
        DWORD bytesRead = 0;
        readPending_ = false;
        if (!GetOverlappedResult(handle_, &overlapped_, &bytesRead, FALSE)) {
            vendorUnavailable_ = true;
            ++vendorReadFailures_;
            return false;
        }
        // LOCAL PATCH: consume the completion event here, not in the next
        // ensureReadPending(). A manual-reset event stays signalled after the
        // IRP completes, so leaving it set lets the next arm reset it *after*
        // the kernel already reported a fresh completion; that lost edge makes
        // the stream freeze with vendorReadFailures_ and vendorParseFailures_
        // both at zero, which is exactly the failure measured under sustained
        // input (frame counter stuck while every error counter stayed still).
        // Resetting immediately after collecting the result means the next wait
        // only sees completions that happened after this frame.
        ResetEvent(event_);
        ++vendorReports_;

        if (bytesRead >= 14 &&
            report_[0] == flydigi::kReportIdIn &&
            report_[1] == flydigi::kMagic0 &&
            report_[2] == flydigi::kMagic1 &&
            report_[3] == flydigi::kCmdGetInfo) {
            const auto parsed = flydigi::Apex5Identity::parseReply(
                std::span<const std::uint8_t>(report_.data(), bytesRead));
            if (parsed) {
                currentBatteryPercent_.store(
                    parsed->batteryPercent(), std::memory_order_relaxed);
                currentChargeState_.store(
                    parsed->chargeState(), std::memory_order_relaxed);
            } else {
                ++vendorParseFailures_;
            }
            return false;
        }

        // LOCAL PATCH: the Vader 5 Pro uses report ID 0x00 and a different
        // 0xEF frame layout, so dispatch on the product instead of requiring
        // the Apex-family input report ID.
        if (vaderProduct_) {
            // Command ACKs (0x12 rumble baselines, 0x11 transport, 0x1C
            // acquire) share this interface and arrive interleaved with the
            // operator stream. They are not operator frames, so skip them
            // without counting a parse failure.
            const bool isCommandAck =
                bytesRead >= 4 && report_[0] == flydigi::kReportIdOutVader &&
                report_[1] == flydigi::kMagic0 &&
                report_[2] == flydigi::kMagic1 &&
                report_[3] != flydigi::kCmdOperatorData;
            if (isCommandAck) return false;

            const auto decoded = flydigi::decodeVaderInputReport(
                std::span<const std::uint8_t>(report_.data(), bytesRead),
                currentBatteryPercent_.load(std::memory_order_relaxed),
                currentChargeState_.load(std::memory_order_relaxed));
            if (!decoded) {
                ++vendorParseFailures_;
                return false;
            }
            const auto decodedAt = std::chrono::steady_clock::now();
            lastVendorState_ = *decoded;
            lastVendorStateAt_ = decodedAt;
            independentTriggerFreshness_.observe(decodedAt);
            vendorGaps_.observe(decodedAt);
            ++vendorStates_;
            return true;
        }

        if (bytesRead < 4 || report_[0] != flydigi::kReportIdIn ||
            report_[1] != flydigi::kMagic0 || report_[2] != flydigi::kMagic1 ||
            report_[3] != flydigi::kCmdOperatorData) {
            return false;
        }
        const auto decoded = flydigi::decodeApex5InputReport(
            std::span<const std::uint8_t>(report_.data(), bytesRead),
            currentBatteryPercent_.load(std::memory_order_relaxed),
            currentChargeState_.load(std::memory_order_relaxed));
        if (!decoded) {
            ++vendorParseFailures_;
            return false;
        }
        const auto decodedAt = std::chrono::steady_clock::now();
        lastVendorState_ = *decoded;
        lastVendorStateAt_ = decodedAt;
        independentTriggerFreshness_.observe(decodedAt);
        vendorGaps_.observe(decodedAt);
        ++vendorStates_;
        return true;
    }

    // LOCAL PATCH: guarantees the vendor read is genuinely in flight. A read
    // whose completion was already consumed elsewhere leaves readPending_ set
    // while no I/O is outstanding, which would silence the 0xEF stream forever.
    bool ensureReadArmed(std::string& error) {
        if (readPending_) {
            // Outstanding I/O with an unsignalled event is healthy.
            if (WaitForSingleObject(event_, 0) != WAIT_OBJECT_0) return true;
            // Signalled: the transfer completed. Collect it so the buffer can
            // be reused, then fall through to arm a fresh read.
            DWORD bytesRead = 0;
            readPending_ = false;
            (void)GetOverlappedResult(handle_, &overlapped_, &bytesRead, FALSE);
        }
        return ensureReadPending(error);
    }

    bool drainVendorReports() noexcept {
        if (vendorUnavailable_) return false;
        constexpr unsigned int kMaximumDrainedReports = 64;
        bool decoded = false;
        for (unsigned int drained = 0; drained < kMaximumDrainedReports; ++drained) {
            // LOCAL PATCH: a one-shot poll returns immediately when the vendor
            // read has not completed yet, which lets a busy mapped-HID
            // collection starve this stream forever. Wait a token amount so a
            // frame already in flight is collected on this pass.
            const DWORD timeout = drained == 0 ? 1U : 0U;
            const DWORD waitResult = WaitForSingleObject(event_, timeout);
            if (waitResult == WAIT_TIMEOUT) {
                // LOCAL PATCH: the vendor stream must always be armed. When a
                // previous read completed but its event was consumed by the
                // mapped-HID branch, readPending_ still says "pending" and the
                // stream dies silently: frame count freezes, no error counter
                // moves, and the session aborts one second later. Re-arm
                // unconditionally here — a live read simply keeps going, and a
                // dead one is resurrected.
                std::string pendingError;
                if (!ensureReadArmed(pendingError)) {
                    vendorUnavailable_ = true;
                    ++vendorReadFailures_;
                }
                return decoded;
            }
            if (waitResult != WAIT_OBJECT_0) {
                vendorUnavailable_ = true;
                ++vendorReadFailures_;
                return decoded;
            }
            if (consumeVendorReport()) decoded = true;
            if (vendorUnavailable_) return decoded;

            std::string pendingError;
            if (!ensureReadPending(pendingError)) {
                vendorUnavailable_ = true;
                ++vendorReadFailures_;
                return decoded;
            }
        }
        return decoded;
    }

    void composeState(dualsense::DualSenseInputState& state) noexcept {
        // Motion remains valid when standard controls bootstrap through XInput.
        // independentTriggersReady() also requires mapped HID, which may never
        // report in this mode and must not suppress a healthy IMU stream.
        const auto now = std::chrono::steady_clock::now();
        const bool vendorReady = !vendorUnavailable_ && independentTriggerFreshness_.ready(now);
        const auto vendorTriggerAge = lastVendorStateAt_
            ? now - *lastVendorStateAt_
            : std::chrono::steady_clock::duration::max();
        // LOCAL PATCH: prefer the vendor frame outright for every standard
        // control when one is available.
        //
        // The composer always merges the mapped HID collection over the vendor
        // frame (mergeApex5MappedControls overwrites both sticks, both triggers,
        // dpad and the whole button mask), so the mapped collection is on the
        // critical path for input that the vendor frame already carries in full
        // -- decodeVaderInputReport decodes sticks, triggers, dpad and all
        // buttons from the 0xEF operator report. Measured on this machine: the
        // mapped game-controller collection degrades from 220 reports/s in
        // isolation to 23 reports/s (-90 %) once the bridge is running, with
        // 815 timeouts over 136 s, while the vendor stream holds 412 frames/s.
        // Letting the degraded collection overwrite good vendor data is what
        // made the whole session freeze; the vendor frame alone is complete
        // apart from IMU, which the Vader does not report at all.
        if (vendorReady && lastVendorState_) {
            state = *lastVendorState_;
            state.batteryPercent = currentBatteryPercent_.load(std::memory_order_relaxed);
            state.chargeState = currentChargeState_.load(std::memory_order_relaxed);
            activity_.observe(state, now);
            return;
        }
        if (!lastMappedState_) {
            // Nothing to compose from yet; report neutral rather than dangling.
            state = dualsense::DualSenseInputState{};
            state.batteryPercent = currentBatteryPercent_.load(std::memory_order_relaxed);
            state.chargeState = currentChargeState_.load(std::memory_order_relaxed);
            return;
        }
        state = composer_.compose(
            *lastMappedState_,
            std::nullopt,
            vendorTriggerAge,
            currentBatteryPercent_.load(std::memory_order_relaxed),
            currentChargeState_.load(std::memory_order_relaxed));
        activity_.observe(state, now);
    }

    bool pollXInputFallback(dualsense::DualSenseInputState& state) noexcept {
        // A short mapped-HID silence is normal for an event-driven collection.
        // XInput may bootstrap installations whose HID never produces a first
        // report, but must never replace a mapped source that already proved
        // healthy: alternating those sources can create false trigger releases.
        if (mappedInputObserved_ || !xinputGamepad_) return false;
        dualsense::DualSenseInputState xinputState{};
        std::string ignored;
        if (!xinputGamepad_->poll(xinputState, ignored)) return false;
        lastMappedState_ = xinputState;
        xinputFallbackActive_ = true;
        drainVendorReports();
        composeState(state);
        return true;
    }

    HANDLE handle_ = INVALID_HANDLE_VALUE;
    std::unique_ptr<OverlappedIo> io_;
    HANDLE event_;
    OVERLAPPED& overlapped_;
    bool readPending_ = false;
    std::vector<std::uint8_t>& report_;
    std::optional<dualsense::DualSenseInputState> lastVendorState_;    std::optional<std::chrono::steady_clock::time_point> lastVendorStateAt_;
    IndependentTriggerStreamFreshness independentTriggerFreshness_;
    flydigi::Apex5InputComposer composer_;
    PhysicalInputActivityTracker activity_;
    VendorStreamGapTracker vendorGaps_{flydigi::Apex5InputComposer::kTriggerFreshness};
    std::uint64_t vendorReports_ = 0;
    std::uint64_t vendorStates_ = 0;
    std::uint64_t vendorParseFailures_ = 0;
    std::uint64_t vendorReadFailures_ = 0;
    bool vendorUnavailable_ = false;
    PhysicalInputSourceStats stats_{};
    std::atomic<std::uint8_t> currentBatteryPercent_{100};
    std::atomic<std::uint8_t> currentChargeState_{0};
    std::unique_ptr<HidPhysicalInputSource> mappedGamepad_;
    std::unique_ptr<XInputGamepad> xinputGamepad_;
    std::optional<dualsense::DualSenseInputState> lastMappedState_;
    bool mappedInputObserved_ = false;
    bool xinputFallbackActive_ = false;
    // LOCAL PATCH: true when this session drives a Vader 5 Pro, which uses the
    // 0x00 report ID and its own 0xEF operator-report layout.
    bool vaderProduct_ = false;
};

class XInputPhysicalInputSource final : public PhysicalInputSource {
public:
    explicit XInputPhysicalInputSource(std::unique_ptr<XInputGamepad> gamepad)
        : gamepad_(std::move(gamepad)) {
        constexpr DWORD kHighResolution = 0x00000002;
        timer_ = CreateWaitableTimerExW(
            nullptr, nullptr, kHighResolution, TIMER_MODIFY_STATE | SYNCHRONIZE);
        if (!timer_) {
            timer_ = CreateWaitableTimerExW(
                nullptr, nullptr, 0, TIMER_MODIFY_STATE | SYNCHRONIZE);
        }
    }

    ~XInputPhysicalInputSource() override {
        if (timer_) CloseHandle(timer_);
    }

    PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override {
        if (timeout.count() > 0) {
            LARGE_INTEGER due{};
            due.QuadPart = -static_cast<LONGLONG>(timeout.count()) * 10000LL;
            if (timer_ && SetWaitableTimer(timer_, &due, 0, nullptr, nullptr, FALSE)) {
                (void)WaitForSingleObject(timer_, INFINITE);
            } else {
                std::this_thread::sleep_for(timeout);
            }
        }
        if (!gamepad_->poll(state, error)) return PhysicalInputStatus::Disconnected;
        const auto now = std::chrono::steady_clock::now();
        if (now - lastBatteryQuery_ >= std::chrono::seconds(15)) {
            lastBatteryQuery_ = now;
            std::uint8_t percent = 100;
            std::uint8_t charge = 0;
            if (gamepad_->queryBattery(percent, charge)) {
                currentBatteryPercent_.store(percent, std::memory_order_relaxed);
                currentChargeState_.store(charge, std::memory_order_relaxed);
            }
        }
        state.batteryPercent = currentBatteryPercent_.load(std::memory_order_relaxed);
        state.chargeState = currentChargeState_.load(std::memory_order_relaxed);
        ++stats_.reports;
        return PhysicalInputStatus::State;
    }
    std::string_view backendName() const noexcept override { return "xinput-fallback"; }
    bool eventDriven() const noexcept override { return false; }
    PhysicalInputSourceStats stats() const noexcept override { return stats_; }
    void setBatteryState(std::uint8_t batteryPercent, std::uint8_t chargeState) noexcept override {
        currentBatteryPercent_.store(batteryPercent, std::memory_order_relaxed);
        currentChargeState_.store(chargeState, std::memory_order_relaxed);
    }

private:
    std::unique_ptr<XInputGamepad> gamepad_;
    HANDLE timer_ = nullptr;
    PhysicalInputSourceStats stats_{};
    std::atomic<std::uint8_t> currentBatteryPercent_{100};
    std::atomic<std::uint8_t> currentChargeState_{0};
    std::chrono::steady_clock::time_point lastBatteryQuery_{};
};

} // namespace

std::unique_ptr<PhysicalInputSource> openPhysicalInputSource(
    const HidDeviceInfo& apexVendorInterface,
    std::optional<unsigned int> requestedXInputIndex,
    std::string& error,
    bool vaderOperatorReport) {
    if (!requestedXInputIndex) {
        if (flydigi::apex6::isProduct(
                apexVendorInterface.vendorId, apexVendorInterface.productId)) {
            std::string apex6Error;
            auto mappedGamepad = HidPhysicalInputSource::open(
                apexVendorInterface, apex6Error);
            if (mappedGamepad) {
                std::string xinputError;
                auto xinputGamepad = openXInputGamepadForDevice(
                    apexVendorInterface.vendorId,
                    apexVendorInterface.productId,
                    std::nullopt, xinputError);
                if (!xinputGamepad) {
                    error = "Apex 6 independent trigger input unavailable (" +
                            xinputError + ").";
                    return {};
                }
                error.clear();
                return std::make_unique<Apex6HybridPhysicalInputSource>(
                    std::move(mappedGamepad), std::move(xinputGamepad));
            }
            error = "Apex 6 mapped HID input unavailable (" + apex6Error + "); ";
        }

        if (apexVendorInterface.vendorId == flydigi::kVendorId &&
            flydigi::isControllerProduct(apexVendorInterface.productId) &&
            !flydigi::apex6::isProduct(
                apexVendorInterface.vendorId, apexVendorInterface.productId)) {
            std::string apex5Error;
            auto apex5 = Apex5DualHidPhysicalInputSource::open(
                apexVendorInterface, apex5Error, vaderOperatorReport);
            if (apex5) {
                error.clear();
                return apex5;
            }
            error = "Apex 5 V2 vendor input unavailable (" + apex5Error + "); ";
        }

        if (flydigi::isApex4Product(
                apexVendorInterface.vendorId, apexVendorInterface.productId)) {
            std::string apex4Error;
            auto apex4 = Apex4PhysicalInputSource::open(
                apexVendorInterface, apex4Error);
            if (apex4) {
                error.clear();
                return apex4;
            }
            error = "Apex 4 V1 input unavailable (" + apex4Error + "); ";
        }

        std::string hidError;
        auto hid = HidPhysicalInputSource::open(apexVendorInterface, hidError);
        if (hid) {
            error.clear();
            return hid;
        }
        error += "APEX HID input unavailable (" + hidError +
                 "); trying XInput fallback.";
    }

    std::string xinputError;
    auto xinput = openXInputGamepadForDevice(
        apexVendorInterface.vendorId, apexVendorInterface.productId,
        requestedXInputIndex, xinputError);
    if (!xinput) {
        if (!error.empty()) error += ' ';
        error += "XInput fallback unavailable: " + xinputError;
        return {};
    }
    return std::make_unique<XInputPhysicalInputSource>(std::move(xinput));
}

} // namespace asb::platform

#endif // _WIN32
