#ifdef NDEBUG
#undef NDEBUG
#endif

// Locks the wire contract taken from VykosX/rewasd-xinput-hide-reverse. The
// numeric IOCTL is the part that fails silently in the field: a mismatched
// DeviceType or Function still opens a handle, and the driver answers every
// request with STATUS_INVALID_DEVICE_REQUEST, which surfaces only as "hide
// refused". reference/protocol.md states the value explicitly, so it is
// asserted here rather than derived from a macro we do not own.

#include "platform/HideFilterIsolation.h"

#include <cassert>
#include <cstdint>

namespace {

// The definition from the DDK, reproduced so the assertion is independent of
// the implementation's own arithmetic.
constexpr std::uint32_t ctlCode(std::uint32_t deviceType, std::uint32_t function,
                                std::uint32_t method, std::uint32_t access) {
    return (deviceType << 16) | (access << 14) | (function << 2) | method;
}

}  // namespace

int main() {
    using namespace asb::platform;

    // reference/protocol.md: CTL_CODE(0x002A, 0xA78, METHOD_BUFFERED,
    //   FILE_READ_DATA | FILE_WRITE_DATA), "numerically: 0x002AE9E0".
    constexpr std::uint32_t kExpected = 0x002AE9E0u;
    static_assert(ctlCode(0x002Au, 0xA78u, 0u, 0x03u) == kExpected,
                  "the reproduced CTL_CODE does not match the published value");
    assert(detail::hideFilterTogglePresenceIoctl() == kExpected);

    // Documented input polarity: 0 hides, any other value shows. Inverting this
    // is the single easiest way to make the driver do the opposite of what the
    // bridge asks.
    assert(detail::hideFilterPresenceByte(true) == 0);
    assert(detail::hideFilterPresenceByte(false) != 0);

    // The published placeholder interface GUID, so a stock driver build is the
    // one this client talks to.
    const void* guid = detail::hideFilterInterfaceGuid();
    assert(guid != nullptr);

    // With no driver installed the client must report that cleanly instead of
    // pretending an isolation is active.
    assert(!HideFilterIsolation::available() ||
           !HideFilterIsolation::enumerate().empty());

    asb::HidDeviceInfo missing{};
    HideFilterIsolation isolation;
    assert(!isolation.active());
    std::string error;
    assert(!isolation.activate(missing, error));
    assert(!error.empty());
    // restore() is idempotent and must succeed when nothing was activated.
    assert(isolation.restore(error));
    assert(isolation.restore(error));
    assert(!isolation.active());

    return 0;
}
