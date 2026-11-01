
// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <utility>

#include <fmt/format.h>

#include "common/common_types.h"
#include "common/logging.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/hle/kernel/k_event.h"
#include "core/hle/kernel/k_process.h"
#include "core/hle/kernel/k_readable_event.h"
#include "core/hle/kernel/k_transfer_memory.h"
#include "core/hle/result.h"
#include "core/memory.h"
#include "core/hle/service/hle_ipc.h"
#include "core/hle/service/ipc_helpers.h"
#include "core/hle/service/kernel_helpers.h"
#include "core/hle/service/nvdrv/nvdrv_interface.h"
#include "core/hle/service/server_manager.h"
#include "core/hle/service/service.h"
#include "core/hle/service/sm/sm.h"
#include "video_core/host1x/host1x.h"
#include "video_core/host1x/syncpoint_manager.h"

namespace Service {

// TIPC replies contain a one-word result followed directly by their data words.
// ResponseBuilder::Push(Result) writes a two-word CMIF result, which shifts TIPC data.
inline void TipcReplyWords(HLERequestContext& ctx, std::span<const u32> words) {
    IPC::ResponseBuilder rb{ctx, static_cast<u32>(2 + words.size())};
    rb.PushRaw<u32>(ResultSuccess.raw);
    for (const u32 word : words) {
        rb.Push<u32>(word);
    }
}

inline void TipcReplyWords(HLERequestContext& ctx, std::initializer_list<u32> words) {
    TipcReplyWords(ctx, std::span<const u32>{words.begin(), words.size()});
}

inline void TipcReplyError(HLERequestContext& ctx, Result result) {
    IPC::ResponseBuilder rb{ctx, 2};
    rb.PushRaw<u32>(result.raw);
}

// Creates a new server-side interface and returns its client session as a copy handle.
template <class T>
inline void ReturnRawSession(HLERequestContext& ctx, std::shared_ptr<T> iface) {
    ctx.AddCopyInterface(std::move(iface));
    IPC::ResponseBuilder rb{ctx, 2, 1, 0};
    rb.Push(ResultSuccess);
}

// Switch 2 syncpoint service returned by nx2:gsv command 202.
class INx2SyncptSession final : public ServiceFramework<INx2SyncptSession> {
public:
    explicit INx2SyncptSession(Core::System& system_)
        : ServiceFramework{system_, "nx2:syncpt"},
          m_context{system_, "nx2:syncpt"} {
        RegisterHandlersTipc({
            {0, &INx2SyncptSession::Handshake, "Handshake"},
            {1, &INx2SyncptSession::GetCount, "GetCount"},
            {22, &INx2SyncptSession::GetSlotEvent, "GetSlotEvent"},
            {103, &INx2SyncptSession::QueryCounter, "QueryCounter"},
            {201, &INx2SyncptSession::RequestSlotEvent, "RequestSlotEvent"},
            {202, &INx2SyncptSession::CancelSlotEvent, "CancelSlotEvent"},
        });
    }

    ~INx2SyncptSession() override {
        {
            std::scoped_lock lock{m_shared->mutex};
            m_shared->alive = false;
        }

        for (auto& [slot, request] : m_requests) {
            Cancel(request);
        }

        for (auto& [slot, event] : m_events) {
            m_context.CloseEvent(event);
        }
    }

private:
    static constexpr u32 HostSyncpoints = 192;

    struct SlotRequest {
        u32 syncpoint{};
        std::atomic<bool> pending{};
        Tegra::Host1x::SyncpointManager::ActionHandle handle{};
    };

    struct SharedState {
        std::mutex mutex;
        bool alive{true};
    };

    void Handshake(HLERequestContext& ctx) {
        const auto* words = ctx.CommandBuffer();
        TipcReplyWords(ctx, {words[2], words[3]});
    }

    void GetCount(HLERequestContext& ctx) {
        LOG_WARNING(Service, "(STUBBED) nx2:syncpt TIPC 1, returning 32");
        TipcReplyWords(ctx, {32});
    }

    void QueryCounter(HLERequestContext& ctx) {
        const u32 syncpoint = ~ctx.CommandBuffer()[2];
        const u32 value = syncpoint < HostSyncpoints
                              ? system.Host1x().GetSyncpointManager().GetGuestSyncpointValue(
                                    syncpoint)
                              : 0;

        LOG_DEBUG(Service, "nx2:syncpt 103 syncpoint={} value={:#x}", syncpoint, value);
        TipcReplyWords(ctx, {value, 0, 0});
    }

    Kernel::KEvent* GetEvent(u32 slot) {
        auto*& event = m_events[slot];
        if (event == nullptr) {
            event = m_context.CreateEvent(fmt::format("nx2:syncpt:Slot{:x}", slot));
        }
        return event;
    }

    void GetSlotEvent(HLERequestContext& ctx) {
        const u32 slot = ctx.CommandBuffer()[2];
        auto* event = GetEvent(slot);

        IPC::ResponseBuilder rb{ctx, 2, 1, 0};
        rb.Push(ResultSuccess);
        rb.PushCopyObjects(ctx, event->GetReadableEvent());
    }

    void RequestSlotEvent(HLERequestContext& ctx) {
        const auto* words = ctx.CommandBuffer();
        const u32 slot = words[2];
        const u32 syncpoint = ~words[3];
        const u64 value = u64{words[4]} | (u64{words[5]} << 32);
        const u32 mode = words[6];

        LOG_DEBUG(Service,
                  "nx2:syncpt TIPC 201 slot={:#x} syncpoint={} value={:#x} mode={}",
                  slot, syncpoint, value, mode);

        auto* event = GetEvent(slot);

        // Jadyx's Host1x guest-action interface compares 32-bit counters.
        // Do not silently treat a requested 64-bit comparison as a 32-bit one.
        if (mode != 0) {
            TipcReplyError(ctx, ResultUnknown);
            return;
        }

        if (syncpoint >= HostSyncpoints) {
            LOG_WARNING(Service, "nx2:syncpt syncpoint {} is out of range; signalling slot",
                        syncpoint);
            event->Signal(system.Kernel());
            TipcReplyWords(ctx, {});
            return;
        }

        auto& request = m_requests[slot];
        Cancel(request);

        event->Clear(system.Kernel());
        request.syncpoint = syncpoint;
        request.pending = true;

        auto& manager = system.Host1x().GetSyncpointManager();
        request.handle = manager.RegisterGuestAction(
            syncpoint, static_cast<u32>(value),
            [shared = m_shared, &kernel = system.Kernel(), &request, event] {
                std::scoped_lock lock{shared->mutex};
                if (!shared->alive) {
                    return;
                }

                request.pending = false;
                event->Signal(kernel);
            });

        TipcReplyWords(ctx, {});
    }

    void CancelSlotEvent(HLERequestContext& ctx) {
        const u32 slot = ctx.CommandBuffer()[2];

        if (auto it = m_requests.find(slot); it != m_requests.end()) {
            Cancel(it->second);
        }

        // The SDK waits on the slot event after cancellation, then rechecks the counter.
        GetEvent(slot)->Signal(system.Kernel());
        TipcReplyWords(ctx, {});
    }

    bool Cancel(SlotRequest& request) {
        if (request.pending.exchange(false)) {
            system.Host1x().GetSyncpointManager().DeregisterGuestAction(
                request.syncpoint, request.handle);
            return true;
        }
        return false;
    }

    KernelHelpers::ServiceContext m_context;
    std::map<u32, Kernel::KEvent*> m_events;
    std::map<u32, SlotRequest> m_requests;
    std::shared_ptr<SharedState> m_shared{std::make_shared<SharedState>()};
};

// Switch 2 graphics-system service. This stage implements service bootstrap and
// syncpoint shared-memory plumbing; it does not provide a Switch 2 GPU renderer.
class INx2GraphicsSystemSession final
    : public ServiceFramework<INx2GraphicsSystemSession> {
public:
    explicit INx2GraphicsSystemSession(Core::System& system_)
        : ServiceFramework{system_, "nx2:gsv"},
          m_context{system_, "nx2:gsv"} {
        RegisterHandlersTipc({
            {4, &INx2GraphicsSystemSession::Stub, "Boot"},
            {5, &INx2GraphicsSystemSession::Stub, "Unknown5"},
            {101, &INx2GraphicsSystemSession::Stub, "Unknown101"},
            {202, &INx2GraphicsSystemSession::OpenSyncptSession, "OpenSyncptSession"},
            {204, &INx2GraphicsSystemSession::AttachSyncptMemory, "AttachSyncptMemory"},
            {1402, &INx2GraphicsSystemSession::OpenNvsrvSession, "OpenNvsrvSession"},
        });
    }

    ~INx2GraphicsSystemSession() override {
        if (m_syncpt_poll) {
            system.CoreTiming().UnscheduleEvent(m_syncpt_poll);
        }

        for (auto* event : m_syncpt_events) {
            if (event != nullptr) {
                m_context.CloseEvent(event);
            }
        }
    }

private:
    static constexpr u32 SyncptValuesOffset = 0x40;
    static constexpr u32 SyncptRoSummaryOffset = 0x1040;
    static constexpr u32 SyncptRoBitsOffset = 0x1100;
    static constexpr u32 SyncptRwSummaryOffset = 0x0;
    static constexpr u32 SyncptRwBitsOffset = 0x100;
    static constexpr u32 HostSyncpoints = 192;

    static constexpr std::chrono::nanoseconds SyncptPollInterval{
        std::chrono::microseconds{500}};

    // TIPC requests may contain a special header followed by copy/move handle words.
    static size_t TipcDataOffset(const u32* command_buffer) {
        if ((command_buffer[1] & 0x80000000) == 0) {
            return 2;
        }

        const u32 descriptor = command_buffer[2];
        return 3 + ((descriptor & 1) ? 2 : 0) +
               ((descriptor >> 1) & 0xF) + ((descriptor >> 5) & 0xF);
    }

    void Stub(HLERequestContext& ctx) {
        const auto* words = ctx.CommandBuffer();
        LOG_WARNING(Service,
                    "(STUBBED) nx2:gsv TIPC {} [2]={:#x} [3]={:#x} [4]={:#x} [5]={:#x}",
                    ctx.GetCommand(), words[2], words[3], words[4], words[5]);

        IPC::ResponseBuilder rb{ctx, 2};
        rb.Push(ResultSuccess);
    }

    void OpenSyncptSession(HLERequestContext& ctx) {
        LOG_WARNING(Service, "(STUBBED) nx2:gsv TIPC 202, returning nx2:syncpt");
        ReturnRawSession(ctx, std::make_shared<INx2SyncptSession>(system));
    }

    void AttachSyncptMemory(HLERequestContext& ctx) {
        const auto* command_buffer = ctx.CommandBuffer();
        const size_t data_offset = TipcDataOffset(command_buffer);
        const u32 version_requested = command_buffer[data_offset];
        const u32 max_syncpoints =
            std::min<u32>(command_buffer[data_offset + 1], (0x1000 - 0x40) / 8);

        LOG_WARNING(Service,
                    "(STUBBED) nx2:gsv TIPC 204 version={} max_syncpoints={}",
                    version_requested, max_syncpoints);

        auto rw = ctx.GetObjectFromHandle<Kernel::KTransferMemory>(ctx.GetCopyHandle(0));
        auto ro = ctx.GetObjectFromHandle<Kernel::KTransferMemory>(ctx.GetCopyHandle(1));

        if (rw.IsNull() || ro.IsNull() || rw->GetSize() < 0x1000 ||
            ro->GetSize() < 0x2000 || rw->GetOwner() == nullptr ||
            ro->GetOwner() == nullptr || rw->GetOwner() != ro->GetOwner() ||
            rw->GetOwner() != ctx.GetThread().GetOwnerProcess()) {
            LOG_ERROR(Service, "nx2:gsv TIPC 204 received invalid transfer memory");
            TipcReplyError(ctx, ResultUnknown);
            return;
        }

        m_syncpt_memory = &ro->GetOwner()->GetMemory();
        m_syncpt_rw = GetInteger(rw->GetSourceAddress());
        m_syncpt_ro = GetInteger(ro->GetSourceAddress());
        m_syncpt_count = max_syncpoints;

        auto& memory = *m_syncpt_memory;
        memory.Write32(m_syncpt_ro + 0x00, 1);
        memory.Write32(m_syncpt_ro + 0x04, 0);
        memory.Write32(m_syncpt_ro + 0x08, max_syncpoints);
        memory.Write32(m_syncpt_ro + 0x0c, SyncptValuesOffset);
        memory.Write32(m_syncpt_ro + 0x10, SyncptRwSummaryOffset);
        memory.Write32(m_syncpt_ro + 0x14, SyncptRoSummaryOffset);
        memory.Write32(m_syncpt_ro + 0x18, SyncptRwBitsOffset);
        memory.Write32(m_syncpt_ro + 0x1c, SyncptRoBitsOffset);

        UpdateSyncptValues();

        // Create each event independently so a failed creation can be retried.
        if (m_syncpt_events[0] == nullptr) {
            m_syncpt_events[0] = m_context.CreateEvent("nx2:gsv:SyncptRequest");
        }
        if (m_syncpt_events[1] == nullptr) {
            m_syncpt_events[1] = m_context.CreateEvent("nx2:gsv:SyncptNotify");
        }

        if (!m_syncpt_poll) {
            m_syncpt_poll = Core::Timing::CreateEvent(
                "nx2:gsv:SyncptPoll",
                [this](s64, std::chrono::nanoseconds)
                    -> std::optional<std::chrono::nanoseconds> {
                    PollSyncpts();
                    return std::nullopt;
                });

            system.CoreTiming().ScheduleLoopingEvent(
                SyncptPollInterval, SyncptPollInterval, m_syncpt_poll);
        }

        IPC::ResponseBuilder rb{ctx, 2, 2, 0};
        rb.Push(ResultSuccess);
        rb.PushCopyObjects(ctx, *m_syncpt_events[0]);
        rb.PushCopyObjects(ctx, m_syncpt_events[1]->GetReadableEvent());
    }

    void UpdateSyncptValues() {
        if (m_syncpt_memory == nullptr) {
            return;
        }

        auto& manager = system.Host1x().GetSyncpointManager();
        for (u32 id = 0; id < std::min(m_syncpt_count, HostSyncpoints); ++id) {
            const u64 value = manager.GetGuestSyncpointValue(id);
            const u64 address = m_syncpt_ro + SyncptValuesOffset + id * 8;
            m_syncpt_memory->Write64(address, value);
        }
    }

    void PollSyncpts() {
        if (m_syncpt_memory == nullptr || m_syncpt_events[1] == nullptr) {
            return;
        }

        UpdateSyncptValues();

        bool acknowledged = false;
        const u32 words = (m_syncpt_count + 63) / 64;

        for (u32 index = 0; index < words; ++index) {
            const u64 rw_bits =
                m_syncpt_memory->Read64(m_syncpt_rw + SyncptRwBitsOffset + index * 8);
            const u64 ro_address = m_syncpt_ro + SyncptRoBitsOffset + index * 8;

            if (m_syncpt_memory->Read64(ro_address) != rw_bits) {
                m_syncpt_memory->Write64(ro_address, rw_bits);
                acknowledged = true;
            }
        }

        m_syncpt_memory->Write64(
            m_syncpt_ro + SyncptRoSummaryOffset,
            m_syncpt_memory->Read64(m_syncpt_rw + SyncptRwSummaryOffset));

        if (acknowledged) {
            m_syncpt_events[1]->Signal(system.Kernel());
        }
    }

    void OpenNvsrvSession(HLERequestContext& ctx) {
        LOG_WARNING(Service, "(STUBBED) nx2:gsv TIPC 1402, returning nvdrv-backed nx2:nvsrv");

        auto service = system.ServiceManager().GetService<Nvidia::NVDRV>("nvdrv", true);
        if (!service) {
            TipcReplyError(ctx, ResultUnknown);
            return;
        }

        auto* process = ctx.GetThread().GetOwnerProcess();
        if (process == nullptr) {
            TipcReplyError(ctx, ResultUnknown);
            return;
        }

        auto nvsrv = std::make_shared<Nvidia::NVDRV>(
            system, service->GetModule(), "nx2:nvsrv");
        nvsrv->InitializeForProcess(process);
        ReturnRawSession(ctx, std::move(nvsrv));
    }

    KernelHelpers::ServiceContext m_context;
    std::array<Kernel::KEvent*, 2> m_syncpt_events{};
    Core::Memory::Memory* m_syncpt_memory{};
    u64 m_syncpt_rw{};
    u64 m_syncpt_ro{};
    u32 m_syncpt_count{};
    std::shared_ptr<Core::Timing::EventType> m_syncpt_poll;
};

} // namespace Service
