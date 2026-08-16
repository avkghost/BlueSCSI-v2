# PIO-accelerated SCSI initiator write

- [x] Add `scsi_host_async_write` + `scsi_host_async_write_wide` to `scsi_accel_host_RP2MCU.pio`
- [x] Regenerate `.pio.h` via `build-ultra-clean/pioasm/pioasm`
- [x] Declare `scsi_accel_host_write` in `scsi_accel_host.h`
- [x] Add `SCSIHOST_WRITE` state, `config_gpio()` WRITE branch (PIO0 mux, pindirs, DATA_DIR high)
- [x] Implement `scsi_accel_host_write()` (TX FIFO push loop, RX completion marker wait, abort handling)
- [x] Wire init: load write program (#ifdef wide), patch REQ waits at +3/+14, build write pio_cfg
- [x] Route `scsiHostWrite()` to accel (narrow any count, wide even count)
- [x] Build both main + bootloader cleanly (pio0 = read 7 + write 18 = 25 ≤ 32)

- [x] Hardware test on Jaz: write 313 kB/s (bit-bang) → 442 kB/s (PIO accel, +41%), read unchanged 668 kB/s
- [x] Commit after test passes (branch `avkghost/birdge-with-daynaport`)
- [ ] Wide path is compile-only (no wide board available)

# DaynaPORT Enable Trace
- [x] Make the Dayna quiet window adaptive when USB bus reset storms repeat.
- [x] Extend the firmware-side Dayna quiet window after USB bus reset and unplug events.
- [x] Narrow the Linux 7.x diagnostic fork to log only the relevant DaynaPORT opcodes.
- [x] Rebuild the Linux 7.x modules and confirm the diagnostic fork still compiles cleanly.

- [x] Add a one-shot host-side log for the first READ(6) after a successful 0x0E enable.
- [x] Add a matching target-side log for the first READ(6) after enable, including phase and status.
- [x] Rebuild the firmware and confirm the instrumentation compiles cleanly.
- [x] Allow pre-enable broadcast/multicast/self-directed Dayna frames to queue instead of being dropped.
- [x] Restore the fixed BlueSCSI USB serial placeholder.
- [x] Trace USB re-enumeration and reboot paths that can trigger the scsilink reset loop.
- [x] Log the boot reset cause before USB init to distinguish watchdog reboot from host USB resets.
- [x] Persist hardfault breadcrumbs across reboot via watchdog scratch registers.
- [x] Log whether a queued Dayna frame was accepted while disabled because it was broadcast, multicast, or self-directed.
- [x] Make the DaynaPORT `0x0e` toggle path explicitly return `GOOD/STATUS`, matching the other command handlers.
- [x] Trace TinyUSB mount/unmount/suspend/resume callbacks to correlate host USB resets with the device-side state.
- [x] Trace TinyUSB reset and CDC line-state callbacks to catch reset storms that do not show up as a clean unmount.
- [x] Trace TinyUSB bus-reset/unplug/suspend/resume events via `tud_event_hook_cb`.
- [x] Reduce the USB lifecycle traces to first-hit-only so the reset storm stays readable.
- [x] Correct the TinyUSB event hook naming so `id=7` is logged as `xfer_complete`, not an unknown event.

## Review

- `cmake --build build-ultra-clean -j2` completed successfully after the trace additions.
- The Dayna RX gate now allows broadcast, multicast, and frames addressed to the Dayna MAC before full enable.
- The USB serial descriptor is back to the fixed placeholder value to reduce enumeration variables.
- USB restart points now log before re-enumeration and MCU reboot, so reset loops can be correlated with the host-side disconnects.
- Boot now logs the watchdog reset reason and mass-storage scratch marker before USB enumeration starts.
- Hardfaults now leave a scratch-register breadcrumb that the next boot prints and clears.
- The queue path now distinguishes disabled-frame acceptance from disabled-frame drops, which should help separate “enable never happened” from “enable happened but the host stack still emitted self-directed traffic.”
- The `0x0e` toggle handler now explicitly terminates in `GOOD/STATUS` instead of relying on prior state.
- USB mount and unmount callbacks now log, which should line up the host-side resets with the device-side enumeration churn.
- USB reset and CDC line-state callbacks now log, which should catch xHCI reset churn even when the device never cleanly unmounts.
- The TinyUSB event hook now logs bus-reset and unplug events, which should finally expose the host reset storm at the device boundary.
- The USB lifecycle traces are now first-hit-only to avoid flooding the capture while still preserving the boundary events.
- TinyUSB event id 7 is `xfer_complete`; the hook now names it correctly.
- The Linux 7.x diagnostic fork now logs only the relevant DaynaPORT command/result pairs at probe and runtime without changing the normal driver path.
- `make -C /lib/modules/$(uname -r)/build M=/home/andy/Projects/Hardware/daynaport-scsilink-linux-driver/linux-7.0 modules` completed successfully after the diagnostic logging was narrowed.
- The diagnostic fork is now narrowed to the DaynaPORT opcodes that matter for this fault path: `0x08`, `0x09`, `0x0A`, and `0x0E`.
- The Linux 7.x module build still passes after that filter was tightened.
- USB bus reset and unplug events now extend the Dayna quiet window, giving the host time to settle before bridge traffic resumes.
- The firmware build still completes successfully after the USB-reset quiet-window change.
- Repeated USB bus resets now back off the Dayna quiet window exponentially up to a capped maximum, and resets within a 30s storm window keep the backoff escalating instead of dropping back to the minimum.
- The firmware build still passes after the adaptive backoff change.
- The Dayna quiet window now suppresses bridge RX/TX during the unstable period instead of only logging that it is active.
- The firmware build still passes after the quiet-window suppression gate change.
- The reset-storm gate now waits for a successful Dayna enable, and pre-enable USB resets are logged once instead of being treated as the active failure path.
- The Linux 7.x diagnostic fork now logs the first transport error after a failed Dayna enable, which should line up with the first USB reset/disconnect in the host log.
- The Linux 7.x diagnostic fork now logs the first transmit and first TX timeout after a failed Dayna enable, giving a last-clean host-side breadcrumb before the reset storm.
- The Linux 7.x diagnostic fork now logs the first RX poll after a failed Dayna enable, so we can see whether the host starts churning while reads are still clean.
- The Linux 7.x diagnostic fork now logs the first malformed Dayna READ header after a failed enable, including the bad offset and raw header bytes.
- The first malformed Dayna READ header is a 60-byte ARP frame (`00 3c 00 00 00 00`), which the parser currently rejects because it expects the length field to include the FCS.
- The Linux 7.x module build still passes after the extra diagnostic state was added.
- The firmware build still passes after the current round of USB reset gating and quiet-window changes.
- The latest log still shows `userif-2: sent link down/up event` immediately before the USB reset storm, which points to host-side VMware/vmnet churn rather than Dayna packet fragmentation.
- [x] Update the shared DaynaPORT RX parser to accept the exact 60-byte short-frame form without FCS while still rejecting 61-63 byte runt records.
- [x] Add a host-side regression test for the 60-byte short-frame case so it stays covered.

## DaynaPORT Linux Driver: 4-Phase Port Plan (complete)

Repo: `/home/andy/Projects/Hardware/daynaport-scsilink-linux-driver`

- [x] Phase 1 — driver diagnostics consolidation behind global `debug` bitmask
      (1 = per-READ RX stats, 2 = Dayna trace + one-shot fault breadcrumbs,
      4 = per-command CDB/result trace); `scsilink_diag.ko` kept as a live
      build variant (bits 2+4 forced) so the two cannot drift.
      Commit `6d5b4ab` (via `08d024c`).
- [x] Phase 2 — BlueSCSI config-driven DaynaPORT raw-bridge initiator mode
      (BlueSCSI-v2) + `blueSCSI.config.example`. Commit `4358f313`.
- [x] Phase 3 — port USB-bridge chunked RX/TX to the 2.0 and 2.4 drivers:
      `usb_bridge=1` param, `daynaport_rx_chunk()` RX reassembly, completion-
      driven TX chunk chains (io tasklet on 2.4, immediate BH on 2.0),
      zero-length WRITE abort on mid-chain failure, README/CHANGES updated.
      Also fixed `daynaport_rx_parse()` diag-arg drift in both old drivers.
      Verified: `-fsyntax-only -Wall -Wextra` clean against kernel-API stubs,
      lib tests 116/0. Commits `6e06758`, `25aa515`.
- [x] Phase 4 — 7.0 poll loop: skip the interval sleep while the fast/idle
      backoff holds the fast rate in bridge mode (bounded by `fast_hold`),
      so inter-frame gaps cost one synchronous READ instead of a `poll0_ms`
      sleep (the ~0.5 Mbit/s ceiling). Commit `e18bdbe`.
- [x] Benchmark runbook written to `reference/benchmark.md` (commit `4e0a8c9`).

### Remaining (needs hardware)

- [x] Run the benchmark per `reference/benchmark.md` (iperf3 RX/TX pre/post
      Phase-4, debug=1 poll-cadence diagnostics, fast-window CPU cost, native
      SCSI regression). Requires the physical BlueSCSI + DaynaPORT bridge rig.
      **Parked**: partial measurement done (down 0.53 / up 1.19 Mbit/s,
      poll cadence captured); the block-I/O leg is unavailable and further
      network saturation tests are deferred indefinitely. Benchmark findings
      recorded in `daynaport-scsilink-linux-driver/reference/benchmark.md`.
