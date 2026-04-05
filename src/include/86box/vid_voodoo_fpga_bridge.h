/*
 * vid_voodoo_fpga_bridge.h — 86Box device that bridges PCI transactions
 *                            to an external SST-1 FPGA model via shared
 *                            memory IPC.
 *
 * Instead of 86Box's software Voodoo emulation, this device forwards all
 * PCI config and memory-mapped register accesses to a Verilator simulation
 * of the real SST-1 (FBI) PCI target controller.
 *
 * The shared memory protocol is defined in sst1_bridge_protocol.h.
 */
#ifndef VID_VOODOO_FPGA_BRIDGE_H
#define VID_VOODOO_FPGA_BRIDGE_H

#include <86box/device.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const device_t voodoo_fpga_bridge_device;

#ifdef __cplusplus
}
#endif

#endif /* VID_VOODOO_FPGA_BRIDGE_H */
