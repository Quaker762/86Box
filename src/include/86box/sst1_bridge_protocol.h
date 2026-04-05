/*
 * sst1_bridge_protocol.h — Shared-memory protocol between 86Box and the
 *                           SST-1 Verilator testbench.
 *
 * Inspired by voodoo-fpga-public's PCem bridge architecture.
 *
 * Architecture:
 *   86Box (emulator) ←→ shared memory ←→ Verilator testbench process
 *
 *   86Box replaces its software Voodoo emulation with a bridge that forwards
 *   PCI configuration and memory-mapped I/O to the Verilator testbench via
 *   a shared-memory ring buffer and direct PCI request/response channel.
 *
 * Two communication mechanisms:
 *   1. Direct PCI channel: For immediate config reads/writes and register
 *      reads. 86Box writes a request and spins until the testbench responds.
 *   2. Command ring buffer: For bulk writes (LFB, texture, FIFO commands)
 *      that don't need an immediate response. Lock-free SPSC ring.
 */

#ifndef SST1_BRIDGE_PROTOCOL_H
#define SST1_BRIDGE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
#include <atomic>
#define SST1_ATOMIC(T) std::atomic<T>
#else
#include <stdatomic.h>
#define SST1_ATOMIC(T) _Atomic T
#endif

/* ========================================================================= */
/* Constants                                                                  */
/* ========================================================================= */

#define SST1_BRIDGE_SHM_NAME     "/sst1_fpga_bridge"
#define SST1_BRIDGE_MAGIC        0x53535431  /* "SST1" */
#define SST1_BRIDGE_CHANNEL_MAGIC 0x42524447  /* "BRDG" */

/* Ring buffer size (must be power of 2) */
#define SST1_CMD_RING_SIZE       4096
#define SST1_CMD_RING_MASK       (SST1_CMD_RING_SIZE - 1)

/* ========================================================================= */
/* Direct PCI request types                                                   */
/* ========================================================================= */

#define SST1_PCI_REQ_NONE          0
#define SST1_PCI_REQ_CONFIG_READ   1
#define SST1_PCI_REQ_CONFIG_WRITE  2
#define SST1_PCI_REQ_READ32        3
#define SST1_PCI_REQ_WRITE32       4
#define SST1_PCI_REQ_READ16        5
#define SST1_PCI_REQ_WRITE16       6

/* ========================================================================= */
/* Ring buffer command types                                                  */
/* ========================================================================= */

#define SST1_CMD_NOP       0
#define SST1_CMD_WRITE32   1
#define SST1_CMD_WRITE16   2
#define SST1_CMD_WRITE8    3

/* ========================================================================= */
/* Ring buffer command entry (16 bytes, cache-line friendly)                  */
/* ========================================================================= */

typedef struct {
    uint32_t cmd;      /* SST1_CMD_xxx */
    uint32_t addr;     /* 24-bit SST address (within 16MB window) */
    uint32_t data;     /* Write data */
    uint32_t flags;    /* Reserved / future use */
} sst1_bridge_cmd_t;

/* ========================================================================= */
/* Shared memory channel layout                                               */
/* ========================================================================= */

typedef struct {
    /* -- Header (verified on connection) -- */
    uint32_t magic;                             /* Must be SST1_BRIDGE_CHANNEL_MAGIC */

    /* -- Connection state -- */
    SST1_ATOMIC(uint32_t) emu_connected;        /* 86Box sets to 1 on init */
    SST1_ATOMIC(uint32_t) tb_connected;         /* Testbench sets to 1 on init */
    SST1_ATOMIC(uint32_t) emu_session_id;       /* Unique per 86Box session */

    /* -- Direct PCI transaction channel -- */
    SST1_ATOMIC(uint32_t) pci_request;          /* SST1_PCI_REQ_xxx (written by 86Box) */
    SST1_ATOMIC(uint32_t) pci_cfg_addr;         /* Config byte address (for config r/w) */
    SST1_ATOMIC(uint32_t) pci_addr;             /* Memory address (for mem r/w) */
    SST1_ATOMIC(uint32_t) pci_wdata;            /* Write data */
    SST1_ATOMIC(uint32_t) pci_rdata;            /* Read data (written by testbench) */
    SST1_ATOMIC(uint32_t) pci_done;             /* Testbench sets to 1 when complete */

    /* -- RTL status (continuously updated by testbench) -- */
    SST1_ATOMIC(uint32_t) rtl_status;           /* SST-1 status register value */

    /* -- Command ring buffer (SPSC: 86Box produces, testbench consumes) -- */
    SST1_ATOMIC(uint32_t) cmd_head;             /* Producer index (86Box) */
    SST1_ATOMIC(uint32_t) cmd_tail;             /* Consumer index (testbench) */

    /* -- Statistics -- */
    SST1_ATOMIC(uint64_t) total_writes;
    SST1_ATOMIC(uint64_t) total_reads;

    /* -- Framebuffer export (written by testbench for display scanout) -- */
    SST1_ATOMIC(uint32_t) fb_width;             /* Display width (pixels) */
    SST1_ATOMIC(uint32_t) fb_height;            /* Display height (lines) */
    SST1_ATOMIC(uint32_t) fb_stride;            /* Row stride in 16-bit pixels */
    SST1_ATOMIC(uint32_t) fb_ready;             /* Set to 1 when fb_pixels is valid */

    /* -- Padding to cache-line boundary -- */
    uint8_t _pad[48];

    /* -- Ring buffer entries -- */
    sst1_bridge_cmd_t ring[SST1_CMD_RING_SIZE];

    /* -- Framebuffer pixel data (RGB565, up to 1024x768) -- */
    uint16_t fb_pixels[1024 * 768];

} sst1_bridge_channel_t;

/* ========================================================================= */
/* Ring buffer helpers (lock-free SPSC)                                       */
/*                                                                            */
/* Use wrapper macros for C/C++ atomic compatibility.                         */
/* ========================================================================= */

#ifdef __cplusplus
#define SST1_ATOMIC_LOAD(ptr)          (ptr)->load(std::memory_order_relaxed)
#define SST1_ATOMIC_STORE(ptr, val)    (ptr)->store((val), std::memory_order_relaxed)
#define SST1_FENCE_RELEASE()           std::atomic_thread_fence(std::memory_order_release)
#define SST1_FENCE_ACQUIRE()           std::atomic_thread_fence(std::memory_order_acquire)
#else
#define SST1_ATOMIC_LOAD(ptr)          atomic_load(ptr)
#define SST1_ATOMIC_STORE(ptr, val)    atomic_store(ptr, val)
#define SST1_FENCE_RELEASE()           atomic_thread_fence(memory_order_release)
#define SST1_FENCE_ACQUIRE()           atomic_thread_fence(memory_order_acquire)
#endif

static inline int sst1_ring_push(sst1_bridge_channel_t* ch,
                                  const sst1_bridge_cmd_t* cmd)
{
    uint32_t head = SST1_ATOMIC_LOAD(&ch->cmd_head);
    uint32_t next = (head + 1) & SST1_CMD_RING_MASK;
    if (next == SST1_ATOMIC_LOAD(&ch->cmd_tail))
        return 0;  /* Full */
    ch->ring[head] = *cmd;
    SST1_FENCE_RELEASE();
    SST1_ATOMIC_STORE(&ch->cmd_head, next);
    return 1;
}

static inline int sst1_ring_pop(sst1_bridge_channel_t* ch,
                                 sst1_bridge_cmd_t* cmd)
{
    uint32_t tail = SST1_ATOMIC_LOAD(&ch->cmd_tail);
    if (tail == SST1_ATOMIC_LOAD(&ch->cmd_head))
        return 0;  /* Empty */
    SST1_FENCE_ACQUIRE();
    *cmd = ch->ring[tail];
    SST1_ATOMIC_STORE(&ch->cmd_tail, (tail + 1) & SST1_CMD_RING_MASK);
    return 1;
}

#endif /* SST1_BRIDGE_PROTOCOL_H */
