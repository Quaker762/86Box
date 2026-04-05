/*
 * vid_voodoo_fpga_bridge.c — 86Box PCI device that bridges to an external
 *                            SST-1 FPGA/Verilator model via shared memory.
 *
 * This replaces 86Box's software Voodoo emulation with a pass-through
 * that forwards all PCI configuration and memory-mapped accesses to the
 * real SST-1 RTL running in a Verilator co-simulation process.
 *
 * Protocol: see sst1_bridge_protocol.h
 *
 * Architecture:
 *   86Box process                         Verilator process
 *   ─────────────                         ──────────────────
 *   vid_voodoo_fpga_bridge.c              sst1_bridge.cpp
 *        │                                     │
 *        │   ┌──────────────────────┐          │
 *        └──▶│  shared memory (shm) │◀─────────┘
 *            │  sst1_bridge_channel │
 *            └──────────────────────┘
 *
 * Config reads/writes use the direct PCI channel (request/done handshake).
 * Memory writes use the SPSC ring buffer for bulk throughput.
 * Memory reads use the direct channel (blocking).
 *
 * Display path:
 *   The Verilator process copies the RTL framebuffer into shared memory
 *   (fb_pixels[], RGB565).  A timer-driven scanline callback in this file
 *   reads those pixels, converts them via video_16to32[] CLUT to ARGB32,
 *   and writes them into the 86Box monitor's target_buffer.  At vblank,
 *   svga_doblit() pushes the frame to the GUI.
 *
 * VGA passthrough:
 *   Like the real Voodoo 1, this device sits alongside a primary VGA card.
 *   When fbiInit0 bit 0 (VGA_PASS) is set, we take over the display via
 *   svga_set_override().  When clear, the VGA card drives the monitor.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __linux__
#    include <fcntl.h>
#    include <unistd.h>
#    include <sys/mman.h>
#    include <sys/stat.h>
#endif

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/pci.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/vid_svga.h>

/* The shared memory protocol header (C-compatible) */
#include <86box/sst1_bridge_protocol.h>

/* ========================================================================= */
/* SST-1 register offsets (within the 16 MB window)                          */
/* ========================================================================= */

/* fbiInit0 is at offset 0x210 in the register space */
#define SST_FBIINIT0           0x210
#define FBIINIT0_VGA_PASS      (1 << 0)
#define FBIINIT0_FBI_RESET     (1 << 1)

/* ========================================================================= */
/* Private device state                                                       */
/* ========================================================================= */

typedef struct voodoo_fpga_bridge_t {
    /* 86Box PCI slot assigned by pci_add_card() */
    uint8_t pci_slot;

    /* Memory mapping for the 16 MB register window */
    mem_mapping_t mapping;

    /* PCI config shadow registers (managed by RTL via bridge) */
    uint32_t mem_base_addr;   /* BAR0 value */
    int      pci_enable;      /* Command register bit 1 (memory space) */

    /* Shared memory IPC */
    int                      shm_fd;
    sst1_bridge_channel_t   *channel;

    /* VGA passthrough */
    svga_t  *svga;            /* Primary VGA card (from svga_get_pri()) */

    /* Display scanline timer */
    pc_timer_t timer;
    int        line;          /* Current scanline being drawn */
    int        v_disp;        /* Visible vertical lines */
    int        h_disp;        /* Visible horizontal pixels */
    int        v_total;       /* Total vertical lines (including blanking) */
    int        dirty_line_low;
    int        dirty_line_high;
    uint8_t    monitor_index;

    /* Shadow registers */
    uint32_t fbiInit0;

    /* RGB565 -> ARGB32 colour lookup table */
    uint32_t video_16to32[0x10000];

} voodoo_fpga_bridge_t;

/* Shared memory segment name (must match sst1_bridge.cpp) */
#define SST1_SHM_NAME  "/sst1_bridge"

/* SST-1 memory window size: 16 MB */
#define SST1_MEM_SIZE  0x01000000

/* Timeout for direct PCI requests (in busy-wait iterations) */
#define SST1_REQ_TIMEOUT  1000000

/* Default display dimensions (640x480 @ ~60 Hz) */
#define SST1_DEFAULT_H_DISP  640
#define SST1_DEFAULT_V_DISP  480
#define SST1_DEFAULT_V_TOTAL 525

/* Forward declarations */
static void fpga_recalc_mapping(voodoo_fpga_bridge_t *dev);
static void fpga_callback(void *priv);

/* ========================================================================= */
/* Shared memory helpers                                                      */
/* ========================================================================= */

static int
bridge_shm_open(voodoo_fpga_bridge_t *dev)
{
#ifdef __linux__
    dev->shm_fd = shm_open(SST1_SHM_NAME, O_RDWR, 0600);
    if (dev->shm_fd < 0) {
        /* If it doesn't exist, create it (the Verilator side may connect later) */
        dev->shm_fd = shm_open(SST1_SHM_NAME, O_CREAT | O_RDWR, 0600);
        if (dev->shm_fd < 0) {
            fprintf(stderr, "[voodoo_fpga] shm_open(%s) failed: %s\n",
                    SST1_SHM_NAME, strerror(errno));
            return -1;
        }
        if (ftruncate(dev->shm_fd, sizeof(sst1_bridge_channel_t)) < 0) {
            fprintf(stderr, "[voodoo_fpga] ftruncate failed: %s\n",
                    strerror(errno));
            close(dev->shm_fd);
            dev->shm_fd = -1;
            return -1;
        }
    }

    dev->channel = (sst1_bridge_channel_t *)mmap(
        NULL, sizeof(sst1_bridge_channel_t),
        PROT_READ | PROT_WRITE, MAP_SHARED,
        dev->shm_fd, 0);

    if (dev->channel == MAP_FAILED) {
        fprintf(stderr, "[voodoo_fpga] mmap failed: %s\n", strerror(errno));
        close(dev->shm_fd);
        dev->shm_fd = -1;
        dev->channel = NULL;
        return -1;
    }

    /* Initialize header if we're the first to connect */
    if (dev->channel->magic != SST1_BRIDGE_CHANNEL_MAGIC) {
        memset(dev->channel, 0, sizeof(sst1_bridge_channel_t));
        dev->channel->magic = SST1_BRIDGE_CHANNEL_MAGIC;
    }

    /* Mark emulator side as connected */
    SST1_ATOMIC_STORE(&dev->channel->emu_connected, 1);

    fprintf(stderr, "[voodoo_fpga] Shared memory connected: %s (%zu bytes)\n",
            SST1_SHM_NAME, sizeof(sst1_bridge_channel_t));

    return 0;
#else
    (void) dev;
    fprintf(stderr, "[voodoo_fpga] Shared memory bridge requires Linux\n");
    return -1;
#endif
}

static void
bridge_shm_close(voodoo_fpga_bridge_t *dev)
{
#ifdef __linux__
    if (dev->channel) {
        SST1_ATOMIC_STORE(&dev->channel->emu_connected, 0);
        munmap(dev->channel, sizeof(sst1_bridge_channel_t));
        dev->channel = NULL;
    }
    if (dev->shm_fd >= 0) {
        close(dev->shm_fd);
        dev->shm_fd = -1;
    }
#else
    (void) dev;
#endif
}

/* ========================================================================= */
/* Direct PCI channel -- blocking request/response                            */
/* ========================================================================= */

/*
 * Submit a PCI request via the direct channel and wait for completion.
 * Returns 0 on success, -1 on timeout or no channel.
 */
static int
bridge_pci_request(voodoo_fpga_bridge_t *dev,
                   uint32_t req_type,
                   uint32_t cfg_addr,
                   uint32_t mem_addr,
                   uint32_t wdata,
                   uint32_t *rdata)
{
    sst1_bridge_channel_t *ch = dev->channel;
    if (!ch)
        return -1;

    /* Set up the request */
    SST1_ATOMIC_STORE(&ch->pci_cfg_addr, cfg_addr);
    SST1_ATOMIC_STORE(&ch->pci_addr, mem_addr);
    SST1_ATOMIC_STORE(&ch->pci_wdata, wdata);
    SST1_ATOMIC_STORE(&ch->pci_done, 0);

    SST1_FENCE_RELEASE();

    /* Issue the request (must be last write) */
    SST1_ATOMIC_STORE(&ch->pci_request, req_type);

    /* Busy-wait for the Verilator side to complete */
    for (int i = 0; i < SST1_REQ_TIMEOUT; i++) {
        SST1_FENCE_ACQUIRE();
        if (SST1_ATOMIC_LOAD(&ch->pci_done)) {
            if (rdata)
                *rdata = SST1_ATOMIC_LOAD(&ch->pci_rdata);
            return 0;
        }
    }

    fprintf(stderr, "[voodoo_fpga] PCI request timeout (req=%u, cfg_addr=0x%x)\n",
            req_type, cfg_addr);
    return -1;
}

/* ========================================================================= */
/* PCI Configuration Space handlers (called by 86Box PCI subsystem)           */
/* ========================================================================= */

static uint8_t
fpga_pci_read(int func, int addr, UNUSED(int len), void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;

    if (func)
        return 0;

    /* Try to forward to the RTL model */
    uint32_t rdata = 0;
    if (bridge_pci_request(dev, SST1_PCI_REQ_CONFIG_READ,
                           (uint32_t)(addr & ~3), 0, 0, &rdata) == 0) {
        /* Extract the requested byte from the 32-bit DWORD */
        int shift = (addr & 3) * 8;
        return (uint8_t)(rdata >> shift);
    }

    /*
     * Fallback: return hardcoded Voodoo 1 identity so 86Box's PCI scan
     * at least finds the device even if the Verilator process isn't running.
     */
    switch (addr) {
        case 0x00: return 0x1a;  /* Vendor ID low (3Dfx = 0x121A) */
        case 0x01: return 0x12;  /* Vendor ID high */
        case 0x02: return 0x01;  /* Device ID low (SST-1) */
        case 0x03: return 0x00;  /* Device ID high */
        case 0x04: return dev->pci_enable ? 0x02 : 0x00;
        case 0x08: return 0x02;  /* Revision ID */
        case 0x0a: return 0x00;  /* Sub-class */
        case 0x0b: return 0x00;  /* Base class (pre-VGA) */
        case 0x0e: return 0x00;  /* Header type */
        case 0x10: return 0x00;  /* BAR0 [7:0] */
        case 0x11: return 0x00;  /* BAR0 [15:8] */
        case 0x12: return 0x00;  /* BAR0 [23:16] */
        case 0x13: return (uint8_t)(dev->mem_base_addr >> 24);
        case 0x3c: return 0x05;  /* Int Line (IRQ5) */
        case 0x3d: return 0x01;  /* Int Pin (INTA#) */
        default:   return 0x00;
    }
}

static void
fpga_pci_write(int func, int addr, UNUSED(int len), uint8_t val, void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;

    if (func)
        return;

    /* Forward to RTL model */
    bridge_pci_request(dev, SST1_PCI_REQ_CONFIG_WRITE,
                       (uint32_t)(addr & ~3), 0, (uint32_t)val << ((addr & 3) * 8),
                       NULL);

    /*
     * Also maintain local shadow state so 86Box's memory mapping works
     * even during Verilator communication.
     */
    switch (addr) {
        case 0x04:
            dev->pci_enable = val & 0x02;
            fpga_recalc_mapping(dev);
            break;

        case 0x13:
            dev->mem_base_addr = (uint32_t)val << 24;
            fpga_recalc_mapping(dev);
            break;

        default:
            break;
    }
}

/* ========================================================================= */
/* Memory-mapped I/O handlers                                                 */
/* ========================================================================= */

/*
 * Recalculate the memory mapping window position based on current BAR0
 * and command register state.
 */
static void
fpga_recalc_mapping(voodoo_fpga_bridge_t *dev)
{
    if (dev->pci_enable && dev->mem_base_addr) {
        mem_mapping_set_addr(&dev->mapping, dev->mem_base_addr, SST1_MEM_SIZE);
    } else {
        mem_mapping_disable(&dev->mapping);
    }
}

/*
 * Intercept writes to shadow fbiInit0 for VGA passthrough control.
 */
static void
fpga_snoop_fbiInit0(voodoo_fpga_bridge_t *dev, uint32_t val)
{
    uint32_t old = dev->fbiInit0;
    dev->fbiInit0 = val;

    /* Detect VGA pass-through bit changes */
    if ((old ^ val) & FBIINIT0_VGA_PASS) {
        if (dev->svga) {
            svga_set_override(dev->svga, (val & FBIINIT0_VGA_PASS) ? 1 : 0);
            fprintf(stderr, "[voodoo_fpga] VGA passthrough %s\n",
                    (val & FBIINIT0_VGA_PASS) ? "ACTIVE (Voodoo drives display)"
                                               : "INACTIVE (VGA drives display)");
        }
    }
}

/*
 * Memory write handler -- push to the ring buffer for bulk throughput.
 * Falls back to the direct channel if the ring is full.
 */
static void
fpga_write_l(uint32_t addr, uint32_t val, void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;

    /* Compute offset within the 16 MB window */
    uint32_t offset = addr - dev->mem_base_addr;

    /* Snoop fbiInit0 writes for VGA passthrough control */
    if (offset == SST_FBIINIT0)
        fpga_snoop_fbiInit0(dev, val);

    /* Try the fast path: ring buffer */
    if (dev->channel) {
        sst1_bridge_cmd_t cmd;
        cmd.cmd   = SST1_CMD_WRITE32;
        cmd.addr  = offset & 0x00FFFFFF;
        cmd.data  = val;
        cmd.flags = 0;

        if (sst1_ring_push(dev->channel, &cmd))
            return;  /* Success -- queued in ring */
    }

    /* Slow path: direct channel (blocks until Verilator processes it) */
    bridge_pci_request(dev, SST1_PCI_REQ_WRITE32, 0, offset, val, NULL);
}

static void
fpga_write_w(uint32_t addr, uint16_t val, void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;
    uint32_t offset = addr - dev->mem_base_addr;

    if (dev->channel) {
        sst1_bridge_cmd_t cmd;
        cmd.cmd   = SST1_CMD_WRITE16;
        cmd.addr  = offset & 0x00FFFFFF;
        cmd.data  = (uint32_t)val;
        cmd.flags = 0;

        if (sst1_ring_push(dev->channel, &cmd))
            return;
    }

    bridge_pci_request(dev, SST1_PCI_REQ_WRITE16, 0, offset, (uint32_t)val, NULL);
}

static void
fpga_write_b(uint32_t addr, uint8_t val, void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;
    uint32_t offset = addr - dev->mem_base_addr;

    if (dev->channel) {
        sst1_bridge_cmd_t cmd;
        cmd.cmd   = SST1_CMD_WRITE8;
        cmd.addr  = offset & 0x00FFFFFF;
        cmd.data  = (uint32_t)val;
        cmd.flags = 0;

        if (sst1_ring_push(dev->channel, &cmd))
            return;
    }

    bridge_pci_request(dev, SST1_PCI_REQ_WRITE32, 0, offset, (uint32_t)val, NULL);
}

/*
 * Memory read handler -- uses the direct channel (blocking).
 * Reads are rare in Voodoo (mostly status register polling).
 */
static uint32_t
fpga_read_l(uint32_t addr, void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;
    uint32_t offset = addr - dev->mem_base_addr;
    uint32_t rdata  = 0xFFFFFFFF;

    bridge_pci_request(dev, SST1_PCI_REQ_READ32, 0, offset, 0, &rdata);

    return rdata;
}

static uint16_t
fpga_read_w(uint32_t addr, void *priv)
{
    return (uint16_t)(fpga_read_l(addr & ~1, priv) >> ((addr & 1) * 8));
}

static uint8_t
fpga_read_b(uint32_t addr, void *priv)
{
    return (uint8_t)(fpga_read_l(addr & ~3, priv) >> ((addr & 3) * 8));
}

/* ========================================================================= */
/* Display scanline timer callback                                            */
/*                                                                            */
/* Modelled on voodoo_callback() in vid_voodoo_display.c.                     */
/* Fires once per scanline.  During the visible region, reads RGB565 pixels   */
/* from the shared memory framebuffer and writes ARGB32 into the 86Box        */
/* monitor's target_buffer.  At vblank, calls svga_doblit() to push the       */
/* completed frame to the GUI.                                                */
/* ========================================================================= */

static void
fpga_callback(void *priv)
{
    voodoo_fpga_bridge_t *dev     = (voodoo_fpga_bridge_t *)priv;
    const monitor_t      *monitor = &monitors[dev->monitor_index];
    int v_y_add = (monitor->mon_overscan_y >> 1);
    int v_x_add = (monitor->mon_overscan_x >> 1);

    /* Pick up display dimensions from the Verilator side if available */
    if (dev->channel && SST1_ATOMIC_LOAD(&dev->channel->fb_ready)) {
        uint32_t w = SST1_ATOMIC_LOAD(&dev->channel->fb_width);
        uint32_t h = SST1_ATOMIC_LOAD(&dev->channel->fb_height);
        if (w > 0 && w <= 1024)
            dev->h_disp = (int)w;
        if (h > 0 && h <= 768)
            dev->v_disp = (int)h;
    }

    /*
     * Only render when VGA passthrough is active (we own the display).
     */
    if (dev->fbiInit0 & FBIINIT0_VGA_PASS) {
        if (dev->line < dev->v_disp) {
            /* Read this scanline from the shared memory framebuffer */
            if (dev->channel && SST1_ATOMIC_LOAD(&dev->channel->fb_ready)) {
                uint32_t *p = &monitor->target_buffer->line[dev->line + v_y_add][v_x_add];
                uint32_t  stride = SST1_ATOMIC_LOAD(&dev->channel->fb_stride);
                if (stride == 0)
                    stride = (uint32_t)dev->h_disp;

                const uint16_t *src = &dev->channel->fb_pixels[dev->line * stride];
                int x;

                if (dev->line < dev->dirty_line_low) {
                    dev->dirty_line_low = dev->line;
                    video_wait_for_buffer_monitor(dev->monitor_index);
                }
                if (dev->line > dev->dirty_line_high)
                    dev->dirty_line_high = dev->line;

                /* Draw left overscan (black) */
                for (x = 0; x < v_x_add; x++)
                    monitor->target_buffer->line[dev->line + v_y_add][x] = 0x00000000;

                /* Convert RGB565 -> ARGB32 via CLUT */
                for (x = 0; x < dev->h_disp; x++)
                    p[x] = dev->video_16to32[src[x]];

                /* Draw right overscan (black) */
                for (x = 0; x < v_x_add; x++)
                    monitor->target_buffer->line[dev->line + v_y_add][dev->h_disp + x + v_x_add] =
                        0x00000000;
            }
        }
    }

    /* At vblank: push the frame to the GUI */
    if (dev->line == dev->v_disp) {
        if (dev->fbiInit0 & FBIINIT0_VGA_PASS) {
            if (dev->dirty_line_high > dev->dirty_line_low) {
                svga_doblit(dev->h_disp, dev->v_disp - 1, dev->svga);
            } else if (dev->svga && dev->svga->override) {
                /* Even if nothing is dirty, keep the rendered-frame counter
                   ticking so 86Box doesn't think the display has frozen. */
                dev->svga->monitor->mon_renderedframes++;
            }
            dev->dirty_line_high = -1;
            dev->dirty_line_low  = 2000;
        }
    }

    dev->line++;
    if (dev->line >= dev->v_total)
        dev->line = 0;

    /* Schedule next scanline callback (~32 us for 640x480 @ 60 Hz) */
    timer_advance_u64(&dev->timer, TIMER_USEC * 32);
}

/* ========================================================================= */
/* RGB565 -> ARGB32 CLUT initialisation                                       */
/* ========================================================================= */

static void
fpga_build_16to32(voodoo_fpga_bridge_t *dev)
{
    /*
     * Build a simple identity CLUT mapping RGB565 to ARGB32.
     * The real Voodoo 1 applies a 33-entry CLUT, but since we're reading
     * the post-CLUT framebuffer from the RTL, a straight expansion is fine.
     */
    for (int c = 0; c < 0x10000; c++) {
        int r = (c >> 8) & 0xf8;
        int g = (c >> 3) & 0xfc;
        int b = (c << 3) & 0xf8;
        /* Fill the low bits for a full 8-bit range */
        r |= (r >> 5);
        g |= (g >> 6);
        b |= (b >> 5);
        dev->video_16to32[c] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    }
}

/* ========================================================================= */
/* Device lifecycle (86Box device_t callbacks)                                */
/* ========================================================================= */

static void *
fpga_init(UNUSED(const device_t *info))
{
    voodoo_fpga_bridge_t *dev;

    dev = (voodoo_fpga_bridge_t *)calloc(1, sizeof(voodoo_fpga_bridge_t));
    if (!dev)
        return NULL;

    dev->shm_fd  = -1;
    dev->channel = NULL;

    fprintf(stderr, "[voodoo_fpga] Initializing SST-1 FPGA bridge device\n");

    /* Build RGB565 -> ARGB32 colour lookup table */
    fpga_build_16to32(dev);

    /* Set default display dimensions */
    dev->h_disp  = SST1_DEFAULT_H_DISP;
    dev->v_disp  = SST1_DEFAULT_V_DISP;
    dev->v_total = SST1_DEFAULT_V_TOTAL;
    dev->dirty_line_low  = 2000;
    dev->dirty_line_high = -1;

    /* Open shared memory channel to the Verilator process */
    if (bridge_shm_open(dev) < 0) {
        fprintf(stderr, "[voodoo_fpga] WARNING: Could not open shared memory. "
                "Device will use fallback values.\n");
    }

    /* Register PCI card (normal PCI slot, same as real Voodoo1) */
    pci_add_card(PCI_ADD_NORMAL,
                 fpga_pci_read, fpga_pci_write,
                 dev, &dev->pci_slot);

    /* Register memory mapping (initially disabled, set by BAR programming) */
    mem_mapping_add(&dev->mapping, 0, 0,
                    fpga_read_b, fpga_read_w, fpga_read_l,
                    fpga_write_b, fpga_write_w, fpga_write_l,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    /* Start scanline display timer (fires immediately, self-re-arms) */
    timer_add(&dev->timer, fpga_callback, dev, 1);

    /* Get the primary VGA card for passthrough control */
    dev->svga = svga_get_pri();
    if (dev->svga) {
        dev->monitor_index = dev->svga->monitor_index;
        fprintf(stderr, "[voodoo_fpga] VGA passthrough: SVGA found (monitor %d)\n",
                dev->monitor_index);
    } else {
        dev->monitor_index = 0;
        fprintf(stderr, "[voodoo_fpga] WARNING: No primary SVGA found for VGA passthrough\n");
    }

    /* Initially VGA passthrough is OFF (fbiInit0 = 0) */
    dev->fbiInit0 = 0;

    fprintf(stderr, "[voodoo_fpga] SST-1 FPGA bridge ready (PCI slot %d)\n",
            dev->pci_slot);

    return dev;
}

static void
fpga_close(void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;

    if (!dev)
        return;

    fprintf(stderr, "[voodoo_fpga] Closing SST-1 FPGA bridge\n");

    /* Release VGA passthrough if active */
    if ((dev->fbiInit0 & FBIINIT0_VGA_PASS) && dev->svga)
        svga_set_override(dev->svga, 0);

    bridge_shm_close(dev);

    free(dev);
}

static void
fpga_speed_changed(void *priv)
{
    /* Nothing to do -- the Verilator model runs at its own clock rate */
    (void) priv;
}

static void
fpga_force_redraw(void *priv)
{
    voodoo_fpga_bridge_t *dev = (voodoo_fpga_bridge_t *)priv;
    if (!dev)
        return;

    /* Mark all lines as dirty so the next vblank pushes a full frame */
    dev->dirty_line_low  = 0;
    dev->dirty_line_high = dev->v_disp;
}

/* ========================================================================= */
/* Device registration                                                        */
/* ========================================================================= */

const device_t voodoo_fpga_bridge_device = {
    .name          = "3Dfx Voodoo Graphics (FPGA Bridge)",
    .internal_name = "voodoo_fpga_bridge",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = fpga_init,
    .close         = fpga_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = fpga_speed_changed,
    .force_redraw  = fpga_force_redraw,
    .config        = NULL
};
