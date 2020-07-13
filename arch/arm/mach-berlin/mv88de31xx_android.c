/*
 * Android specific low-level init/reboot code for Marvell MV88DE31XX SoCs.
 * Ported from IntelCE version authored by Eugene Surovegin.
 *
 * Copyright (c) 2012 Google, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/err.h>
#include <linux/flash_ts.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/random.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <asm/setup.h>

#define ONE_MB     (1024 * 1024)

int __initdata board_rev;

static int bcb_fts_reboot_hook(struct notifier_block*, unsigned long, void*);

/* We need to reserve chunk of system memory for Marvell SDK.
 * Different boards may have different requirements.
 * There are also per-board flash layouts.
 */
static __initdata struct board_config {
    char *name;
    unsigned int board_rev;

    /* MTD layout(s) in cmdlinepart format */
    const char *mtdparts;

  /* MTD layout(s) in restricted mode triggered
   * by boot command line param ro. Only really useful for
   * hardware revisions used for engineering development.
   */
    const char *mtdparts_ro;

    /* optional MTD layout(s) in cmdlinepart format for recovery mode */
    const char *mtdparts_recovery;

    /* default root device in case nothing was specified */
    const char *default_root;

    /* optional reboot hook */
    int (*reboot_notifier)(struct notifier_block*, unsigned long, void*);

} boards[] = {
#ifdef CONFIG_BERLIN2CD
    { .name = "bg2proto",
      .board_rev = 0,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 4G MLC, 8K pages, 1M block */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define BG2PROTO_MTDPARTS(__RO__)                      \
      "mv_nand:"                                       \
      "1M(block0)ro," /* flash config params */        \
      "8M(bootloader)" __RO__ "," /* 8x copies */      \
      "7M(fts)ro,"                                     \
      "16M(kernel)"__RO__ ","                          \
      "60M(recovery),"                                 \
      "80M(backupsys)ro,"                              \
      "16M(factory_store)" __RO__ ","                  \
      "400M(rootfs)" __RO__ ","                        \
      "32M(localstorage),"                             \
      "300M(cache),"                                   \
      "1024M(userdata),"                                \
      "8M@4088M(bbt)ro"
      /* TODO: limit write access to important partitions
       * in non-recovery mode
       */
      .mtdparts = BG2PROTO_MTDPARTS(""),
      .mtdparts_ro = BG2PROTO_MTDPARTS("ro"),
      .mtdparts_recovery = BG2PROTO_MTDPARTS(""),
    },
    { .name = "eureka-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 2G MLC, 4K pages, 1M block */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define EUREKA_B1_MTDPARTS(__RO__,__RRO__)             \
      "mv_nand:"                                       \
      "1M(block0)ro," /* flash config params */        \
      "8M(bootloader)" __RO__ "," /* 8x copies */      \
      "7M(fts)ro,"                                     \
      "16M(kernel)"__RO__ ","                          \
      "60M(recovery),"                                 \
      "80M(backupsys),"                                \
      "16M(factory_store),"                            \
      "400M(rootfs)" __RRO__","                        \
      "300M(cache),"                                   \
      "1024M(userdata),"                               \
      "8M@2040M(bbt)ro"
      .mtdparts = EUREKA_B1_MTDPARTS("ro",""),
      .mtdparts_ro = EUREKA_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = EUREKA_B1_MTDPARTS("",""),
    },
    { .name = "eureka-b2",
      .board_rev = 2,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 2G MLC, 4K pages, 1M block */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define EUREKA_B2_MTDPARTS(__RO__)                           \
      "mv_nand:"                                             \
      "1M(block0)ro," /* flash config params */              \
      "8M(bootloader)" __RO__ "," /* 8x copies */            \
      "16M(kernel)"__RO__ ","                                \
      "400M(rootfs)"__RO__ ","                               \
      "300M(cache),"                                         \
      "1147M(userdata),"                                     \
      "48M(recovery),"                                       \
      "96M(backupsys),"                                      \
      "8M(fts)ro,"                                           \
      "16M(factory_store),"                                  \
      "8M(bbt)ro"
      .mtdparts = EUREKA_B2_MTDPARTS("ro"),
      .mtdparts_ro = EUREKA_B2_MTDPARTS("ro"),
      .mtdparts_recovery = EUREKA_B2_MTDPARTS(""),
    },
    { .name = "eureka-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 2G MLC, 4K pages, 1M block */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define EUREKA_B3_MTDPARTS(__RO__)                           \
      "mv_nand:"                                             \
      "1M(block0)ro," /* flash config params */              \
      "8M(bootloader)" __RO__ "," /* 8x copies */            \
      "16M(kernel)"__RO__ ","                                \
      "400M(rootfs)"__RO__ ","                               \
      "300M(cache),"                                         \
      "1147M(userdata),"                                     \
      "48M(recovery),"                                       \
      "96M(backupsys),"                                      \
      "8M(fts)ro,"                                           \
      "16M(factory_store)" __RO__ ","                        \
      "8M(bbt)ro"
      .mtdparts = EUREKA_B3_MTDPARTS("ro"),
      .mtdparts_ro = EUREKA_B3_MTDPARTS("ro"),
      .mtdparts_recovery = EUREKA_B3_MTDPARTS(""),
    },
#elif defined(CONFIG_BERLIN2CDP)
    { .name = "salami-proto",
      .board_rev = 0,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 2G MLC, 4K pages, 1M block */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define SALAMI_PROTO_MTDPARTS(__RO__,__RRO__)          \
      "mv_nand:"                                       \
      "1M(block0)ro," /* flash config params */        \
      "8M(bootloader)" __RO__ "," /* 8x copies */      \
      "7M(fts)ro,"                                     \
      "16M(kernel)"__RO__ ","                          \
      "60M(recovery),"                                 \
      "80M(backupsys),"                                \
      "16M(factory_store)"                             \
      "400M(rootfs)" __RRO__","                        \
      "300M(cache),"                                   \
      "1024M(userdata),"                               \
      "8M@2040M(bbt)ro"
      .mtdparts = SALAMI_PROTO_MTDPARTS("ro",""),
      .mtdparts_ro = SALAMI_PROTO_MTDPARTS("ro","ro"),
      .mtdparts_recovery = SALAMI_PROTO_MTDPARTS("",""),
    },
    { .name = "lexx-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
#define LEXX_B3_MTDPARTS(__RO__,__RRO__)             \
      "mv_nand:"                                       \
      "128K(block0)ro," /* flash config params */      \
      "1M(prebootloader)" __RO__ "," /* 8x copies */   \
      "4M(TZ)" __RO__ ","                              \
      "4M(TZ-B)" __RO__ ","                            \
      "512K(postbootloader)" __RO__ ","                  \
      "512K(postbootloader-B)" __RO__ ","                \
      "8M(kernel)" __RO__ ","                          \
      "105M(rootfs)" __RRO__","                        \
      "117120K(cache),"                                \
      "15M(recovery),"                                 \
      "512K(fts)ro,"                                   \
      "2M(factory_store)" ","                          \
      "1M@255M(bbt)ro"
      .mtdparts = LEXX_B3_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B3_MTDPARTS("",""),
    },
    { .name = "lexx-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 256MB SLC, 2K page size, 128K block size */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define LEXX_B4_MTDPARTS(__RO__,__RRO__)                  \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "2304K(TZ)" __RO__ ","                              \
      "2304K(TZ-B)" __RO__ ","                            \
      "512K(postbootloader)" __RO__ ","                   \
      "512K(postbootloader-B)" __RO__ ","                 \
      "4608K(kernel)" __RO__ ","                          \
      "90M(rootfs)" __RRO__","                            \
      "146944K(cache),"                                   \
      "9728K(recovery),"                                  \
      "1M(factory_store)" __RO__ ","                      \
      "512K(fts)ro,"                                      \
      "384K(bbt)ro"
      .mtdparts = LEXX_B4_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B4_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B4_MTDPARTS("",""),
    },
    { .name = "earth-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* earth-b1 uses same partition map as lexx-b3 */
      .mtdparts = LEXX_B3_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B3_MTDPARTS("",""),
    },
    { .name = "earth-b2",
      .board_rev = 2,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* earth-b2 uses same partition map as lexx-b3 */
      .mtdparts = LEXX_B3_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B3_MTDPARTS("",""),
    },
    { .name = "earth-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 256MB SLC, 2K page size, 128K block size */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define EARTH_B3_MTDPARTS(__RO__,__RRO__)                 \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "4M(TZ)" __RO__ ","                                 \
      "4M(TZ-B)" __RO__ ","                               \
      "640K(postbootloader)" __RO__ ","                   \
      "640K(postbootloader-B)" __RO__ ","                 \
      "8M(kernel)" __RO__ ","                             \
      "105M(rootfs)" __RRO__","                           \
      "117120K(cache),"                                   \
      "15104K(recovery),"                                 \
      "512K(fts)ro,"                                      \
      "2M(factory_store)" ","                             \
      "1M@255M(bbt)ro"
      .mtdparts = EARTH_B3_MTDPARTS("ro",""),
      .mtdparts_ro = EARTH_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = EARTH_B3_MTDPARTS("",""),
    },
    { .name = "earth-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 256MB SLC, 2K page size, 128K block size */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define EARTH_B4_MTDPARTS(__RO__,__RRO__)                 \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "2304K(TZ)" __RO__ ","                              \
      "2304K(TZ-B)" __RO__ ","                            \
      "640K(postbootloader)" __RO__ ","                   \
      "640K(postbootloader-B)" __RO__ ","                 \
      "4608K(kernel)" __RO__ ","                          \
      "90M(rootfs)" __RRO__","                            \
      "146944K(cache),"                                   \
      "9472K(recovery),"                                  \
      "1M(factory_store)" __RO__ ","                      \
      "512K(fts)ro,"                                      \
      "384K(bbt)ro"
      .mtdparts = EARTH_B4_MTDPARTS("ro",""),
      .mtdparts_ro = EARTH_B4_MTDPARTS("ro","ro"),
      .mtdparts_recovery = EARTH_B4_MTDPARTS("",""),
    },
    { .name = "hendrix-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* hendrix-b1 uses same partition map as lexx-b3 */
      .mtdparts = LEXX_B3_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B3_MTDPARTS("",""),
    },
    { .name = "hendrix-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* hendrix-b3 uses same partition map as earth-b3 */
      .mtdparts = EARTH_B3_MTDPARTS("ro",""),
      .mtdparts_ro = EARTH_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = EARTH_B3_MTDPARTS("",""),
    },
    { .name = "hendrix-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* TODO(mtraver): If we decide to change hendrix rootfs size,
       * make a new HENDRIX_B4_MTDPARTS. */
      /* hendrix-b4 uses same partition map as earth-b4 */
      .mtdparts = EARTH_B4_MTDPARTS("ro",""),
      .mtdparts_ro = EARTH_B4_MTDPARTS("ro","ro"),
      .mtdparts_recovery = EARTH_B4_MTDPARTS("",""),
    },
    { .name = "chirp-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* chirp-b1 uses same partition map as lexx-b3 */
      .mtdparts = LEXX_B3_MTDPARTS("ro",""),
      .mtdparts_ro = LEXX_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = LEXX_B3_MTDPARTS("",""),
    },
    { .name = "chirp-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 256MB SLC, 2K page size, 128K block size */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define CHIRP_B3_MTDPARTS(__RO__,__RRO__)                 \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "4M(TZ)" __RO__ ","                                 \
      "4M(TZ-B)" __RO__ ","                               \
      "640K(postbootloader)" __RO__ ","                   \
      "640K(postbootloader-B)" __RO__ ","                 \
      "5M(kernel)" __RO__ ","                             \
      "105M(rootfs)" __RRO__","                           \
      "128128K(cache),"                                   \
      "7M(recovery),"                                     \
      "512K(fts)ro,"                                      \
      "2M(factory_store)" ","                             \
      "1M@255M(bbt)ro"
      .mtdparts = CHIRP_B3_MTDPARTS("ro",""),
      .mtdparts_ro = CHIRP_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CHIRP_B3_MTDPARTS("",""),
    },
    { .name = "chirp-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,

      /* 256MB SLC, 2K page size, 128K block size */
      /*
       * MTD partition name has to be shorter than 14 bytes due to
       * limitation of Android init.
       */
#define CHIRP_B4_MTDPARTS(__RO__,__RRO__)                 \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "2304K(TZ)" __RO__ ","                              \
      "2304K(TZ-B)" __RO__ ","                            \
      "640K(postbootloader)" __RO__ ","                   \
      "640K(postbootloader-B)" __RO__ ","                 \
      "4608K(kernel)" __RO__ ","                          \
      "90M(rootfs)" __RRO__","                            \
      "149248K(cache),"                                   \
      "7M(recovery),"                                     \
      "1M(factory_store)" __RO__ ","                      \
      "512K(fts)ro,"                                      \
      "384K(bbt)ro"
      .mtdparts = CHIRP_B4_MTDPARTS("ro",""),
      .mtdparts_ro = CHIRP_B4_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CHIRP_B4_MTDPARTS("",""),
    },
    { .name = "joplin-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
#define JOPLIN_B1_MTDPARTS(__RO__,__RRO__)             \
      "mv_nand:"                                       \
      "128K(block0)ro," /* flash config params */      \
      "1M(prebootloader)" __RO__ "," /* 8x copies */   \
      "4M(TZ)" __RO__ ","                              \
      "4M(TZ-B)" __RO__ ","                            \
      "512K(postbootloader)" __RO__ ","                \
      "512K(postbootloader-B)" __RO__ ","              \
      "8M(kernel)" __RO__ ","                          \
      "105M(rootfs)" __RRO__","                        \
      "117120K(cache),"                                \
      "15M(recovery),"                                 \
      "512K(fts)ro,"                                   \
      "2M(factory_store)" __RO__ ","                   \
      "1M@255M(bbt)ro"
      .mtdparts = JOPLIN_B1_MTDPARTS("ro",""),
      .mtdparts_ro = JOPLIN_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = JOPLIN_B1_MTDPARTS("",""),
    },
    { .name = "joplin-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
#define JOPLIN_B3_MTDPARTS(__RO__,__RRO__)                \
      "mv_nand:"                                          \
      "128K(block0)ro," /* flash config params */         \
      "1M(prebootloader)" __RO__ "," /* 8x copies */      \
      "4M(TZ)" __RO__ ","                                 \
      "4M(TZ-B)" __RO__ ","                               \
      "640K(postbootloader)" __RO__ ","                   \
      "640K(postbootloader-B)" __RO__ ","                 \
      "5M(kernel)" __RO__ ","                             \
      "105M(rootfs)" __RRO__","                           \
      "128128K(cache),"                                   \
      "7M(recovery),"                                     \
      "512K(fts)ro,"                                      \
      "2M(factory_store)" __RO__ ","                      \
      "1M@255M(bbt)ro"
      .mtdparts = JOPLIN_B3_MTDPARTS("ro",""),
      .mtdparts_ro = JOPLIN_B3_MTDPARTS("ro","ro"),
      .mtdparts_recovery = JOPLIN_B3_MTDPARTS("",""),
    },
    { .name = "joplin-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      .mtdparts = CHIRP_B4_MTDPARTS("ro",""),
      .mtdparts_ro = CHIRP_B4_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CHIRP_B4_MTDPARTS("",""),
    },
    { .name = "coco-p1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
#define COCO_P1_MTDPARTS(__RO__,__RRO__)               \
      "mv_nand:"                                       \
      "128K(block0)ro," /* flash config params */      \
      "1M(prebootloader)" __RO__ "," /* 8x copies */   \
      "4M(TZ)" __RO__ ","                              \
      "4M(TZ-B)" __RO__ ","                            \
      "512K(postbootloader)" __RO__ ","                \
      "512K(postbootloader-B)" __RO__ ","              \
      "8M(kernel)" __RO__ ","                          \
      "105M(rootfs)" __RRO__","                        \
      "117120K(cache),"                                \
      "15M(recovery),"                                 \
      "512K(fts)ro,"                                   \
      "2M(factory_store)" __RO__ ","                   \
      "1M@255M(bbt)ro"
      .mtdparts = COCO_P1_MTDPARTS("ro",""),
      .mtdparts_ro = COCO_P1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = COCO_P1_MTDPARTS("",""),
    },
    { .name = "coco-p2",
      .board_rev = 2,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      .mtdparts = COCO_P1_MTDPARTS("ro",""),
      .mtdparts_ro = COCO_P1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = COCO_P1_MTDPARTS("",""),
    },
    {
#define CAPRICA_B1_MTDPARTS(__RO__,__RRO__)               \
      "mv_nand:"                                          \
      "256K(block0)ro," /* flash config params */         \
      "2M(prebootloader)" __RO__ "," /* 8x copies */      \
      "2304K(TZ)" __RO__ ","                              \
      "2304K(TZ-B)" __RO__ ","                            \
      "1M(postbootloader)" __RO__ ","                     \
      "1M(postbootloader-B)" __RO__ ","                   \
      "5632K(kernel)" __RO__ ","                          \
      "180M(rootfs)" __RRO__","                           \
      "310528K(cache),"                                   \
      "9728K(recovery),"                                  \
      "1M(fts)ro,"                                        \
      "3M(factory_store)" ","                             \
      "1M@511M(bbt)ro"
      .name = "caprica-b1",
      .board_rev = 1,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      .mtdparts = CAPRICA_B1_MTDPARTS("ro",""),
      .mtdparts_ro = CAPRICA_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CAPRICA_B1_MTDPARTS("",""),
    },
    { .name = "caprica-b2",
      .board_rev = 2,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* caprica-b2 uses same partition map as caprica-b1 */
      .mtdparts = CAPRICA_B1_MTDPARTS("ro",""),
      .mtdparts_ro = CAPRICA_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CAPRICA_B1_MTDPARTS("",""),
    },
    {
      .name = "caprica-b3",
      .board_rev = 3,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* caprica-b3 uses same partition map as caprica-b1 */
      .mtdparts = CAPRICA_B1_MTDPARTS("ro",""),
      .mtdparts_ro = CAPRICA_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CAPRICA_B1_MTDPARTS("",""),
    },
    { .name = "caprica-b4",
      .board_rev = 4,
      .default_root = "/dev/mtdblock:rootfs",
      .reboot_notifier = bcb_fts_reboot_hook,
      /* caprica-b4 uses same partition map as caprica-b1 */
      .mtdparts = CAPRICA_B1_MTDPARTS("ro",""),
      .mtdparts_ro = CAPRICA_B1_MTDPARTS("ro","ro"),
      .mtdparts_recovery = CAPRICA_B1_MTDPARTS("",""),
    },
#endif /* CONFIG_BERLIN2CD */
};

/* This is what we will use if nothing has been passed by bootloader
 * or name wasn't recognized.
 */
#define DEFAULT_BOARD  "bg2proto"
static __initdata char board_name[32] = DEFAULT_BOARD;

static __initdata char console[32];
static __initdata char boot_mode[32];
static __initdata char root[32];
static __initdata int ro_mode = 0;
/* buffer for sanitized command line */
static char mv88de31xx_command_line[COMMAND_LINE_SIZE] __initdata;

/* Optional reboot notfier we use to write BCB */
static struct notifier_block reboot_notifier;

static void __init add_boot_param(const char *param, const char *val)
{
    if (mv88de31xx_command_line[0])
        strlcat(mv88de31xx_command_line, " ", COMMAND_LINE_SIZE);
    strlcat(mv88de31xx_command_line, param, COMMAND_LINE_SIZE);
    if (val) {
        strlcat(mv88de31xx_command_line, "=", COMMAND_LINE_SIZE);
        strlcat(mv88de31xx_command_line, val, COMMAND_LINE_SIZE);
    }
}

static int __init do_param(char *param, char *val, const char *unused)
{
    /* Skip all mem= and memmap= parameters */
    if (!strcmp(param, "mem") || !strcmp(param, "memmap"))
        return 0;

    /* Capture some params we will need later */
    if (!strcmp(param, "androidboot.hardware") && val) {
        strlcpy(board_name, val, sizeof(board_name));
        return 0;
    }

    if (!strcmp(param, "androidboot.mode") && val) {
        strlcpy(boot_mode, val, sizeof(boot_mode));
        return 0;
    }

    if (!strcmp(param, "ro")) {
        ro_mode = 1;
        return 0;
    }

    if (!strcmp(param, "console") && val)
        strlcpy(console, val, sizeof(console));

    if (!strcmp(param, "root") && val)
        strlcpy(root, val, sizeof(root));

    if (!strcmp(param, "prng_data") && val) {
        int size = strlen(val);
        early_print(KERN_INFO "Add randomness with %d bytes\n", size);
        add_device_randomness(val, size);
    }

    /* Re-add this parameter back */
    add_boot_param(param, val);

    return 0;
}

static __init struct board_config *get_board_config(const char *name)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(boards); ++i)
        if (!strcmp(name, boards[i].name))
            return boards + i;
    return NULL;
}

static __init char *get_cmdline(void)
{
    static char boot_command_line_copy[COMMAND_LINE_SIZE] __initdata;
    memcpy(boot_command_line_copy, boot_command_line, COMMAND_LINE_SIZE);
    return boot_command_line_copy;
}

/* Handle android specific kernel boot params and memory setup.
 * This function takes care of processing command line ATAG and
 * copying/updating mv88de31xx_command_line used in later part of kernel
 * setup, incl parsing of boot params.
 */
void __init mv88de31xx_android_fixup(char **from)
{
    struct board_config *cfg;
    const char *mtdparts;
    int recovery_boot;
    char *cmdline;

    /* Locate command line ATAG and extract command line */
    cmdline = get_cmdline();

    /* Override from(defaultp_command_line) */
    *from = mv88de31xx_command_line;
    mv88de31xx_command_line[0] = '\0';

    /* Scan boot command line removing all memory layout
     * parameters on the way.
     */
    mv88de31xx_command_line[0] = '\0';
    parse_args("memory_setup", cmdline, NULL, 0, 0, 0, do_param);

    recovery_boot = !strcmp(boot_mode, "recovery");
    if (recovery_boot)
        early_print(KERN_INFO "Recovery boot detected\n");

    /* Board-specific config */
    cfg = get_board_config(board_name);
    if (!cfg) {
        early_print(KERN_WARNING "Unknown board name: '%s', using '"
               DEFAULT_BOARD "'\n", board_name);
        cfg = get_board_config(DEFAULT_BOARD);
        strlcpy(board_name, DEFAULT_BOARD, sizeof(board_name));
        BUG_ON(!cfg);
    }

    // Save board_rev
    board_rev = cfg->board_rev;

    /* Add root device if it wasn't specified explicitly */
    if (!recovery_boot && !root[0])
        add_boot_param("root", cfg->default_root);

    /* Flash layout (in cmdlinepart format) */
    mtdparts = ro_mode ? cfg->mtdparts_ro : cfg->mtdparts;
    if (recovery_boot && cfg->mtdparts_recovery)
        mtdparts = cfg->mtdparts_recovery;
    if (mtdparts)
        add_boot_param("mtdparts", mtdparts);

    /* Add some Android-specific parameters */
    add_boot_param("androidboot.hardware", board_name);
    if (boot_mode[0])
        add_boot_param("androidboot.mode", boot_mode);
    if (console[0]) {
        /* keep only console device name, e.g. ttyS0 */
        char *p = strchr(console, ',');
        if (p)
            *p = '\0';
        add_boot_param("androidboot.console", console);
    }

    /* Register optional reboot hook */
    if (cfg->reboot_notifier) {
        reboot_notifier.notifier_call = cfg->reboot_notifier;
        register_reboot_notifier(&reboot_notifier);
    }

    return;
}

/* This is to prevent "Unknown boot option" messages */
static int __init dummy_param(char *arg) { return 0; }
__setup_param("androidboot.bootloader", dummy_1, dummy_param, 1);
__setup_param("androidboot.console", dummy_2, dummy_param, 1);
__setup_param("androidboot.hardware", dummy_3, dummy_param, 1);
__setup_param("androidboot.mode", dummy_4, dummy_param, 1);

/* BCB (boot control block) support */
static int bcb_fts_reboot_hook(struct notifier_block *notifier,
                   unsigned long code, void *cmd)
{
    if (code == SYS_RESTART && cmd && !strcmp(cmd, "recovery")) {
        if (flash_ts_set("bootloader.command", "boot-recovery") ||
            flash_ts_set("bootloader.status", "") ||
            flash_ts_set("bootloader.recovery", ""))
        {
            printk(KERN_ERR "Failed to set bootloader command\n");
        }
    }
    if (code == SYS_RESTART && cmd && !strcmp(cmd, "backupsys")) {
        if (flash_ts_set("bootloader.command", "boot-backupsys") ||
            flash_ts_set("bootloader.status", "") ||
            flash_ts_set("bootloader.recovery", ""))
        {
            printk(KERN_ERR "Failed to set bootloader command\n");
        }
    }

    return NOTIFY_DONE;
}
