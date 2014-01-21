//
// boot.c
//
// DO NOT EDIT THIS FILE.  This file was generated automatically
// by a script that converts an assembler listing into a C structure
// containing the hex values of the instructions in the listing.
// You must edit the assembler source directly and execute the build
// again.
//

static unsigned int aui_boot_code[] = {
               // G /tmp/ccRj5ql6.s    page 1
               // 
               // 
               // # 1 "boot.S"
               // #define CORE0_BUF0 0x900000003ff01020   
               // 
               // 
               // /* $OpenBSD: regdef.h,v 1.3 1999/01/27 04:46:06 imp Exp $ */
               // #define CORE1_BUF0 0x900000003ff01120   
               // #define CORE2_BUF0 0x900000003ff01220   
               // #define CORE3_BUF0 0x900000003ff01320   
               // 
               // .text
               // .global _start;
               // _start:
  0x40086000,  // mfc0 t0, CP0_STATUS
  0x240900E0,  // li      t1, 0x00e0      # {cu3,cu2,cu1,cu0}<={0110, status_fr<=1
  0x01094025,  // or      t0, t0, t1
  0x40886000,  // mtc0    t0, CP0_STATUS
  0x00000000,  // 
  0x40086000,  // mfc0    t0, CP0_STATUS
  0x3C090040,  // lui     t1, 0x40        #bev
  0x01094025,  // or      t0, t0, t1
  0x40886000,  // mtc0    t0, CP0_STATUS
  0x00000000,  // 
  0x40806800,  // mtc0    zero, CP0_CAUSE
  0x00000000,  // 
  0x40088000,  // mfc0 t0, CP0_CONFIG
  0x00000000,  // 
  0x35080007,  // ori     t0, t0, 7
  0x39080004,  // xori t0, t0, 4
  0x40888000,  // mtc0 t0, CP0_CONFIG
               // 
               // 
               // .set mips64
  0x400A7801,  // mfc0    t2, $15, 1
  0x314A03FF,  // andi    t2, 0x3ff
               // .set mips3
               // ########
               // #define FN_OFF 0x020
               // #define SP_OFF 0x028
               // #define GP_OFF 0x030
               // #define A1_OFF 0x038
               // 
  0x3C089000,  // dli     t0, 0x900000003ff01000 
  0x00084438,  // 
  0x35083FF0,  // 
  0x00084438,  // 
  0x35081000,  // 
  0x314B0003,  // andi    t3, t2, 0x3  #local cpuid
  0x000B5A00,  // sll     t3, 8
  0x010B4025,  // or      t0, t0, t3
               // 
  0x314C000C,  // andi    t4, t2, 0xc  #node id
  0x000C62BC,  // dsll     t4, 42
  0x010C4025,  // or      t0, t0, t4
               // 
               // 
               // waitforinit:   
               // 
               // NG /tmp/ccRj5ql6.s    page 2
               // 
               // 
  0x8D020020,  // lw      v0, FN_OFF(t0)
  0x1040FFFE,  // beqz    v0, waitforinit
  0x00000000,  // 
  0x00000000,  // nop
               // 
               // 
  0x2409FFFF,  // dli      t1, 0xffffffff00000000 
  0x0009483C,  // 
  0x00491025,  // or       v0, t1
               // 
  0x34099800,  // dli      t1, 0x9800000000000000 
  0x00094C3C,  // 
  0xDD1D0028,  // ld      sp, SP_OFF(t0)
  0x03A9E825,  // or      sp, t1
  0xDD1C0030,  // ld      gp, GP_OFF(t0)
  0x0389E025,  // or      gp, t1
  0x0040F809,  // ld      a1, A1_OFF(t0)
               // 
               // //PRINTSTR("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& !!\r\n");
               // 
  0xDD050038,  // jalr    v0  #byebye 
  0x00000000,  // nop
               // 
               // #######
  0x1000FFFF,  // 1:  b   1b
  0x00000000,  // 
  0x00000000,  // nop
  0x00000000,  // 
  0x00000000,  // 
  0x00000000,  // 
  0x00000000}; // <- Inserted by script to terminate array.

