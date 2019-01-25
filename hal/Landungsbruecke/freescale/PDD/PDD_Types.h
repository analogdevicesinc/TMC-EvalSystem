/* Common type declarations for PDD layer */
/* This file is static and it is generated from API-Factory */

#ifndef PDD_TYPES_H_
#define PDD_TYPES_H_

/* --------------------------------------------------------------- */
/* --------- PDD_TBool */
/* --------------------------------------------------------------- */

/* Implementation note for boolean type: */
/* Native "bool" type should be used for boolean values */
/* because the compiler provides optimization and best type representation. */
/* If the compiler does not support native "bool" type, common "Bool" */
/* at each place there should be used the most suitable integer type */
/* and no explicit conversion to 0 and 1 values should be used. */
/* Common "TBool" typedef could have possible performance issue */
/* (there is a problem how wide type to select). It is good to avoid using */
/* explicit "!= 0" because of performance penalty, but without such */
/* explicit conversion to 0/1 the resulting type may be too wide. */
/* Every bool type (native or replaced by integer value) should be always */
/* treated as zero-equal or non-zero (see below for examples). */

/* E.g.: This compiler supports boolean type only in C++ mode. */
/* Out of the C++ mode the boolean type must be simulated. */

#ifndef __cplusplus
typedef uint16 PDD_TBool;
#else
typedef bool PDD_TBool;
#endif

/* ============================================================================
   ================== General PDD macros ======================================
   ============================================================================ */

#define PDD_DISABLE 0u
#define PDD_ENABLE  1u

/* ============================================================================
   ================== General register manipulating routines ==================
   ============================================================================ */

#define setReg8(Reg, val)                                   ((Reg) = (uint8)(val))
#define getReg8(Reg)                                        (Reg)
#define testReg8Bits(Reg, GetMask)                          ((Reg) & (uint8)(GetMask))
#define clrReg8Bits(Reg, ClrMask)                           ((Reg) &= (uint8)((uint8)(~(uint8)(ClrMask))))
#define setReg8Bits(Reg, SetMask)                           ((Reg) |= (uint8)(SetMask))
#define invertReg8Bits(Reg, InvMask)                        ((Reg) ^= (uint8)(InvMask))
#define clrSetReg8Bits(Reg, ClrMask, SetMask)               ((Reg) = (uint8)(((Reg) & (uint8)(~(uint8)(ClrMask))) | (uint8)(SetMask)))

#define setReg16(Reg, val)                                  ((Reg) = (uint16)(val))
#define getReg16(Reg)                                       (Reg)
#define testReg16Bits(Reg, GetMask)                         ((Reg) & (uint16)(GetMask))
#define clrReg16Bits(Reg, ClrMask)                          ((Reg) &= (uint16)(~(uint16)(ClrMask)))
#define setReg16Bits(Reg, SetMask)                          ((Reg) |= (uint16)(SetMask))
#define invertReg16Bits(Reg, InvMask)                       ((Reg) ^= (uint16)(InvMask))
#define clrSetReg16Bits(Reg, ClrMask, SetMask)              ((Reg) = (uint16)(((Reg) & (uint16)(~(uint16)(ClrMask))) | (uint16)(SetMask)))

#define setReg32(Reg, val)                                  ((Reg) = (uint32)(val))
#define getReg32(Reg)                                       (Reg)
#define testReg32Bits(Reg, GetMask)                         ((Reg) & (uint32)(GetMask))
#define clrReg32Bits(Reg, ClrMask)                          ((Reg) &= (uint32)(~(uint32)(ClrMask)))
#define setReg32Bits(Reg, SetMask)                          ((Reg) |= (uint32)(SetMask))
#define invertReg32Bits(Reg, InvMask)                       ((Reg) ^= (uint32)(InvMask))
#define clrSetReg32Bits(Reg, ClrMask, SetMask)              ((Reg) = (uint32)(((Reg) & (uint32)(~(uint32)(ClrMask))) | (uint32)(SetMask)))


#endif  /* #if !defined(PDD_TYPES_H_) */
