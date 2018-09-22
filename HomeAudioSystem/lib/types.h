#ifndef LIB_TYPES_H_INC
#define LIB_TYPES_H_INC
/*
    types.h - declarations of types shared between modules

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/


// Pinset_t - enumeration used to specify whether a peripheral should use the default or alternate
// pin-set.
//
typedef enum Pinset
{
    PinsetDefault       = 0,
    PinsetAlternative   = 1
} Pinset_t;

#endif
