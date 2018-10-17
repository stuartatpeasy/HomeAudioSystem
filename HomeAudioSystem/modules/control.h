#ifndef MODULES_CONTROL_H_INC
#define MODULES_CONTROL_H_INC
/*
    control.h - declarations relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/


void ctrl_init();
void ctrl_serial_isr();
void ctrl_worker();

#endif
