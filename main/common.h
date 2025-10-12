//
// Created by felix on 10.10.25.
//

#ifndef T2V_MODULE_FW_COMMON_H
#define T2V_MODULE_FW_COMMON_H

#define FW_VERSION_MAJOR (0)
#define FW_VERSION_MINOR (1)
#define FW_VERSION_PATCH (0)

#define TAG_T2V_MODULE "tv2_module"
#define TAG_T2V_MODULE_NEC_RCV "t2v_module::nec"
#define TAG_T2V_MODULE_NEC_DECODER "t2v_module::nec::decoder"
#define TAG_T2V_MODULE_USB "t2v_module::usb"


// Entry points for tasks

void ir_nec_task_main(void*);
void usb_device_task_main(void*);


#endif //T2V_MODULE_FW_COMMON_H