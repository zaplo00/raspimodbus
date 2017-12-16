/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: zaplo00
 *
 * Created on 15. joulukuuta 2017, 17:16
 */
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <modbus/modbus.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

char TimeString[128];

std::ostream &log() {
    return std::cout << "[" << TimeString << "] ";
}

std::ostream &loge() {
    return std::cerr << "[" << TimeString << "] ";
}

void init_log_time() {
    static int secondsLast = 99;

    timeval curTime;
    gettimeofday(&curTime, NULL);
    if (secondsLast == curTime.tv_sec)
        exit(-1);

    secondsLast = curTime.tv_sec;

    strftime(TimeString, 80, "%d.%m.%Y %H:%M:%S", localtime(&curTime.tv_sec));
}

#define DEVICE_FILE_NAME "/dev/vcio"
#define MAJOR_NUM 100
#define MAX_WORDS 256
#define IOCTL_MBOX_PROPERTY _IOWR(MAJOR_NUM, 0, char *)

// use ioctl to send mbox property message

static int mbox_property(int desc, void *buf)
{
    int ret = ioctl(desc, IOCTL_MBOX_PROPERTY, buf);
    
    if (ret < 0)
    {
        loge() << "ioctl_set_msg failed " << ret << std::endl;
    }
    
    return ret;
}

static int mbox_open()
{
    int desc;
    
    //char device to communicate with kernel mbox driver
    desc = open(DEVICE_FILE_NAME, 0);
    if (desc < 0)
    {
        loge() << "Failed to open device " << DEVICE_FILE_NAME << std::endl;
        loge() << "Try creating device file with mknod " << DEVICE_FILE_NAME << " c " << MAJOR_NUM << " 0" << std::endl;
        exit(-1);
    }
    return desc;
}

static void mbox_close(int desc)
{
    close(desc);
}

int getGPUTemp()
{
    int mb = mbox_open(); // videocore mailbox
    uint32_t __attribute__((aligned(16))) tag[8]; //word align
    
    tag[0] = sizeof(tag); //buffer size in bytes (including the header values, the end tag and padding)
    tag[1] = 0; //buffer request/response code (process request 0x0)
    tag[2] = 0x30006; //tag id (get temperature)
    tag[3] = 8; //size of the buffer
    tag[4] = 4; //size of the data
    tag[5] = 0; //temperature id 0
    tag[6] = 0;
    tag[7] = 0;
    
    int s = mbox_property(mb, tag);    
    mbox_close(mb);
    
    if (s != -1)
        return tag[6]; //thousandth of GPU temperature in celsius
    
    return -123;
}

int main(int argc, char** argv) {

    init_log_time();

    modbus_t *ctx;
    modbus_mapping_t *mbMapping = modbus_mapping_new(
            0, //R/W coils (bool values)
            0, //read only coils
            0, //R/W registers
            1  //read only registers
    ); 

    ctx = modbus_new_tcp_pi("192.168.1.6", "1502");
    if (!ctx) {
        loge() << "Unable to allocate libmodbus context" << std::endl;
        return -1;
    }

    if (!mbMapping) {
        loge() << "Failed to allocate the mapping: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }

    //R/W 
    //mbMapping->tab_bits[0] = TRUE;
    //mbMapping->tab_registers[0] = 623;

    //Read only    
    //modbus_set_bits_from_byte(mbMapping->tab_input_bits, 0, 1); //index 0 true
    mbMapping->tab_input_registers[0] = getGPUTemp();

    int ret = modbus_set_slave(ctx, MODBUS_TCP_SLAVE);
    if (ret == -1) {
        loge() << "set slave failed: " << modbus_strerror(errno) << std::endl;
        modbus_mapping_free(mbMapping);
        modbus_free(ctx);
        return -1;
    }

    int sock = -1;
    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH];
    int len = 0;

    sock = modbus_tcp_pi_listen(ctx, 1);
    modbus_tcp_pi_accept(ctx, &sock);

    while (1) {
        do {
            len = modbus_receive(ctx, &req[0]);
            if (len == -1) {
                loge() << "Recv failed: " << modbus_strerror(errno) << std::endl;
                
                modbus_mapping_free(mbMapping);
                modbus_free(ctx);
                return -1;
            }
        } while (len == 0);

        len = modbus_reply(ctx, req, len, mbMapping);
        if (len == -1)
            break;
    }

    modbus_mapping_free(mbMapping);
    modbus_free(ctx);

    return EXIT_SUCCESS;
}

