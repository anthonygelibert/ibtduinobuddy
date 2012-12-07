/******************************************************************************
 * Copyright (c) 2012, Anthony GELIBERT and Jessy GIACOMONI.                  *
 * All rights reserved.                                                       *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 *     * Redistributions of source code must retain the above copyright       *
 * notice, this list of conditions and the following disclaimer.              *
 *                                                                            *
 *     * Redistributions in binary form must reproduce the above copyright    *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A    *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER   *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,   *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY        *
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT               *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
 ******************************************************************************/

/*
 * Dependance : https://github.com/felis/USB_Host_Shield
 */

#include <Max3421e.h>
#include <Max3421e_constants.h>
#include <Usb.h>

#define IBUDDY_VID_LO      0x30  // Tenx Technology
#define IBUDDY_VID_HI      0x11
#define IBUDDY_ADDR        1
#define IBUDDY_EP          0x00
#define IBUDDY_IF          1
#define IBUDDY_TX_SPEED    115200

MAX3421E Max;
USB Usb;

void iBuddyInit(void);
void iBuddyTest(int i);

void setup(void)
{
    Serial.begin(IBUDDY_TX_SPEED);
    Max.powerOn();
    Serial.println("=== iBuddy Remote controller using BTM on Arduino UNO ===\n");
    delay(200);
}

void loop(void)
{
    byte rcode;
    byte incomingByte;
    byte usbstate;

    while (1)
    {
        delay(200);
        Max.Task();
        Usb.Task();
        usbstate = Usb.getUsbTaskState();
        switch(usbstate)
        {
            case(USB_ATTACHED_SUBSTATE_RESET_DEVICE):
                Serial.println("Device connected.");
                break;
            case (USB_STATE_CONFIGURING) :
                iBuddyInit();
                Serial.println("Device configured, you can now send your commands.");
                break;
            case(USB_STATE_ERROR):
                Serial.println("Problem with your iBuddy ! Disconnect it, NOW !");
                break;
            case (USB_STATE_RUNNING):
                if (Serial.available() > 0)
                {
                    // read the oldest byte in the serial buffer:
                    incomingByte = Serial.read();
                    switch (incomingByte)
                    {
                        case 'H':
                            Serial.println("Received request (TYPE 1)");
                            iBuddyTest(0);
                            break;
                        case 'h':
                            Serial.println("Received request (TYPE 2)");
                            iBuddyTest(0xFF);
                            break;
                    }
                }
                break;
        }
    }
}

/**
 * Initialize the iBuddy.<br/>
 * Before initializing the USB device, check its VID.
 */
void iBuddyInit(void)
{
    /* General purpose buffer for usb data */
    char buf[0x16] = {0};

    /* Copy device 0 endpoint information to device 1. */
    Usb.setDevTableEntry(IBUDDY_ADDR, Usb.getDevTableEntry(0,0));

    /* Get device description. */
    byte rcode = Usb.getDevDescr(IBUDDY_ADDR, IBUDDY_EP, 0x12 , buf);
    if(rcode)
    {
        Serial.print("Error attempting read device descriptor. Return code :");
        Serial.println(rcode, HEX);
        Usb.setUsbTaskState(USB_STATE_ERROR);
        return;
    }

    /* Check iBuddy VID. */
    if((buf[ 8 ] != IBUDDY_VID_LO) || (buf[ 9 ] != IBUDDY_VID_HI))
    {
        Serial.println("Unsupported USB Device");
        Usb.setUsbTaskState(USB_STATE_ERROR);
        return;
    }

    /* Configure device. */
    rcode = Usb.setConf(IBUDDY_ADDR, IBUDDY_EP, 0x01);
    if(rcode)
    {
        Serial.print("Error attempting to configure iBuddy. Return code :");
        Serial.println(rcode, HEX);
        Usb.setUsbTaskState(USB_STATE_ERROR);
        return;
    }

    /* iBuddy is now running */
    Usb.setUsbTaskState(USB_STATE_RUNNING);
}

/**
 * Simple function sending to different settings to iBuddy according to <i>i</i> value.
 *
 * @param i i==0 => CONFIG 1 // i!=0 => CONFIG 2
 */
void iBuddyTest(int i)
{
    char header[] = { 0x55, 0x53, 0x42, 0x43, 0x00, 0x40, 0x02, 0xFF };
    header[7] = i ? 0x5A : 0xED;

    byte rcode = Usb.setReport(IBUDDY_ADDR, IBUDDY_EP, sizeof(header),  IBUDDY_IF, 0x02, 0x00 , header);
    if(rcode)
    {
        Serial.print("Set report error: ");
        Serial.println(rcode, HEX);
        Usb.setUsbTaskState(USB_STATE_ERROR);
        return;
    }
}









