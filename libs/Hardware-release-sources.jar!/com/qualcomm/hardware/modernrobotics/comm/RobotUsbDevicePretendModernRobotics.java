/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.hardware.modernrobotics.comm;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.internal.CircularByteBuffer;

/**
 * This class implements a dummy RobotUsbDevice that will apparently successfully do reads and
 * writes but doesn't actually do anything.
 */
@SuppressWarnings("WeakerAccess")
public class RobotUsbDevicePretendModernRobotics implements RobotUsbDevice
    {
    protected FirmwareVersion           firmwareVersion = null;
    protected CircularByteBuffer        circularByteBuffer = new CircularByteBuffer(0);
    protected boolean                   interruptRequested = false;
    protected SerialNumber              serialNumber;
    protected DeviceManager.DeviceType  deviceType = DeviceManager.DeviceType.FTDI_USB_UNKNOWN_DEVICE;

    public RobotUsbDevicePretendModernRobotics(SerialNumber serialNumber)
        {
        this.serialNumber = serialNumber;
        }
    @Override @NonNull public SerialNumber getSerialNumber()
        {
        return this.serialNumber;
        }
    @Override public void setDeviceType(@NonNull DeviceManager.DeviceType deviceType)
        {
        this.deviceType = deviceType;
        }
    @Override @NonNull public DeviceManager.DeviceType getDeviceType()
        {
        return this.deviceType;
        }
    @Override public void close()
        {
        }
    @Override public boolean isOpen()
        {
        return true;
        }
    @Override public void setBaudRate(int i) throws RobotCoreException
        {
        }
    @Override public void setDataCharacteristics(byte b, byte b1, byte b2) throws RobotCoreException
        {
        }
    @Override public void setLatencyTimer(int i) throws RobotCoreException
        {
        }
    @Override public void setBreak(boolean enable) throws RobotCoreException
        {
        }
    @Override public void purge(RobotUsbDevice.Channel channel) throws RobotCoreException
        {
        }
    @Override public int read(byte[] bytes) throws RobotCoreException, InterruptedException
        {
        return this.read(bytes, bytes.length, 0/*bogus*/);
        }
    @Override public void write(byte[] bytes) throws RobotCoreException
        {
        // Interpret the byte data. Note: this only works because higher levels only ever write
        // whole requests in one fell swoop
        ModernRoboticsRequest request = ModernRoboticsRequest.from(bytes);
        if (0 != request.getFunction()) throw new IllegalArgumentException("undefined function: " + request.getFunction());

        // Generate an appropriate response
        ModernRoboticsResponse response;
        if (request.isWrite())
            {
            response = new ModernRoboticsResponse(0);
            response.setWrite(request.getFunction());
            response.setAddress(request.getAddress());
            }
        else
            {
            response = new ModernRoboticsResponse(request.getPayloadLength());
            response.setRead(request.getFunction());
            response.setAddress(request.getAddress());
            response.setPayloadLength(request.getPayloadLength());
            // Payload data will already be initialized as zero
            }

        // Remember the response for a subsequent read
        circularByteBuffer.write(response.data);

        // We have a bit of a dilemma: we don't actually need to take any time to, e.g.,
        // communicate with an actual piece of hardware like a non-pretend device does.
        // As a result, were we to do nothing, our ReadWriteRunnable would spin through its
        // run() loop lickity-split, much faster than a real device. That just pointlessly eats
        // up cycles.
        //
        // One way around that would be to have that run() loop actually *block* if there's
        // nothing for it to do. For the moment though we just slow things down here to
        // better match real devices.
        //
        // The constant used here (3.5ms) has been observed in a non-scientific study to roughly
        // approximate the time taken for a RobotUsbDevice write() followed by a read(). But only
        // roughly. We could break that into two sleep()s, one in write() and the other in
        // read(), but we don't care enough about verisimilitude to be bothered.
        try {
            Thread.sleep(3, 500000);
            }
        catch (InterruptedException e)
            {
            Thread.currentThread().interrupt();
            }
        }

    @Override public int read(byte[] bytes, int cbReadExpected, int timeout) throws RobotCoreException, InterruptedException
        {
        return read(bytes, cbReadExpected, (long)timeout);
        }

    @Override public int read(byte[] bytes, int cbReadExpected, long timeout) throws RobotCoreException
        {
        // Return what we can
        return circularByteBuffer.read(bytes, 0, cbReadExpected);
        }

    @Override
    public FirmwareVersion getFirmwareVersion()
        {
        return firmwareVersion;
        }

    @Override
    public void setFirmwareVersion(FirmwareVersion version)
        {
        this.firmwareVersion = version;
        }

    @Override
    public void requestInterrupt()
        {
        this.interruptRequested = true;
        }

    @Override
    public USBIdentifiers getUsbIdentifiers()
        {
        USBIdentifiers result = new USBIdentifiers();
        result.vendorId = 0x0403;   // FTDI
        result.productId = 0;       // fake
        result.bcdDevice = 0;       // fake
        return result;
        }
    }
