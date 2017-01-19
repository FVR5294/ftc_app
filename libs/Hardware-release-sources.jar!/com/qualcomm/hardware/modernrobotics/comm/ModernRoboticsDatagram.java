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

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.TypeConversion;

/**
 * {@link ModernRoboticsDatagram} understands the basic structure of datagrams sent to
 * and from Modern Robotics controllers.
 */
@SuppressWarnings("WeakerAccess")
@SuppressLint("DefaultLocale")
public abstract class ModernRoboticsDatagram
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final int CB_HEADER = 5;
    public static final int IB_SYNC_0 = 0;
    public static final int IB_SYNC_1 = 1;
    public static final int IB_FUNCTION = 2;
    public static final int IB_ADDRESS = 3;
    public static final int IB_LENGTH = 4;

    public byte[] data;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    protected ModernRoboticsDatagram(int sync0, int sync1, int cbPayload)
        {
        data = new byte[CB_HEADER + cbPayload];
        data[IB_SYNC_0] = (byte)sync0;
        data[IB_SYNC_1] = (byte)sync1;
        }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    public boolean isRead()
        {
        return (data[IB_FUNCTION] & 0x80) != 0;
        }

    public boolean isWrite()
        {
        return !isRead();
        }

    public void setRead(int function)
        {
        data[IB_FUNCTION] = (byte)(0x80 | (function & 0x7F));
        }

    public void setWrite(int function)
        {
        data[IB_FUNCTION] = (byte)(function & 0x7F);
        }

    public void setRead()
        {
        setRead(getFunction());
        }

    public void setWrite()
        {
        setWrite(getFunction());
        }

    public int getFunction()
        {
        return data[IB_FUNCTION] & 0x7F;
        }

    public void setFunction(int function)
        {
        data[IB_FUNCTION] = (byte)((data[IB_FUNCTION] & 0x80) | (function & 0x7F));
        }

    public int getAddress()
        {
        return TypeConversion.unsignedByteToInt(data[IB_ADDRESS]);
        }

    public void setAddress(int address)
        {
        if (address < 0 || address > 255) throw new IllegalArgumentException(String.format("address=%d; must be unsigned byte", address));
        data[IB_ADDRESS] = (byte)address;
        }

    public void setPayload(byte[] payload)
        {
        setPayloadLength(payload.length);
        System.arraycopy(payload, 0, data, CB_HEADER, payload.length);
        }

    public int getPayloadLength()
        {
        return TypeConversion.unsignedByteToInt(data[IB_LENGTH]);
        }

    public void setPayloadLength(int length)
        {
        if (length < 0 || length > 255) throw new IllegalArgumentException(String.format("length=%d; must be unsigned byte", length));
        data[IB_LENGTH] = (byte)length;
        }

    public boolean isFailure()
        {
        return data[IB_FUNCTION] == (byte)0xFF && data[IB_ADDRESS] == (byte)0xFF;
        }
    }
