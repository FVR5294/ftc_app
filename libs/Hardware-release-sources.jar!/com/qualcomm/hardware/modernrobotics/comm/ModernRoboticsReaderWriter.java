/*
 * Copyright (c) 2014, 2015, 2016 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 /*
Copyright (c) 2016-2017 Robert Atkinson

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

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice.Channel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.TimeoutException;

@SuppressWarnings("WeakerAccess")
@SuppressLint("DefaultLocale")
public class ModernRoboticsReaderWriter
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String TAG = "MRReaderWriter";
    public static boolean DEBUG = false;

    /**
     * The timeouts defined by the MR USB spec are subtle. Our historical timeout of 100ms
     * to read the header and the 100ms to read the payload was demonstrably insufficient, as teams
     * were hitting the payload timeout in the wild. Per the MR spec, the payload timeout should
     * be payload-length-dependent.
     *
     * Here's the skinny:
     *
     * The MR spec speaks thusly: "In order to detect a failure of too few Data payload bytes being
     * received, a timeout is used. This timeout is set to 10mS. This timeout will start as a soon
     * as the initial 0x55 is received. It is reset upon the arrival of each subsequent byte. [...]
     * A similar timeout process may be performed by the host. The host may also use a timeout of
     * 50mS for receipt of a response. If no response is received within 50mS, then it is assumed
     * that either the USB VCP communication link is non-operational, or the controller power is
     * not present"
     */
    public static int MS_INTER_BYTE_TIMEOUT            = 10;
    public static int MS_USB_HUB_LATENCY               = 2;    // this is probably way overkill
    public static int MS_REQUEST_RESPONSE_TIMEOUT      = 50 + MS_USB_HUB_LATENCY * 2;

    /**
     * We run on a garbage collected system. That doesn't run very often, and doesn't run back to
     * back, only one at a time, but when it does run it shuts things down for 15ms-40ms. One of
     * those can happen in the middle of any of our timeouts. So we need to allow for same.
     */
    public static int MS_GARBAGE_COLLECTION_SPURT      = 40;

    /** We made this up: we want to be very generous in trying to recover from failures as we
     * read through possibly-old data trying to synchronize, and there's little harm in doing so */
    public static int MS_RESYNCH_TIMEOUT               = 1000;

    public static int MS_FAILURE_WAIT                  = 40;   // per email from MR
    public static int MS_COMM_ERROR_WAIT               = 100;  // historical. we made this up

    public static int MAX_SEQUENTIAL_USB_ERROR_COUNT   = 10;   // we made this up

    public final static String COMM_FAILURE_READ  = "comm failure read";
    public final static String COMM_FAILURE_WRITE = "comm failure write";
    public final static String COMM_TIMEOUT_READ  = "comm timeout read";
    public final static String COMM_TIMEOUT_WRITE = "comm timeout write";
    public final static String COMM_ERROR_READ    = "comm error read";
    public final static String COMM_ERROR_WRITE   = "comm error write";
    public final static String COMM_SYNC_LOST     = "comm sync lost";

    protected final RobotUsbDevice  device;
    protected int                   usbSequentialReadErrorCount = 0;
    protected int                   usbSequentialWriteErrorCount = 0;
    protected boolean               isSynchronized = false;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public ModernRoboticsReaderWriter(RobotUsbDevice device)
        {
        this.device = device;
        }

    public void throwIfUsbErrorCountIsTooHigh() throws RobotCoreException
        {
        if (this.usbSequentialReadErrorCount > MAX_SEQUENTIAL_USB_ERROR_COUNT || this.usbSequentialWriteErrorCount > MAX_SEQUENTIAL_USB_ERROR_COUNT)
            {
            throw new RobotCoreException("Too many sequential USB errors on device");
            }
        }

    public void close()
        {
        this.device.close();
        }

    //----------------------------------------------------------------------------------------------
    // Reading and writing
    //----------------------------------------------------------------------------------------------

    public void purge(Channel channel) throws RobotCoreException
        {
        this.device.purge(channel);
        }

    public void read(int address, byte[] buffer) throws RobotCoreException, InterruptedException
        {
        if (DEBUG) RobotLog.vv(TAG, "read(serial=%s addr=%d cb=%d)", device.getSerialNumber(), address, buffer.length);

        // Create and send a read request
        ModernRoboticsRequest request = new ModernRoboticsRequest(0);
        request.setRead(0);
        request.setAddress(address);
        request.setPayloadLength(buffer.length);
        this.device.write(request.data);

        try {
            // Read the response
            ModernRoboticsResponse response = readResponse(request);
            if (response.isFailure())
                {
                // In order to avoid having the MR firmware think we might be about to try to update
                // it with a new version, we wait a prescribed delay per MR information
                Thread.sleep(MS_FAILURE_WAIT);
                ++this.usbSequentialReadErrorCount;
                logAndThrow(request, response, COMM_FAILURE_READ);
                }
            else if (response.isRead()
                    && response.getFunction()==0
                    && response.getAddress()==address
                    && response.getPayloadLength()==buffer.length)
                {
                // Success! All is well.
                this.usbSequentialReadErrorCount = 0;
                System.arraycopy(response.data, ModernRoboticsDatagram.CB_HEADER, buffer, 0, buffer.length);
                }
            else
                {
                // Historical code had a sleep here. We don't believe that it's actually
                // necessary, but it's harmless, so we keep it (for now) as it's harmless
                // and only very rarely happens.
                Thread.sleep(MS_COMM_ERROR_WAIT);

                ++this.usbSequentialReadErrorCount;
                logAndThrow(request, response, COMM_ERROR_READ);
                }
            }
        catch (TimeoutException e)
            {
            // In *some cases* historical code had a sleep here, too, namely the 'incorrect sync
            // bytes' case. In the present code paths, that ends up timing out instead, as our
            // readResponse() intrinsically synchronizes. But we add the sleep here for good measure
            // as it's harmless, and only very rarely happens.
            Thread.sleep(MS_COMM_ERROR_WAIT);

            ++usbSequentialReadErrorCount;
            logAndThrow(e, request, timeoutMessage(COMM_TIMEOUT_READ, e));
            }
        }

    public void write(int address, byte[] buffer) throws RobotCoreException, InterruptedException
        {
        if (DEBUG) RobotLog.vv(TAG, "write(serial=%s addr=%d cb=%d)", device.getSerialNumber(), address, buffer.length);

        // Create and send a write request
        ModernRoboticsRequest request = new ModernRoboticsRequest(buffer.length);
        request.setWrite(0);
        request.setAddress(address);
        request.setPayload(buffer);
        this.device.write(request.data);

        try {
            // Read the response
            ModernRoboticsResponse response = readResponse(request);
            if (response.isFailure())
                {
                // In order to avoid having the MR firmware think we might be about to try to update
                // it with a new version, we wait a prescribed delay per MR information
                Thread.sleep(MS_FAILURE_WAIT);
                ++this.usbSequentialWriteErrorCount;
                this.logAndThrow(request, response, COMM_FAILURE_WRITE);
                }
            else if (response.isWrite()
                    && response.getFunction()==0
                    && response.getAddress()==address
                    && response.getPayloadLength()==0)
                {
                // All is well
                this.usbSequentialWriteErrorCount = 0;
                }
            else
                {
                // Historical code had a sleep here. We don't believe that it's actually
                // necessary, but it's harmless, so we keep it (for now) as it's harmless
                // and only very rarely happens.
                Thread.sleep(MS_COMM_ERROR_WAIT);

                ++this.usbSequentialWriteErrorCount;
                this.logAndThrow(request, response, COMM_ERROR_WRITE);
                }
            }
        catch (TimeoutException e)
            {
            // In *some cases* historical code had a sleep here, too, namely the 'incorrect sync
            // bytes' case. In the present code paths, that ends up timing out instead, as our
            // readResponse() intrinsically synchronizes. But we add the sleep here for good measure
            // as it's harmless, and only very rarely happens.
            Thread.sleep(MS_COMM_ERROR_WAIT);

            ++this.usbSequentialWriteErrorCount;
            this.logAndThrow(e, request, timeoutMessage(COMM_TIMEOUT_WRITE, e));
            }
        }

    //----------------------------------------------------------------------------------------------
    // Response reading
    //----------------------------------------------------------------------------------------------

    protected ModernRoboticsResponse readResponse(ModernRoboticsRequest request) throws RobotCoreException, InterruptedException, TimeoutException
        {
        ElapsedTime timer = new ElapsedTime();
        final int msTimeout = MS_RESYNCH_TIMEOUT;
        while (timer.milliseconds() < msTimeout)
            {
            // Read the header, synchronizing carefully if we need to
            ModernRoboticsResponse header = new ModernRoboticsResponse(0);
            if (!isSynchronized)
                {
                byte[] singleByte = new byte[1];
                byte[] headerSuffix = new byte[ModernRoboticsDatagram.CB_HEADER-2];

                // Synchronize by looking for the first synchronization byte
                if (readSingleByte(singleByte, MS_REQUEST_RESPONSE_TIMEOUT, "sync0") != ModernRoboticsResponse.syncBytes[0])
                    {
                    continue;
                    }

                // Having found the first, if we don't next see the second, then go back to looking for the first
                if (readSingleByte(singleByte, 0, "sync1") != ModernRoboticsResponse.syncBytes[1])
                    {
                    continue;
                    }

                // Read the remaining header bytes
                readIncomingBytes(headerSuffix, headerSuffix.length, 0, "syncSuffix");

                // Assemble the header from the pieces
                System.arraycopy(ModernRoboticsResponse.syncBytes, 0, header.data, 0, 2);
                System.arraycopy(headerSuffix,                     0, header.data, 2, headerSuffix.length);
                }
            else
                {
                readIncomingBytes(header.data, header.data.length, MS_REQUEST_RESPONSE_TIMEOUT, "header");
                if (!header.syncBytesValid())
                    {
                    // We've lost synchronization, yet we received *some* data. Thus, since there
                    // is no retransmission in the USB protocol, we're never going to get the response
                    // that we were looking for. So get out of Dodge with an error.
                    logAndThrow(request, header, COMM_SYNC_LOST);
                    }
                }

            // Read the data
            byte[] payload = new byte[header.getPayloadLength()];
            readIncomingBytes(payload, payload.length, 0, "payload");

            // We're ok to go more quickly the next time
            isSynchronized = true;

            // Assemble and return a response
            ModernRoboticsResponse result = new ModernRoboticsResponse(header.getPayloadLength());
            System.arraycopy(header.data, 0, result.data, 0,                  header.data.length);
            System.arraycopy(payload,     0, result.data, header.data.length, payload.length);
            return result;
            }

        throw newTimeoutException(true, String.format("timeout waiting %d ms for MR response", msTimeout));
        }

    protected void readIncomingBytes(byte[] buffer, int cbToRead, int msExtraTimeout, String debugContext) throws RobotCoreException, InterruptedException, TimeoutException
        {
        if (cbToRead > 0)
            {
            // In theory, per the MR spec, we might see MS_INTER_BYTE_TIMEOUT elapse
            // between each incoming byte. For a long-ish read response, that can amount
            // to some significant time: we routinely read 31 byte swaths of data on the
            // motor controller, for example, which would amount to nearly a third of a second
            // to see the response. Now that doesn't happen in practice: most inter-byte intervals
            // are actually much smaller than permitted. We could in theory do the MS_INTER_BYTE_TIMEOUT
            // timeouts on each individual byte, but that's cumbersome and inefficient given the read
            // API we have to work with. So instead we specify the pessimistic maximum, plus a little
            // extra to help avoid false hits, and thus see any communications failure a little
            // later than we otherwise would see (but not ridiculously later).
            //
            // Additionally, we allow for any extra timeout duration that has to do with other protocol
            // aspects unrelated to inter-byte timing.
            //
            // Finally, we allow for a garbage collection.
            //
            long msReadTimeout = MS_INTER_BYTE_TIMEOUT * (cbToRead + 2 /* just being generous */) + msExtraTimeout + MS_GARBAGE_COLLECTION_SPURT;

            int cbRead = device.read(buffer, cbToRead, msReadTimeout);
            if (cbRead == cbToRead)
                {
                // We got all the data we came for. Just return gracefully
                }
            else if (cbRead == 0)
                {
                // Couldn't read the data in the time allotted.
                throw newTimeoutException(false, String.format("%s: unable to read %d bytes in %d ms", debugContext, cbToRead, msReadTimeout));
                }
            else
                {
                // An unexpected error occurred
                RobotLog.ee(TAG, "readIncomingBytes(%s) cbToRead=%d cbRead=%d: throwing RobotCoreException", debugContext, cbToRead, cbRead);
                throw new RobotCoreException("readIncomingBytes(%s) cbToRead=%d cbRead=%d", debugContext, cbToRead, cbRead);
                }
            }
        }

    protected byte readSingleByte(byte[] buffer, int msExtraTimeout, String debugContext) throws RobotCoreException, InterruptedException, TimeoutException
        {
        readIncomingBytes(buffer, 1, msExtraTimeout, debugContext);
        return buffer[0];
        }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    protected String timeoutMessage(String root, TimeoutException e)
        {
        return String.format("%s: %s", root, e.getMessage());
        }

    protected void doExceptionBookkeeping() throws RobotCoreException
        {
        isSynchronized = false;
        }

    protected TimeoutException newTimeoutException(boolean log, String message) throws RobotCoreException
        {
        if (log) RobotLog.ee(TAG, "timeout: %s", message);
        //
        doExceptionBookkeeping();
        return new TimeoutException(message);
        }

    protected void logAndThrow(Exception e, ModernRoboticsRequest request, String message) throws RobotCoreException
        {
        ModernRoboticsRequest requestHeader = new ModernRoboticsRequest(0);  // Don't log the payload data
        System.arraycopy(request.data, 0, requestHeader.data, 0, requestHeader.data.length);
        //
        RobotLog.ee(TAG, "%s: %s: %s", device.getSerialNumber(), message, bufferToString(requestHeader.data));
        doExceptionBookkeeping();
        throw RobotCoreException.createChained(e, message);
        }

    protected void logAndThrow(ModernRoboticsRequest request, ModernRoboticsResponse response, String message) throws RobotCoreException
        {
        ModernRoboticsRequest requestHeader = new ModernRoboticsRequest(0);  // Don't log the payload data
        System.arraycopy(request.data, 0, requestHeader.data, 0, requestHeader.data.length);
        //
        ModernRoboticsResponse responseHeader = new ModernRoboticsResponse(0);  // Don't log the payload data
        System.arraycopy(response.data, 0, responseHeader.data, 0, responseHeader.data.length);
        //
        RobotLog.ee(TAG, "%s: %s -> %s", message, bufferToString(requestHeader.data), bufferToString(responseHeader.data));
        doExceptionBookkeeping();
        throw new RobotCoreException(message);
        }

    protected static String bufferToString(byte[] buffer)
        {
        StringBuilder result = new StringBuilder();
        result.append("[");
        if (buffer.length > 0)
            {
            result.append(String.format("%02x", buffer[0]));
            }

        int cbMax = 16;
        int cb = Math.min(buffer.length, cbMax);

        for (int ib = 1; ib < cb; ++ib)
            {
            result.append(String.format(" %02x", buffer[ib]));
            }

        if (cb < buffer.length)
            {
            result.append(" ...");
            }

        result.append("]");
        return result.toString();
        }
    }
