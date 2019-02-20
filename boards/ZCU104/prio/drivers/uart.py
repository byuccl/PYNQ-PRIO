import asyncio, time


class UART() :   

    RX_OFFSET = 0x00
    TX_OFFSET = 0x04
    STATUS_OFFSET = 0x08
    CONTROL_OFFSET = 0x0C


    RX_AVAIL_BIT = 0x01
    RX_FULL_BIT = 0x02
    TX_EMPTY_BIT = 0x04
    TX_FULL_BIT = 0x08

    RST_FIFO_BIT = 0x02

    CTRL_BIT_EN_INT = 0x10
    CTRL_BIT_DIS_INT = 0XEF

    def __init__(self, pr_region, name=None):
        self._pr_region = pr_region
        
        if name is None:
            self.name = "UART_" + str(_pr_region)
        else:
            self.name = name

    def txReady(self):
        cur_val = self._pr_region.read(self.STATUS_OFFSET)
        return not (cur_val & self.TX_FULL_BIT)

    def rxAvail(self):
        cur_val = self._pr_region.read(self.STATUS_OFFSET)
        return  (cur_val & self.RX_AVAIL_BIT) == self.RX_AVAIL_BIT

    def enableInterrupts(self, enable):
        ctrl = self._pr_region.read(self.CONTROL_OFFSET)
        if enable:
            ctrl |= self.CTRL_BIT_EN_INT
        else:
            ctrl &= self.CTRL_BIT_DIS_INT
        self._pr_region.write(self.CONTROL_OFFSET, ctrl)

    def write(self, msg):
        for b in msg:
            # Wait for ready to send
            while not self.txReady():
                pass

            # Send data
            self._pr_region.write(self.TX_OFFSET, b)
        
    def readRxByte(self):
        byte = self._pr_region.read(self.RX_OFFSET)
        return byte

    def WriteTxByte(self, byte):
        # Wait for ready to send
        while not self.txReady():
            pass

        self._pr_region.write(self.TX_OFFSET, byte)

    #timeout_secs can be initialized to None to disable timeout
    def read(self, size=1, timeout_secs=1):
        recvd = []
        timeout = _Timeout(timeout_secs)
        while len(recvd) < size:
            #waits for data to be available
            while not self.rxAvail() and not timeout.expired():
                pass

            #exits if time has expired.
            if timeout.expired():
                break

            recvd.append(self._pr_region.read(self.RX_OFFSET))
        

        return recvd
        
        
    def printStatus(self):
        status = self._pr_region.read(self.STATUS_OFFSET)
        print(self.name + " status:")
        print("\tRX Available: " + str((status & self.RX_AVAIL_BIT) == self.RX_AVAIL_BIT))
        print("\tRX Full: " + str((status & self.RX_FULL_BIT) == self.RX_FULL_BIT))
        print("\tTX Empty: " + str((status & self.TX_EMPTY_BIT) == self.TX_EMPTY_BIT))
        print("\tTX Full: " + str((status & self.TX_FULL_BIT) == self.TX_FULL_BIT))
        print("\tInterrupts Enabled: " + str((status & self.CTRL_BIT_EN_INT) == self.CTRL_BIT_EN_INT))
       
       
    def resetFIFOs(self):
        self._pr_region.write(self.CONTROL_OFFSET, self.RST_FIFO_BIT) 
        
    
    # Run this interrupt handler until all messages have been received
    # msg_size - Number of bytes to wait for (if 0, run forever)
    async def isr_recv(self, msg_size = 0):
        recvd_msg = []
        while True:
            await self._pr_region.interrupt.wait()
            if self.rxAvail():
                recvd = self.readRxByte()
                recvd_msg.append(recvd)                

                if msg_size > 0:
                    print(self.name  + " isr received byte #" + str(len(recvd_msg)) + " of " + str(msg_size) + ": " + hex(recvd))                
                    if (len(recvd_msg) == msg_size):                        
                        return recvd_msg
                else:
                    print(self.name + " isr received byte #" + str(len(recvd_msg)) + ": " + hex(recvd))                



# This class is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2001-2016 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause
class _Timeout(object):
    """\
    Abstraction for timeout operations. Using time.monotonic() if available
    or time.time() in all other cases.
    The class can also be initialized with 0 or None, in order to support
    non-blocking and fully blocking I/O operations. The attributes
    is_non_blocking and is_infinite are set accordingly.
    """
    if hasattr(time, 'monotonic'):
        # Timeout implementation with time.monotonic(). This function is only
        # supported by Python 3.3 and above. It returns a time in seconds
        # (float) just as time.time(), but is not affected by system clock
        # adjustments.
        TIME = time.monotonic
    else:
        # Timeout implementation with time.time(). This is compatible with all
        # Python versions but has issues if the clock is adjusted while the
        # timeout is running.
        TIME = time.time

    def __init__(self, duration):
        """Initialize a timeout with given duration"""
        self.is_infinite = (duration is None)
        self.is_non_blocking = (duration == 0)
        self.duration = duration
        if duration is not None:
            self.target_time = self.TIME() + duration
        else:
            self.target_time = None

    def expired(self):
        """Return a boolean, telling if the timeout has expired"""
        return self.target_time is not None and self.time_left() <= 0

    def time_left(self):
        """Return how many seconds are left until the timeout expires"""
        if self.is_non_blocking:
            return 0
        elif self.is_infinite:
            return None
        else:
            delta = self.target_time - self.TIME()
            if delta > self.duration:
                # clock jumped, recalculate
                self.target_time = self.TIME() + self.duration
                return self.duration
            else:
                return max(0, delta)

    def restart(self, duration):
        """\
        Restart a timeout, only supported if a timeout was already set up
        before.
        """
        self.duration = duration
        self.target_time = self.TIME() + duration
