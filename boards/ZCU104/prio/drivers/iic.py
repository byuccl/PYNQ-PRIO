import asyncio
import time
# from pynq.interrupt import Interrupt

class XIicStats:
    def __init__(self):
        self.ArbitrationLost = 0 #/**< Number of times arbitration was lost */
        self.RepeatedStarts = 0 # /**< Number of repeated starts */
        self.BusBusy = 0 #    /**< Number of times bus busy status returned */
        self.RecvBytes = 0 #      /**< Number of bytes received */
        self.RecvInterrupts = 0 # /**< Number of receive interrupts */
        self.SendBytes = 0 #      /**< Number of transmit bytes received */
        self.SendInterrupts = 0 # /**< Number of transmit interrupts */
        self.TxErrors = 0 #       /**< Number of transmit errors (no ack) */
        self.IicInterrupts = 0#  /**< Number of IIC (device) interrupts */


class IIC():
    XST_SUCCESS = 0
    XST_INVALID_PARAM = 1
    XST_IIC_BUS_BUSY = 2
    XST_IIC_GENERAL_CALL_ADDRESS = 3 #  /*!< mastersend attempted with general call address*/

    XII_GENERAL_CALL_OPTION = 0x00000001
    XII_REPEATED_START_OPTION = 0x00000002
    XII_SEND_10_BIT_OPTION = 0x00000004

    XII_ADDR_TO_SEND_TYPE = 1 #/**< Bus address of slave device */
    XII_ADDR_TO_RESPOND_TYPE = 2 #/**< This device's bus address as slave */

    XIL_COMPONENT_IS_READY = 0x11111111
    XIL_COMPONENT_IS_STARTED = 0x22222222

    XIIC_DGIER_OFFSET   = 0x1C  #/**< Global Interrupt Enable Register */
    XIIC_IISR_OFFSET    = 0x20  #/**< Interrupt Status Register */
    XIIC_IIER_OFFSET    = 0x28  #/**< Interrupt Enable Register */
    XIIC_RESETR_OFFSET  = 0x40  #/**< Reset Register */
    XIIC_CR_REG_OFFSET  = 0x100 #/**< Control Register */
    XIIC_SR_REG_OFFSET  = 0x104 #/**< Status Register */
    XIIC_DTR_REG_OFFSET = 0x108 #/**< Data Tx Register */
    XIIC_DRR_REG_OFFSET = 0x10C #/**< Data Rx Register */
    XIIC_ADR_REG_OFFSET = 0x110 #/**< Address Register */
    XIIC_TFO_REG_OFFSET = 0x114 #/**< Tx FIFO Occupancy */
    XIIC_RFO_REG_OFFSET = 0x118 #/**< Rx FIFO Occupancy */
    XIIC_TBA_REG_OFFSET = 0x11C #/**< 10 Bit Address reg */
    XIIC_RFD_REG_OFFSET = 0x120 #/**< Rx FIFO Depth reg */
    XIIC_GPO_REG_OFFSET = 0x124 #/**< Output Register */

    XIIC_GINTR_ENABLE_MASK=0x80000000 #/**< Global Interrupt Enable Mask */

    XIIC_INTR_ARB_LOST_MASK=0x00000001 #/**< 1 = Arbitration lost */
    XIIC_INTR_TX_ERROR_MASK=0x00000002 #/**< 1 = Tx error/msg complete */
    XIIC_INTR_TX_EMPTY_MASK=0x00000004 #/**< 1 = Tx FIFO/reg empty */
    XIIC_INTR_RX_FULL_MASK=0x00000008 #/**< 1 = Rx FIFO/reg=OCY level */
    XIIC_INTR_BNB_MASK=0x00000010 #/**< 1 = Bus not busy */
    XIIC_INTR_AAS_MASK=0x00000020 #/**< 1 = When addr as slave */
    XIIC_INTR_NAAS_MASK=0x00000040 #/**< 1 = Not addr as slave */
    XIIC_INTR_TX_HALF_MASK=0x00000080 #/**< 1 = Tx FIFO half empty */

    XIIC_TX_INTERRUPTS=(XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_TX_EMPTY_MASK | XIIC_INTR_TX_HALF_MASK)
    XIIC_TX_RX_INTERRUPTS=(XIIC_INTR_RX_FULL_MASK | XIIC_TX_INTERRUPTS)

    XIIC_CR_ENABLE_DEVICE_MASK = 0x00000001 #/**< Device enable = 1 */
    XIIC_CR_TX_FIFO_RESET_MASK = 0x00000002 #/**< Transmit FIFO reset=1 */
    XIIC_CR_MSMS_MASK          = 0x00000004 #/**< Master starts Txing=1 */
    XIIC_CR_DIR_IS_TX_MASK     = 0x00000008 #/**< Dir of Tx. Txing=1 */
    XIIC_CR_NO_ACK_MASK        = 0x00000010 #/**< Tx Ack. NO ack = 1 */
    XIIC_CR_REPEATED_START_MASK= 0x00000020 #/**< Repeated start = 1 */
    XIIC_CR_GENERAL_CALL_MASK  = 0x00000040 #/**< Gen Call enabled = 1 */

    XIIC_SR_GEN_CALL_MASK      = 0x00000001 #/**< 1 = A Master issued a GC */
    XIIC_SR_ADDR_AS_SLAVE_MASK = 0x00000002 #/**< 1 = When addressed as slave */
    XIIC_SR_BUS_BUSY_MASK      = 0x00000004 #/**< 1 = Bus is busy */
    XIIC_SR_MSTR_RDING_SLAVE_MASK = 0x00000008 #/**< 1 = Dir: Master <-- slave */
    XIIC_SR_TX_FIFO_FULL_MASK = 0x00000010 #/**< 1 = Tx FIFO full */
    XIIC_SR_RX_FIFO_FULL_MASK = 0x00000020 #/**< 1 = Rx FIFO full */
    XIIC_SR_RX_FIFO_EMPTY_MASK =0x00000040 #/**< 1 = Rx FIFO empty */
    XIIC_SR_TX_FIFO_EMPTY_MASK =0x00000080 #/**< 1 = Tx FIFO empty */

    XIIC_READ_OPERATION = 1 #/**< Read operation on the IIC bus */
    XIIC_WRITE_OPERATION = 0 #/**< Write operation on the IIC bus */

    IIC_TX_FIFO_DEPTH = 16 #/**< Tx fifo capacity */
    IIC_RX_FIFO_DEPTH = 16 #/**< Rx fifo capacity */

    XIIC_TX_ADDR_SENT = 0x00
    XIIC_TX_ADDR_MSTR_RECV_MASK = 0x02

    XIIC_RESET_MASK = 0x0000000A

    XIIC_MASTER_ROLE = 1 # /**< Master on the IIC bus */
    XIIC_SLAVE_ROLE = 0 #/**< Slave on the IIC bus */

    def __init__(self, partial_region):
        self._partial_region = partial_region

        self.isStarted = 0
        self.RecvHandler = None
        self.recvBuf = None

        self.SendHandler = None
        self.SendCallBackRef = None

        self.SendBuffer = None
        self.SendBufferIndex = None

        self.StatusHandler = None

        self.has10BitAddr = False

        self.IsReady = self.XIL_COMPONENT_IS_READY
        self.Options = 0
        self.BNBOnly = False

        self.isDynamic = False

        self.AddrOfSlave = None

        self.SendByteCount = 0
        self.RecvByteCount = 0

        self.addrAsSlaveFuncPtr = None
        self.notAddrAsSlaveFuncPtr = None
        self.recvSlaveFuncPtr = None
        self.sendSlaveFuncPtr = None
        self.recvMasterFuncPtr = None
        self.sendMasterFuncPtr = None
        self.arbLostFuncPtr = None
        self.busNotBusyFuncPtr = None

        self.Stats = XIicStats()

        self.CallBackRefCounter = 0

        self.reset()

        self.shutoffInterruptHandler = asyncio.Event()

    def readReg(self, offset):
        return self._partial_region.read(offset)

    def writeReg(self, offset, val):
        return self._partial_region.write(offset, val)

    def reset(self):
        assert(self.IsReady ==self.XIL_COMPONENT_IS_READY)
        self.writeReg(self.XIIC_RESETR_OFFSET,self.XIIC_RESET_MASK)

    def getCallBackRefCount(self):
        x = self.CallBackRefCounter
        self.CallBackRefCounter += 1
        return x

    def send7BitAddr (self, SlaveAddress, Operation):
        LocalAddr = SlaveAddress << 1
        LocalAddr = (LocalAddr & 0xFE) | (Operation)
        self.writeReg(self.XIIC_DTR_REG_OFFSET, LocalAddr)

    def setAddress(self, AddressType,  Address):
        assert Address < 1023

        # /*
        #  * Set address to respond to for this device into address registers.
        #  */
        if (AddressType == self.XII_ADDR_TO_RESPOND_TYPE):
            # /*
            #  * Address in upper 7 bits.
            #  */
            SendAddr = ((Address & 0x007F) << 1)
            self.writeReg(self.XIIC_ADR_REG_OFFSET, SendAddr)

            if (self.Has10BitAddr):
                # /*
                #  * Write upper 3 bits of addr to DTR only when 10 bit
                #  * option included in design i.e. register exists.
                #  */
                SendAddr = ((Address & 0x0380) >> 7)
                self.writeReg(self.XIIC_TBA_REG_OFFSET, SendAddr)

            return self.XST_SUCCESS
        # /*
        #  * Store address of slave device being read from.
        #  */
        if (AddressType == self.XII_ADDR_TO_SEND_TYPE):
            self.AddrOfSlave = Address
            return self.XST_SUCCESS

        return self.XST_INVALID_PARAM


    def writeSendByte(self):
        self.writeReg(self.XIIC_DTR_REG_OFFSET,self.SendBuffer[self.SendBufferIndex])
        self.SendBufferIndex += 1
        self.SendByteCount -= 1
        self.Stats.SendBytes += 1

    def readRecvByte(self):
        self.RecvBuffer[self.RecvBufferIndex] = self.readReg(self.XIIC_DRR_REG_OFFSET)
        self.RecvBufferIndex += 1
        self.RecvByteCount -= 1
        self.Stats.RecvBytes += 1


    def transmitFifoFill(self, Role):
        # /*
        #  * Determine number of bytes to write to FIFO. Number of bytes that
        #  * can be put into the FIFO is (FIFO depth) - (current occupancy + 1)
        #  * When more room in FIFO than msg bytes put all of message in the FIFO.
        #  */
        AvailBytes = self.IIC_TX_FIFO_DEPTH - self.readReg(self.XIIC_TFO_REG_OFFSET) + 1

        if (self.SendByteCount > AvailBytes):
            NumBytesToSend = AvailBytes
        else:
            # /*
            #  * More space in FIFO than bytes in message.
            #  */
            if ((self.Options & self.XII_REPEATED_START_OPTION) or (Role == self.XIIC_SLAVE_ROLE)):
                NumBytesToSend = self.SendByteCount
            else:
                NumBytesToSend = self.SendByteCount - 1

        # /*
        #  * Fill FIFO with amount determined above.
        #  */
        for _ in range(NumBytesToSend):
            self.writeSendByte()

    def setControlRegister(self, ControlRegister, ByteCount):
        ControlRegister &= ~(self.XIIC_CR_NO_ACK_MASK | self.XIIC_CR_DIR_IS_TX_MASK)

        if self.Options & self.XII_SEND_10_BIT_OPTION:
            ControlRegister |= self.XIIC_CR_DIR_IS_TX_MASK

        elif ByteCount == 1:
            ControlRegister |= self.XIIC_CR_NO_ACK_MASK

    def intrGlobalEnable(self):
        self.writeReg(self.XIIC_DGIER_OFFSET, self.XIIC_GINTR_ENABLE_MASK)

    def intrGlobalDisable(self):
        self.writeReg(self.XIIC_DGIER_OFFSET, 0)

    def writeIier(self, Enable):
        self.writeReg(self.XIIC_IIER_OFFSET, (Enable))

    def readIier(self):
        return self.readReg(self.XIIC_IIER_OFFSET)

    def writeIisr(self, Status):
        self.writeReg(self.XIIC_IISR_OFFSET, (Status))

    def readIisr(self):
        return self.readReg(self.XIIC_IISR_OFFSET)

    def disableIntr(self, InterruptMask):
        self.writeIier(self.readIier() & ~(InterruptMask))

    def enableIntr(self, InterruptMask):
        self.writeIier(self.readIier() | (InterruptMask))

    def clearEnableIntr(self, InterruptMask):
        self.writeIisr(self.readIisr() & (InterruptMask))
        self.writeIier(self.readIier() | (InterruptMask))

    def clearIntr(self, InterruptMask):
        self.writeIisr(self.readIisr() & (InterruptMask))

    def isIntrGlobalEnabled(self):
        return self.readReg(self.XIIC_DGIER_OFFSET) == self.XIIC_GINTR_ENABLE_MASK

    def IsBusBusy(self):
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        StatusReg = self.readReg(self.XIIC_SR_REG_OFFSET)

        # /*
        #  * If this device is already master of the bus as when using the
        #  * repeated start and the bus is busy setup to wait for it to not be
        #  * busy.
        #  */
        if (((CntlReg & self.XIIC_CR_MSMS_MASK) == 0) and (StatusReg & self.XIIC_SR_BUS_BUSY_MASK)):
            # /*
            #  * The bus is busy, clear pending BNB interrupt incase
            #  * previously set and then enable BusNotBusy interrupt.
            #  */
            self.BNBOnly = True
            self.clearEnableIntr(self.XIIC_INTR_BNB_MASK)
            return True
        return False

    def SendMasterData(self):
        # /*
        #  * The device is a master on the bus.  If there is still more address
        #  * bytes to send when in master receive operation and the slave device
        #  * is 10 bit addressed.
        #  * This requires the lower 7 bits of address to be resent when the mode
        #  * switches to Read instead of write (while sending addresses).
        #  */
        if (self.TxAddrMode & self.XIIC_TX_ADDR_MSTR_RECV_MASK):
            # /*
            #  * Send the 1st byte of the slave address in the read operation
            #  * and change the state to indicate this has been done
            #  */
            self.SendSlaveAddr()
            self.TxAddrMode = IIC.XIIC_TX_ADDR_SENT

        # /*
        #  * In between 1st and last byte of message, fill the FIFO with more data
        #  * to send, disable the 1/2 empty interrupt based upon data left to
        #  * send.
        #  */
        elif (self.SendByteCount > 1):
            self.transmitFifoFill(IIC.XIIC_MASTER_ROLE)

            if (self.SendByteCount < 2):
                self.disableIntr(IIC.XIIC_INTR_TX_HALF_MASK)

        # /*
        #  * If there is only one byte left to send, processing differs between
        #  * repeated start and normal messages.
        #  */
        elif (self.SendByteCount == 1):
            # /*
            #  * When using repeated start, another interrupt is expected
            #  * after the last byte has been sent, so the message is not
            #  * done yet.
            #  */
            if (self.Options & IIC.XII_REPEATED_START_OPTION):
                self.writeSendByte()

            # /*
            #  * When not using repeated start, the stop condition must be
            #  * generated after the last byte is written. The bus is
            #  * throttled waiting for the last byte.
            #  */
            else:
                # /*
                #  * Set the stop condition before sending the last byte
                #  * of data so that the stop condition will be generated
                #  * immediately following the data another transmit
                #  * interrupt is not expected so the message is done.
                #  */
                CntlReg = self.readReg(IIC.XIIC_CR_REG_OFFSET)
                CntlReg &= ~IIC.XIIC_CR_MSMS_MASK
                self.writeReg(IIC.XIIC_CR_REG_OFFSET, CntlReg)
                self.writeSendByte()

                # /*
                #  * Wait for bus to not be busy before declaring message
                #  * has been sent for the no repeated start operation.
                #  * The callback will be called from the BusNotBusy part
                #  * of the Interrupt handler to ensure that the message
                #  * is completely sent.
                #  * Disable the Tx interrupts and enable the BNB
                #  * interrupt.
                #  */

                self.BNBOnly = False
                self.disableIntr(IIC.XIIC_TX_INTERRUPTS)
                self.enableIntr(IIC.XIIC_INTR_BNB_MASK)

        else:
            if (self.Options & IIC.XII_REPEATED_START_OPTION):

                # /*
                #  * The message being sent has completed. When using
                #  * repeated start with no more bytes to send repeated
                #  * start needs to be set in the control register so
                #  * that the bus will still be held by this master.
                #  */
                CntlReg = self.readReg(IIC.XIIC_CR_REG_OFFSET)
                CntlReg |= IIC.XIIC_CR_REPEATED_START_MASK
                self.writeReg(IIC.XIIC_CR_REG_OFFSET, CntlReg)

                # /*
                #  * If the message that was being sent has finished,
                #  * disable all transmit interrupts and call the callback
                #  * that was setup to indicate the message was sent,
                #  * with 0 bytes remaining.
                #  */

                self.disableIntr(IIC.XIIC_TX_INTERRUPTS)
                self.SendHandler(self.SendCallBackRef, 0)


    def RecvMasterData(self):
        # /*
        #  * Device is a master receiving, get the contents of the control
        #  * register and determine the number of bytes in fifo to be read out.
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        BytesInFifo = self.readReg(self.XIIC_RFO_REG_OFFSET) + 1

        # /*
        #  * If data in FIFO holds all data to be retrieved - 1, set NOACK and
        #  * disable the Tx error.
        #  */
        if ((self.RecvByteCount - BytesInFifo) == 1):
            # /*
            #  * Disable Tx error interrupt to prevent interrupt
            #  * as this device will cause it when it set NO ACK next.
            #  */
            self.disableIntr(self.XIIC_INTR_TX_ERROR_MASK)
            self.clearIntr(self.XIIC_INTR_TX_ERROR_MASK)

            # /*
            #  * Write control reg with NO ACK allowing last byte to
            #  * have the No ack set to indicate to slave last byte read.
            #  */
            self.writeReg(self.XIIC_CR_REG_OFFSET, (CntlReg | self.XIIC_CR_NO_ACK_MASK))

            # /*
            #  * Read one byte to clear a place for the last byte to be read
            #  * which will set the NO ACK.
            #  */
            self.readRecvByte()

        # /*
        #  * If data in FIFO is all the data to be received then get the data
        #  * and also leave the device in a good state for the next transaction.
        #  */
        elif (self.RecvByteCount - BytesInFifo) == 0:
            # /*
            #  * If repeated start option is off then the master should stop
            #  * using the bus, otherwise hold the bus, setting repeated start
            #  * stops the slave from transmitting data when the FIFO is read.
            #  */
            if (self.Options & self.XII_REPEATED_START_OPTION) == 0:
                CntlReg &= ~self.XIIC_CR_MSMS_MASK
            else:
                CntlReg |= self.XIIC_CR_REPEATED_START_MASK
            self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)

            # /*
            #  * Read data from the FIFO then set zero based FIFO read depth
            #  * for a byte.
            #  */
            for LoopCnt in range(0, BytesInFifo):
                self.readRecvByte()

            self.writeReg(self.XIIC_RFD_REG_OFFSET, 0)

            # /*
            #  * Disable Rx full interrupt and write the control reg with ACK
            #  * allowing next byte sent to be acknowledged automatically.
            #  */
            self.disableIntr(self.XIIC_INTR_RX_FULL_MASK)
            self.writeReg(self.XIIC_CR_REG_OFFSET, (CntlReg & ~self.XIIC_CR_NO_ACK_MASK))

            # /*
            #  * Send notification of msg Rx complete in RecvHandler callback.
            #  */
            self.RecvHandler(self.RecvCallBackRef, 0)
        else:
            # /*
            #  * Fifo data not at n-1, read all but the last byte of data
            #  * from the slave, if more than a FIFO full yet to receive
            #  * read a FIFO full.
            #  */
            BytesToRead = self.RecvByteCount - BytesInFifo - 1
            if BytesToRead > self.IIC_RX_FIFO_DEPTH:
                BytesToRead = self.IIC_RX_FIFO_DEPTH

            # /*
            #  * Read in data from the FIFO.
            #  */
            for LoopCnt in range (0, BytesToRead):
                self.readRecvByte()




    def setSendHandler(self, CallBackRef, FuncPtr):
        assert self.IsReady == self.XIL_COMPONENT_IS_READY
        assert CallBackRef is not None
        assert FuncPtr is not None

        self.SendHandler = FuncPtr
        self.SendCallBackRef = CallBackRef

    def setRecvHandler(self, CallBackRef, FuncPtr):
        assert self.IsReady == self.XIL_COMPONENT_IS_READY
        assert CallBackRef is not None
        assert FuncPtr is not None

        self.RecvHandler = FuncPtr
        self.RecvCallBackRef = CallBackRef


    def XIIC_MASTER_INCLUDE(self):
        self.recvMasterFuncPtr = self.RecvMasterData
        self.sendMasterFuncPtr = self.SendMasterData

    def masterSend(self, TxMsg):
        self.intrGlobalDisable()

        # /*
        #  * Ensure that the master processing has been included such that events
        #  * will be properly handled.
        #  */
        self.XIIC_MASTER_INCLUDE()
        self.isDynamic = False

        # /*
        #  * If the bus+ is busy, then exit the critical region and wait for the
        #  * bus to not be busy, the function enables the bus not busy interrupt.
        #  */
        if (self.IsBusBusy()):
            self.intrGlobalEnable()
            return self.XST_IIC_BUS_BUSY

        # /*
        #  * If it is already a master on the bus (repeated start), the direction
        #  * was set to Tx which is throttling bus. The control register needs to
        #  * be set before putting data into the FIFO.
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        if (CntlReg & self.XIIC_CR_MSMS_MASK):
            CntlReg &= ~self.XIIC_CR_NO_ACK_MASK
            CntlReg |= (self.XIIC_CR_DIR_IS_TX_MASK | self.XIIC_CR_REPEATED_START_MASK)

            self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)
            self.Stats.RepeatedStarts+=1

        # /*
        #  * Save message state.
        #  */
        self.SendByteCount = len(TxMsg)
        self.SendBuffer = TxMsg
        self.SendBufferIndex = 0
        self.RecvBuffer = None

        # /*
        #  * Put the address into the FIFO to be sent and indicate that the
        #  * operation to be performed on the bus is a write operation,
        #  * a general call address is handled the same as a 7 bit address even
        #  * if 10 bit address is selected.
        #  * Set the transmit address state to indicate the address has been sent.
        #  */
        if ((self.Options & self.XII_SEND_10_BIT_OPTION) and (self.AddrOfSlave != 0)):
            raise NotImplemented
            # send10BitAddrByte1(InstancePtr->AddrOfSlave,
            #              XIIC_WRITE_OPERATION)
            # send10BitAddrByte2(InstancePtr->AddrOfSlave)
        else:
            self.send7BitAddr(self.AddrOfSlave, self.XIIC_WRITE_OPERATION)

        # /*
        #  * Set the transmit address state to indicate the address has been sent
        #  * for communication with event driven processing.
        #  */
        self.TxAddrMode = self.XIIC_TX_ADDR_SENT

        # /*
        #  * Fill remaining available FIFO with message data.
        #  */
        if (self.SendByteCount > 1):
            self.transmitFifoFill(self.XIIC_MASTER_ROLE)

        # /*
        #  * After filling fifo, if data yet to send > 1, enable Tx � empty
        #  * interrupt.
        #  */
        if (self.SendByteCount > 1):
            self.clearEnableIntr(self.XIIC_INTR_TX_HALF_MASK)

        # /*
        #  * Clear any pending Tx empty, Tx Error and then enable them.
        #  */
        self.clearEnableIntr(self.XIIC_INTR_TX_ERROR_MASK | self.XIIC_INTR_TX_EMPTY_MASK)

        # /*
        #  * When repeated start not used, MSMS must be set after putting data
        #  * into transmit FIFO, start the transmitter.
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        if ((CntlReg & self.XIIC_CR_MSMS_MASK) == 0):
            CntlReg &= ~self.XIIC_CR_NO_ACK_MASK
            CntlReg |= self.XIIC_CR_MSMS_MASK | self.XIIC_CR_DIR_IS_TX_MASK
            self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)

        self.intrGlobalEnable()

        return self.XST_SUCCESS

    def masterRecv(self, ByteCount):

        # /*
        #  * If the slave address is zero (general call) the master can't perform
        #  * receive operations, indicate an error.
        #  */
        if (self.AddrOfSlave == 0):
            return self.XST_IIC_GENERAL_CALL_ADDRESS

        self.intrGlobalDisable()

        # /*
        #  * Ensure that the master processing has been included such that events
        #  * will be properly handled.
        #  */
        self.XIIC_MASTER_INCLUDE()
        self.isDynamic = False

        # /*
        #  * If the bus is busy, then exit the critical region and wait for the
        #  * bus to not be busy, the function enables the bus not busy interrupt.
        #  */
        if (self.IsBusBusy()):
            self.intrGlobalEnable()
            return self.XST_IIC_BUS_BUSY

        # /*
        #  * Save message state.
        #  */
        self.RecvByteCount = ByteCount
        self.RecvBuffer = bytearray(ByteCount)
        self.RecvBufferIndex = 0
        self.SendBuffer = None

        # /*
        #  * Clear and enable Rx full interrupt if using 7 bit, If 10 bit, wait
        #  * until last address byte sent incase arbitration gets lost while
        #  * sending out address.
        #  */
        if ((self.Options & self.XII_SEND_10_BIT_OPTION) == 0):
            self.clearEnableIntr(self.XIIC_INTR_RX_FULL_MASK)

        # /*
        #  * If already a master on the bus, the direction was set by Rx Interrupt
        #  * routine to Tx which is throttling bus because during Rxing, Tx reg is
        #  * empty = throttle. CR needs setting before putting data or the address
        #  * written will go out as Tx instead of receive. Start Master Rx by
        #  * setting CR Bits MSMS to Master and msg direction.
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)

        if (CntlReg & self.XIIC_CR_MSMS_MASK):
            CntlReg |= self.XIIC_CR_REPEATED_START_MASK
            self.setControlRegister(CntlReg, ByteCount)

            self.Stats.RepeatedStarts+=1
            self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)

        # /*
        #  * Set receive FIFO occupancy depth which must be done prior to writing
        #  * the address in the FIFO because the transmitter will immediatedly
        #  * start when in repeated start mode followed by the receiver such that
        #  * the number of  bytes to receive should be set 1st.
        #  */
        if ByteCount == 1:
            Temp = 0
        else:
            if ByteCount <= self.IIC_RX_FIFO_DEPTH:
                Temp = ByteCount - 2
            else:
                Temp = self.IIC_RX_FIFO_DEPTH - 1

        self.writeReg(self.XIIC_RFD_REG_OFFSET, Temp)

        if (self.Options & self.XII_SEND_10_BIT_OPTION):
            # /*
            #  * Send the 1st and 2nd byte of the 10 bit address of a write
            #  * operation, write because it's a 10 bit address.
            #  */
            self.send10BitAddrByte1(self.AddrOfSlave, self.XIIC_WRITE_OPERATION)
            self.send10BitAddrByte2(self.AddrOfSlave)

            # /*
            #  * Set flag to indicate the next byte of the address needs to be
            #  * send, clear and enable Tx empty interrupt.
            #  */
            self.TxAddrMode = self.XIIC_TX_ADDR_MSTR_RECV_MASK
            self.clearEnableIntr(self.XIIC_INTR_TX_EMPTY_MASK)
        else:
            # /*
            #  * 7 bit slave address, send the address for a read operation
            #  * and set the state to indicate the address has been sent.
            #  */
            self.send7BitAddr(self.AddrOfSlave, self.XIIC_READ_OPERATION)
            self.TxAddrMode = self.XIIC_TX_ADDR_SENT

        # /*
        #  * Tx error is enabled incase the address (7 or 10) has no device to
        #  * answer with Ack. When only one byte of data, must set NO ACK before
        #  * address goes out therefore Tx error must not be enabled as it will
        #  * go off immediately and the Rx full interrupt will be checked.
        #  * If full, then the one byte was received and the Tx error will be
        #  * disabled without sending an error callback msg.
        #  */
        self.clearEnableIntr(self.XIIC_INTR_TX_ERROR_MASK)

        # /*
        #  * When repeated start not used, MSMS gets set after putting data
        #  * in Tx reg. Start Master Rx by setting CR Bits MSMS to Master and
        #  * msg direction.
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        if (CntlReg & self.XIIC_CR_MSMS_MASK) == 0:
            CntlReg |= self.XIIC_CR_MSMS_MASK
            self.setControlRegister(CntlReg, ByteCount)
            self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)

        self.intrGlobalEnable()
        return self.XST_SUCCESS

    def flushTxFifo(self):
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg | self.XIIC_CR_TX_FIFO_RESET_MASK)
        self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)

    def TxErrorHandler(self):
        # /*
        #  * When Sending as a slave, Tx error signals end of msg. Not Addressed
        #  * As Slave will handle the callbacks. this is used to only flush
        #  * the Tx fifo. The addressed as slave bit is gone as soon as the bus
        #  * has been released such that the buffer pointers are used to determine
        #  * the direction of transfer (send or receive).
        #  */

        if self.RecvBuffer is None:
            # /*
            #  * Master Receiver finished reading message. Flush Tx fifo to
            #  * remove an 0xFF that was written to prevent bus throttling,
            #  * and disable all transmit and receive interrupts.
            #  */
            self.flushTxFifo()
            self.disableIntr(self.XIIC_TX_RX_INTERRUPTS)

            # /*
            #  * If operating in Master mode, call status handler to indicate
            #  * NOACK occured.
            #  */
            IntrStatus = self.readIisr()
            if ((IntrStatus & self.XIIC_INTR_AAS_MASK) == 0):
                print("NOACK")
                #InstancePtr->StatusHandler(InstancePtr->
                #   StatusCallBackRef,
                #   XII_SLAVE_NO_ACK_EVENT)
            else:
                # /* Decrement the Tx Error since Tx Error interrupt
                #  * implies transmit complete while sending as Slave
                #  */
                self.Stats.TxErrors-=1
            return

        # /*
        #  * Data in the receive register from either master or slave receive
        #  * When:slave, indicates master sent last byte, message completed.
        #  * When:master, indicates a master Receive with one byte received. When
        #  * a byte is in Rx reg then the Tx error indicates the Rx data was
        #  * recovered normally Tx errors are not enabled such that this should
        #  * not occur.
        #  */
        IntrStatus = self.readIisr()
        if (IntrStatus & self.XIIC_INTR_RX_FULL_MASK):
            # /* Rx Reg/FIFO has data,  Disable Tx error interrupts */

            self.disableIntr(self.XIIC_INTR_TX_ERROR_MASK)
            return


        self.flushTxFifo()

        # /*
        #  * Disable and clear Tx empty, � empty, Rx Full or Tx error interrupts.
        #  */
        self.disableIntr(self.XIIC_TX_RX_INTERRUPTS)
        self.clearIntr(self.XIIC_TX_RX_INTERRUPTS)

        # /* Clear MSMS as on Tx error when Rxing, the bus will be
        #  * stopped but MSMS bit is still set. Reset to proper state
        #  */
        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        CntlReg &= ~self.XIIC_CR_MSMS_MASK
        self.writeReg(self.XIIC_CR_REG_OFFSET, CntlReg)


        # /*
        #  * Set FIFO occupancy depth = 1 so that the first byte will throttle
        #  * next recieve msg.
        #  */
        self.writeReg(self.XIIC_RFD_REG_OFFSET, 0)

        # /*
        #  * Call the event callback.
        #  */
        # self.StatusHandler(self.StatusCallBackRef, self.XII_SLAVE_NO_ACK_EVENT)


    async def wait_for_isr_shutoff(self):
        await self.shutoffInterruptHandler.wait()

    async def wait_for_interrupt(self):
        await self.interrupt.wait()

    async def interrupt_handler(self):

        while True:

            self.async_task_interrupt = asyncio.ensure_future(self._partial_region.interrupt.wait())
            self.async_task_shutoff_isr = asyncio.ensure_future(self.shutoffInterruptHandler.wait())

            done, pending = await asyncio.wait([self.async_task_interrupt, self.async_task_shutoff_isr], return_when=asyncio.FIRST_COMPLETED)

            #await self.async_task_interrupt
            #await self.interrupt.wait()


            if self.async_task_shutoff_isr in done:
                return


            # /*
            #  * Get the interrupt Status.
            #  */
            IntrPending = self.readIisr()
            IntrEnable = self.readIier()
            IntrStatus = IntrPending & IntrEnable

            # /*
            #  * Do not processes a devices interrupts if the device has no
            #  * interrupts pending or the global interrupts have been disabled.
            #  */
            if ((IntrStatus == 0) or (self.isIntrGlobalEnabled() == False)):
                continue

            # /*
            #  * Update interrupt stats and get the contents of the status register.
            #  */
            self.Stats.IicInterrupts+=1
            Status = self.readReg(self.XIIC_SR_REG_OFFSET)

            # /*
            #  * Service requesting interrupt.
            #  */
            if (IntrStatus & self.XIIC_INTR_ARB_LOST_MASK):
                # /* Bus Arbritration Lost */

                self.Stats.ArbitrationLost+=1
                raise NotImplementedError
                #arbLostFuncPtr(IicPtr)

                Clear = self.XIIC_INTR_ARB_LOST_MASK
            elif (IntrStatus & self.XIIC_INTR_TX_ERROR_MASK):
                # /* Transmit errors (no acknowledge) received */
                self.Stats.TxErrors+=1

                #raise NotImplementedError
                self.TxErrorHandler()
                Clear = self.XIIC_INTR_TX_ERROR_MASK

            elif (IntrStatus & self.XIIC_INTR_NAAS_MASK):
                # /* Not Addressed As Slave */

                raise NotImplementedError
                # self.notAddrAsSlaveFuncPtr()
                Clear = self.XIIC_INTR_NAAS_MASK
            elif (IntrStatus & self.XIIC_INTR_RX_FULL_MASK):
                # /* Receive register/FIFO is full */
                self.Stats.RecvInterrupts+=1


                if (Status & self.XIIC_SR_ADDR_AS_SLAVE_MASK):
                    raise NotImplementedError
                else:
                    self.recvMasterFuncPtr()

                Clear = self.XIIC_INTR_RX_FULL_MASK
            elif (IntrStatus & self.XIIC_INTR_AAS_MASK):
                # /* Addressed As Slave */

                raise NotImplementedError

                # addrAsSlaveFuncPtr(IicPtr)
                # Clear = XIIC_INTR_AAS_MASK
            elif (IntrStatus & self.XIIC_INTR_BNB_MASK):
                # /* IIC bus has transitioned to not busy */

                # /* Check if send callback needs to run */
                if (self.BNBOnly):
                    self.busNotBusyFuncPtr()
                    self.BNBOnly = False
                else:
                    self.SendHandler(self.SendCallBackRef, 0)

                Clear = IIC.XIIC_INTR_BNB_MASK

                # /* The bus is not busy, disable BusNotBusy interrupt */
                self.disableIntr(IIC.XIIC_INTR_BNB_MASK)

            elif ((IntrStatus & self.XIIC_INTR_TX_EMPTY_MASK) or (IntrStatus & self.XIIC_INTR_TX_HALF_MASK)):
                # /* Transmit register/FIFO is empty or ? empty */
                self.Stats.SendInterrupts+=1

                #  raise NotImplementedError
                if (Status & self.XIIC_SR_ADDR_AS_SLAVE_MASK):
                    raise NotImplementedError
                    #self.sendSlaveFuncPtr()
                else:
                    self.sendMasterFuncPtr()

                IntrStatus = self.readIisr()
                Clear = IntrStatus & (IIC.XIIC_INTR_TX_EMPTY_MASK | IIC.XIIC_INTR_TX_HALF_MASK)
                
            # /*
            #  * Clear Interrupts.
            #  */
            self.writeIisr(Clear)

    def start(self):
        assert self.IsReady == self.XIL_COMPONENT_IS_READY

        # /*
        #  * Mask off all interrupts, each is enabled when needed.
        #  */
        self.writeIier(0)

        # /*
        #  * Clear all interrupts by reading and rewriting exact value back.
        #  * Only those bits set will get written as 1 (writing 1 clears intr).
        #  */
        self.clearIntr(0xFFFFFFFF)

        # /*
        #  * Enable the device.
        #  */
        self.writeReg(self.XIIC_CR_REG_OFFSET, self.XIIC_CR_ENABLE_DEVICE_MASK)

        # /*
        #  * Set Rx FIFO Occupancy depth to throttle at
        #  * first byte(after reset = 0).
        #  */
        self.writeReg(self.XIIC_RFD_REG_OFFSET, 0)

        # /*
        #  * Clear and enable the interrupts needed.
        #  */
        self.clearEnableIntr(self.XIIC_INTR_AAS_MASK | self.XIIC_INTR_ARB_LOST_MASK)

        self.isStarted = self.XIL_COMPONENT_IS_STARTED
        self.isDynamic = False

        # /*
        #  * Enable the Global interrupt enable.
        #  */
        self.intrGlobalEnable()

        return self.XST_SUCCESS

    def stop(self):
        # /*
        #  * Disable all interrupts globally.
        #  */
        self.intrGlobalDisable()

        CntlReg = self.readReg(self.XIIC_CR_REG_OFFSET)
        Status = self.readReg(self.XIIC_SR_REG_OFFSET)

        if ((CntlReg & self.XIIC_CR_MSMS_MASK) or (Status & self.XIIC_SR_ADDR_AS_SLAVE_MASK)):
            # /*
            #  * When this device is using the bus
            #  * - re-enable interrupts to finish current messaging
            #  * - return bus busy
            #  */
            self.intrGlobalEnable()
            return self.XST_IIC_BUS_BUSY
        self.isStarted = 0
        return self.XST_SUCCESS

    def sendCompleted(self, CallBackRef, bytesRemaining):
        assert bytesRemaining == 0

        # Trigger event indicating that send has completed
        self.blockingSendEvent.set()

        # Shut down the interrupt handler
        self.shutoffInterruptHandler.set()

    def recvCompleted(self, CallBackRef, bytesRemaining):
        assert bytesRemaining == 0

        # Trigger event indicating that recieve has completed
        self.blockingRecvEvent.set()

        # Shut down the interrupt handler
        self.shutoffInterruptHandler.set()

    def masterSend_blocking(self, TxMsg):
        refCnt = self.getCallBackRefCount()
        self.setSendHandler(refCnt, self.sendCompleted)

        self.shutoffInterruptHandler.clear()

        # Send Message (non-blocking)
        r = self.masterSend(TxMsg)
        assert r == self.XST_SUCCESS

        self.blockingSendEvent = asyncio.Event()

        # Create task to wait for send message callback
        self.async_task_send = asyncio.ensure_future(self.blockingSendEvent.wait())

        # Create task for the interrupt handler
        self.async_task_isr = asyncio.ensure_future(self.interrupt_handler())

        # Launch the tasks and wait for them to complete
        asyncio.get_event_loop().run_until_complete(asyncio.gather(self.async_task_send, self.async_task_isr))

    def masterRecv_blocking(self, byteCount=1):
        refCnt = self.getCallBackRefCount()
        self.setRecvHandler(refCnt, self.recvCompleted)

        self.shutoffInterruptHandler.clear()

        # Recieve Message (non-blocking)
        r = self.masterRecv(byteCount)
        assert r == self.XST_SUCCESS

        self.blockingRecvEvent = asyncio.Event()

        # Create task to wait for recieve message callback
        self.async_task_recv = asyncio.ensure_future(self.blockingRecvEvent.wait())

        # Create task for the interrupt handler
        self.async_task_isr = asyncio.ensure_future(self.interrupt_handler())

        # Launch the tasks and wait for them to complete
        asyncio.get_event_loop().run_until_complete(asyncio.gather(self.async_task_recv, self.async_task_isr))


    # Nice wrappers for MasterSend_blocking and MasterRecv_blocking
    def write(self, reg, val):
        if not self.isStarted:
            self.start()
        self.masterSend_blocking([reg, val])

    def read(self, reg, byteCount=1):
        if not self.isStarted:
            self.start()
        self.masterSend_blocking([reg])

        r = self.stop()
        assert r == self.XST_SUCCESS

        self.start()

        self.masterRecv_blocking(byteCount)

        r = self.stop()
        assert r == self.XST_SUCCESS

        return self.RecvBuffer
