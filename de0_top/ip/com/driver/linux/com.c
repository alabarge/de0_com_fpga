/*-----------------------------------------------------------------------------

   1  ABSTRACT

   1.1 Module Type

      Fast Serial COM I/O Driver

   1.2 Functional Description

      The COM I/O Interface routines are contained in this module.

      Steps for adding this driver to the main application :

         1. Call com_init() from main
         2. Call com_final() in user_control_c() exit
         3. Call com_revision() in cp_msg() version request
         4. Adjust the ADC sample rates per interface speed
         5. Call com_pipe() from daq_hal_run() and daq_msg()
         6. Change baudrate in fw_cfg.h, CFG_BAUD_RATE
         7. Set ADC_POOL_CNT for interface speed

   1.3 Specification/Design Reference

      See fw_cfg.h under the share directory.

   1.4 Module Test Specification Reference

      None

   1.5 Compilation Information

      See fw_cfg.h under the share directory.

   1.6 Notes

      NONE

   2  CONTENTS

      1 ABSTRACT
        1.1 Module Type
        1.2 Functional Description
        1.3 Specification/Design Reference
        1.4 Module Test Specification Reference
        1.5 Compilation Information
        1.6 Notes

      2 CONTENTS

      3 VOCABULARY

      4 EXTERNAL RESOURCES
        4.1  Include Files
        4.2  External Data Structures
        4.3  External Function Prototypes

      5 LOCAL CONSTANTS AND MACROS

      6 MODULE DATA STRUCTURES
        6.1  Local Function Prototypes
        6.2  Local Data Structures

      7 MODULE CODE
         7.1   com_init()
         7.2   com_thread()
         7.3   com_intack()
         7.4   com_tx()
         7.5   com_cmio()
         7.6   com_msgtx()
         7.7   com_pipe()
         7.8   com_version()
         7.9   com_final()

-----------------------------------------------------------------------------*/

// 3 VOCABULARY

// 4 EXTERNAL RESOURCES

// 4.1  Include Files

#include "main.h"

// 4.2   External Data Structures

// 4.3 External Function Prototypes

// 5 LOCAL CONSTANTS AND MACROS

// 6 MODULE DATA STRUCTURES

// 6.1  Local Function Prototypes

   static   void *com_thread(void *data);

// 6.2  Local Data Structures

   static   uint8_t        cm_port = CM_PORT_NONE;
   static   int32_t        fd;
   static   pthread_t      thread_id;

   static   com_txq_t     txq;
   static   uint8_t        tx_head;
   static   uint8_t        rx_tail;

   static   struct uio_info_t *p;

   static   volatile pcom_regs_t   regs;

// 7 MODULE CODE

// ===========================================================================

// 7.1

uint32_t com_init(uint32_t baudrate, uint8_t port) {

/* 7.1.1   Functional Description

   The COM Interface is initialized in this routine.

   7.1.2   Parameters:

   baudrate Serial Baud Rate
   port     COM Port

   7.1.3   Return Values:

   result   CFG_ERROR_OK

-----------------------------------------------------------------------------
*/

// 7.1.4   Data Structures

   uint32_t    result = CFG_ERROR_OK;
   int32_t     int_info;
   uint32_t    j;
   uint8_t     uio_found = 0;
   char        uio_name[16] = {0};

   com_ctl_reg_t ctl;

// 7.1.5   Code

   // Search for all Available UIO Devices
   p = uio_find_devices(-1);
   if (!p) {
      printf("com_init() Error : No UIO Devices Available\n");
      return CFG_ERROR_COM;
   }

   // Cycle over UIO devices
   while (p) {
      uio_get_all_info(p);
      if  (strncmp(p->name, COM_DEV_NAME, strlen(COM_DEV_NAME)) == 0) {
         uio_found = 1;
         break;
      }
      p = p->next;
   }

   // Open UIO Device
   if (uio_found) {
      sprintf(uio_name,"/dev/uio%d",p->uio_num);
      fd = open(uio_name, O_RDWR);
      if (fd < 0) {
         printf("com_init() Error : UIO Device did not Open, %s\n", uio_name);
         return CFG_ERROR_COM;
      }
   }
   else {
      printf("com_init() Error : UIO Device is not present\n");
      return CFG_ERROR_COM;
   }

   // Map the hardware region
   regs = (pcom_regs_t)mmap(NULL, p->maps[0].size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   if (regs == MAP_FAILED) {
      printf("com_init() Error : Mmap failed to acquire region, %d\n", p->maps[0].size);
      close(fd);
      return CFG_ERROR_COM;
   }

   // Acknowledge UIO Interrupt
   int_info = 1;
   if (write(fd, &int_info, sizeof(int_info)) < 0) {
      printf("com_init() Error : Cannot acknowledge UIO device interrupt: %s\n",
          strerror(errno));
      return CFG_ERROR_COM;
   }

   // Reset Control
   ctl.i = 0;
   regs->ctl = ctl.i;
   usleep(10);

   // Enable the hardware
   ctl.b.enable = 1;
   regs->ctl = ctl.i;
   usleep(10);

   // Init Control register
   ctl.b.tx_head     = 0;
   ctl.b.rx_tail     = 0;
   ctl.b.pipe_int    = 0;
   ctl.b.dma_req     = 0;
   ctl.b.rx_int      = 0;
   ctl.b.tx_int      = 0;
   ctl.b.pipe_run    = 0;
   ctl.b.ftdi_run    = 0;
   regs->ctl         = ctl.i;

   // Init Circular Address
   regs->addr_beg    = 0;
   regs->addr_end    = 0;

   // Set Baudrate
   regs->ticks       = (uint16_t)((100E6 / baudrate) + 0.5);

   // Enable the state machines
   ctl.b.pipe_run = 0;
   ctl.b.ftdi_run = 1;
   regs->ctl      = ctl.i;

   // Clear the hardware TX Buffer
   for (j=0;j<(COM_TX_BUFFER_LEN>>2);j++) {
      regs->tx_buf[j] = 0;
   }

   // Clear the hardware RX Buffer
   for (j=0;j<(COM_RX_BUFFER_LEN>>2);j++) {
      regs->rx_buf[j] = 0;
   }

   // Initialize the TX Queue
   memset(&txq, 0, sizeof(com_txq_t));
   for (j=0;j<COM_TX_QUE;j++) {
      txq.buf[j] = NULL;
   }
   pthread_mutex_init(&txq.mutex, NULL);
   txq.head  = 0;
   txq.tail  = 0;
   txq.slots = COM_TX_QUE;
   tx_head   = 0;
   rx_tail   = 0;

   // Clear ALL Pending Interrupts
   com_intack(COM_INT_ALL);

   // Register the I/O Interface callback for CM
   cm_ioreg(com_cmio, port, CM_MEDIA_COM);

   // Update CM Port
   cm_port = port;

   // Enable the RX & TX Interrupts
   ctl.b.pipe_int = 0;
   ctl.b.rx_int   = 1;
   ctl.b.tx_int   = 1;
   regs->ctl      = ctl.i;

   // Start the Interrupt Polling Thread
   if (pthread_create(&thread_id, NULL, com_thread, NULL)) {
      result = CFG_ERROR_COM;
   }

   // Print Hardware Version to Serial Port
   if (gc.trace & CFG_TRACE_ID) {
      printf("%-10s (%s) base:rev %08lX:%d\n", uio_name, p->name, p->maps[0].addr, regs->version);
      printf("%-10s (%s) rate:    %d.%d Mbps\n", uio_name, p->name, baudrate / 1000000, baudrate % 1000000);
      printf("%-10s (%s) port:    %d\n", uio_name, p->name, cm_port);
   }

   return result;

}  // end com_init()


// ===========================================================================

// 7.2

static void *com_thread(void *data) {

/* 7.2.1   Functional Description

   This thread will service the COM Interrupt from the UIO framework.

   7.2.2   Parameters:

   data     Thread parameters

   7.2.3   Return Values:

   return   Thread exit status

-----------------------------------------------------------------------------
*/

// 7.2.4   Data Structures

   uint32_t    ret_code, i;
   int32_t     int_info, ret;
   uint8_t     slotid;
   pcmq_t      slot;
   pcm_msg_t   msg;

   com_int_reg_t irq;
   com_ctl_reg_t ctl;
   com_sta_reg_t sta;

   struct pollfd fds = {
      .fd     = fd,
      .events = POLLIN,
   };

// 7.2.5   Code

   if (gc.trace & CFG_TRACE_ID) {
      printf("com_thread() started, data:tid %08X:%lu\n", (uint32_t)data, syscall(SYS_gettid));
   }

   // acknowledge UIO interrupt
   int_info = 1;
   write(fd, &int_info, sizeof(int_info));

   // Interrupt Polling loop
   while (1) {
      // Prevent arbitrary cancellation point
      pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
      pthread_testcancel();
      pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
      // polling with 1000 mS timeout, allow for thread cancellation
      ret = poll(&fds, 1, 1000);
      if (ret == 0) {
         // timeout
      }
      // handle com interrupts
      else if (ret >= 1) {
         // NOTE: fd must be read to clear UIO interrupt
         read(fd, &int_info, sizeof(int_info));
         // acknowledge UIO interrupt
         int_info = 1;
         write(fd, &int_info, sizeof(int_info));
         // report interrupt request
         if (gc.trace & CFG_TRACE_IRQ) {
            printf("com_thread() irq = %02X\n", regs->irq);
         }
         // process all interrupt signals, receive has priority
         while ((irq.i = regs->irq) != 0) {
            // Lock the TX mutex
            pthread_mutex_lock(&txq.mutex);
            //
            // RX INTERRUPT
            //
            if (irq.b.rx == 1) {
               // show activity
               gpio_set_val(GPIO_LED_COM, GPIO_LED_ON);
               // allocate slot from cmq
               slot = cm_alloc();
               if (slot != NULL) {
                  sta.i = regs->sta;
                  if (sta.b.rx_head != rx_tail) {
                     msg = (pcm_msg_t)slot->buf;
                     // preserve q slot id
                     slotid = msg->h.slot;
                     // uint32_t boundary, copy multiple of 32-bits
                     // account for rx_tail buffer position
                     // always read CM header + parms in order
                     // to determine message length
                     for (i=0;i<sizeof(cm_msg_t) >> 2;i++) {
                        slot->buf[i] = regs->rx_buf[i + (rx_tail * COM_MSGLEN_UINT32)];
                     }
                     slot->msglen = msg->h.msglen;
                     // read rest of CM message body, uint32_t per cycle
                     if (slot->msglen > sizeof(cm_msg_t) && (slot->msglen <= COM_MSGLEN_UINT8)) {
                        for (i=0;i<(slot->msglen + 3 - sizeof(cm_msg_t)) >> 2;i++) {
                           slot->buf[i + (sizeof(cm_msg_t) >> 2)] =
                                 regs->rx_buf[i + (rx_tail * COM_MSGLEN_UINT32) + (sizeof(cm_msg_t) >> 2)];
                        }
                     }
                     // restore q slot id
                     msg->h.slot = slotid;
                  }
                  // report message content
                  if ((gc.trace & CFG_TRACE_UART) && (slot != NULL)) {
                     printf("com_thread() msglen = %d\n", msg->h.msglen);
                     dump((uint8_t *)slot->buf, slot->msglen, 0, 0);
                  }
                  // advance the h/w tail pointer
                  if (++rx_tail == COM_RX_SLOTS) rx_tail = 0;
                  ctl.i         = regs->ctl;
                  ctl.b.rx_tail = rx_tail;
                  regs->ctl     = ctl.i;
               }
            }
            // Unlock the TX mutex
            pthread_mutex_unlock(&txq.mutex);
            //
            // INTERRUPT PROCESSING OUTSIDE OF MUTEX
            //
            // queue the message
            if (irq.b.rx == 1) {
               // clear the interrupt if head = tail
               sta.i = regs->sta;
               if (sta.b.rx_head == rx_tail) com_intack(FIFO_INT_RX);
               if (slot != NULL) {
                  cm_qmsg((pcm_msg_t)slot->buf);
                  slot = NULL;
               }
            }
            // check tx queue
            else if (irq.b.tx == 1) {
               // clear the interrupt
               com_intack(COM_INT_TX);
               com_msgtx();
            }
            // send interrupt indication
            else if (irq.b.pipe == 1) {
               // clear the interrupt
               com_intack(COM_INT_PIPE);
               cm_local(CM_ID_DAQ_SRV, DAQ_INT_IND, DAQ_INT_FLAG_PIPE, DAQ_OK);
            }
         }
      }
      else {
         fprintf(stderr, "com_thread() Error : poll() returned error: %d\n", ret);
         ret_code = COM_ERR_POLL;
         break;
      }
   }

   return (void *)ret_code;

} // end com_thread()


// ===========================================================================

// 7.3

void com_intack(uint8_t int_type) {

/* 7.3.1   Functional Description

   This routine will Acknowledge specific COM Interrupts, or ALL.

   7.3.2   Parameters:

   intType  COM_INT_RX:  COM Receive Interrupt
            COM_INT_TX:  COM Transmit Interrupt
            COM_INT_ALL: All COM Interrupts

   7.3.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.3.4   Data Structures

   com_int_reg_t irq;

// 7.3.5   Code

   irq.i = regs->irq;

   if (int_type & COM_INT_RX) {
      irq.b.rx   = 1;
   }

   if (int_type & COM_INT_TX) {
      irq.b.tx   = 1;
   }

   if (int_type & COM_INT_PIPE) {
      irq.b.pipe = 1;
   }

   if (int_type & COM_INT_ALL) {
      irq.b.rx   = 1;
      irq.b.tx   = 1;
      irq.b.pipe = 1;
   }

   regs->irq = irq.i;

} // end com_intack()


// ===========================================================================

// 7.4

void com_tx(pcm_msg_t msg) {

/* 7.4.1   Functional Description

   This routine will transmit the message. The txq_mutex is used to prevent
   corruption of status and control registers. This routine may be called
   by different threads.

   7.4.2   Parameters:

   pMsg     Message to send.

   7.4.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.4.4   Data Structures

   uint32_t    i;
   uint8_t     tail;
   uint32_t   *out = (uint32_t *)msg;

   com_ctl_reg_t ctl;
   com_sta_reg_t sta;

// 7.4.5   Code

   // Trace Entry
   if (gc.trace & CFG_TRACE_UART) {
      printf("com_tx() srvid:msgid:msglen:msg = %02X:%02X:%04X:%08X\n",
               msg->p.srvid, msg->p.msgid, msg->h.msglen, (uint32_t)msg);
      dump((uint8_t *)msg, msg->h.msglen, 0, 0);
   }

   // Lock the TX mutex
   pthread_mutex_lock(&txq.mutex);

   // local copies
   ctl.i = regs->ctl;
   sta.i = regs->sta;

   // h/w transmit tail
   tail = (sta.b.tx_tail + 1) & (COM_TX_SLOTS - 1);

   // Validate message, drop if NULL
   if (msg == NULL) {
      // advance the tx queue
      if (++txq.tail == txq.slots) txq.tail = 0;
   }
   // check for h/w slot availability?
   else if (tx_head != tail) {
      // copy message to hardware
      for (i=0;i<(msg->h.msglen + 3) >> 2;i++) {
         regs->tx_buf[i + (tx_head * COM_MSGLEN_UINT32)] = out[i];
      }
      // clear the msg pointer
      txq.buf[txq.tail] = NULL;
      // advance the tx queue
      if (++txq.tail == txq.slots) txq.tail = 0;
      // advance the h/w tx queue
      if (++tx_head == COM_TX_SLOTS) tx_head = 0;
      // transmit, read-modify-write control
      // NOTE: writing directly to regs->ctl.b.tx_head doesn't work
      ctl.b.tx_head  = tx_head;
      regs->ctl      = ctl.i;
      // show activity
      gpio_set_val(GPIO_LED_COM, GPIO_LED_ON);
      // release message
      cm_free(msg);
   }

   // Unlock the TX mutex
   pthread_mutex_unlock(&txq.mutex);

} // end com_tx()


// ===========================================================================

// 7.5

void com_cmio(uint8_t op_code, pcm_msg_t msg) {

/* 7.5.1   Functional Description

   OPCODES

   CM_IO_TX : The transmit queue index will be incremented,
   this causes the top of the queue to be transmitted.

   7.5.2   Parameters:

   msg     Message Pointer
   opCode  CM_IO_TX

   7.5.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.5.4   Data Structures

// 7.5.5   Code

   if (gc.trace & CFG_TRACE_UART) {
      printf("com_cmio() op_code:msg = %02X:%08X\n", op_code, (uint32_t)msg);
   }

   // place in transmit queue
   txq.buf[txq.head] = (uint32_t *)msg;
   if (++txq.head == txq.slots) txq.head = 0;

   // try to transmit message
   com_msgtx();

} // end com_cmio()


// ===========================================================================

// 7.6

void com_msgtx(void) {

/* 7.6.1   Functional Description

   This routine will check for outgoing messages and route them to the
   transmitter comTx().

   7.6.2   Parameters:

   NONE

   7.6.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.6.4   Data Structures

// 7.6.5   Code

   // Check for message in Queue
   if (txq.head != txq.tail) {
      com_tx((pcm_msg_t)txq.buf[txq.tail]);
   }

} // end com_msgtx()


// ===========================================================================

// 7.7

void com_pipe(uint32_t opcode, uint32_t addr_beg, uint32_t addr_end, uint32_t pktcnt) {

/* 7.7.1   Functional Description

   This routine will start the master read state machine and stream pipe messages
   through the COM. The txq.mutex is used to prevent corruption of status
   and control registers.

   7.7.2   Parameters:

   opcode     Operation Codes
   addr_beg   Begin address of COM in Memory, On Chip or SDRAM
   addr_end   Ending address of COM
   pktcnt     Specific Pipe Message Count

   7.7.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.7.4   Data Structures

   com_ctl_reg_t ctl;

// 7.7.5   Code

   if (gc.trace & CFG_TRACE_UART) {
      printf("com_pipe() opcode:addr_beg:addr_end:pktcnt = %02X:%08X:%08X:%08X\n",
            opcode, addr_beg, addr_end, pktcnt);
   }

   // Lock the TX mutex
   pthread_mutex_lock(&txq.mutex);

   ctl.i = regs->ctl;

   if (opcode & COM_OP_START) {
      // begin-end memory address of pipe message COM
      regs->addr_beg = addr_beg;
      regs->addr_end = addr_end;
      // pipe message (packet) count, zero for continuous
      regs->pktcnt   = pktcnt;
      // run request
      ctl.b.pipe_run = 1;
      ctl.b.pipe_int = 1;
      regs->ctl      = ctl.i;
   }
   else if (opcode & COM_OP_STOP) {
      // allow COM to finish pipe burst
      usleep(200);
      // stop request
      ctl.b.pipe_run = 0;
      ctl.b.pipe_int = 0;
      regs->ctl      = ctl.i;
   }

   // Unlock the TX mutex
   pthread_mutex_unlock(&txq.mutex);

} // end fifoPipe()


// ===========================================================================

// 7.8

uint32_t com_version(void) {

/* 7.8.1   Functional Description

   This routine will return the COM VERSION register value.

   7.8.2   Parameters:

   NONE

   7.8.3   Return Values:

   return   VERSION register

-----------------------------------------------------------------------------
*/

// 7.8.4   Data Structures

// 7.8.5   Code

   return regs->version;

} // end com_version()


// ===========================================================================

// 7.9

void com_final(void) {

/* 7.9.1   Functional Description

   This routine will clean-up any allocated resources.

   7.9.2   Parameters:

   NONE

   7.9.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.9.4   Data Structures

// 7.9.5   Code

   // Cancel Interrupt Thread
   pthread_cancel(thread_id);
   pthread_join(thread_id, NULL);

   // Disable Hardware
   regs->ctl = 0;

   // Release the memory map
   munmap((void*)regs, p->maps[0].size);

   // Close UIO
   close(fd);

} // end com_final()



