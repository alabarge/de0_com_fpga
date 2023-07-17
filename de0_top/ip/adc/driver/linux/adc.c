/*-----------------------------------------------------------------------------

   1  ABSTRACT

   1.1 Module Type

      LTC2308 Driver

   1.2 Functional Description

      The ADC Interface routines are contained in this module.

   1.3 Specification/Design Reference

      See fw_cfg.h under the BOOT/SHARE directory.

   1.4 Module Test Specification Reference

      None

   1.5 Compilation Information

      See fw_cfg.h under the BOOT/SHARE directory.

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
      7.1   adc_init()
      7.2   adc_thread()
      7.3   adc_intack()
      7.4   adc_run()
      7.5   adc_version()
      7.6   adc_final()

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

   static   void *adc_thread(void *data);

// 6.2  Local Data Structures

   static   int32_t        fd;
   static   pthread_t      thread_id;

   static   struct uio_info_t *p;

   static   volatile padc_regs_t   regs;

// 7 MODULE CODE

// ===========================================================================

// 7.1

uint32_t adc_init(void) {

/* 7.1.1   Functional Description

   This routine is responsible for initializing the driver hardware.

   7.1.2   Parameters:

   NONE

   7.1.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.1.4   Data Structures

   uint32_t    result = CFG_STATUS_OK;
   int32_t     int_info;
   uint8_t     uio_found = 0;
   char        uio_name[16] = {0};

   adc_ctl_reg_t ctl;

// 7.1.5   Code

   // Search for all Available UIO Devices
   p = uio_find_devices(-1);
   if (!p) {
      printf("adc_init() Error : No UIO Devices Available\n");
      return CFG_ERROR_ADC;
   }

   // Cycle over UIO devices
   while (p) {
      uio_get_all_info(p);
      if  (strncmp(p->name, ADC_DEV_NAME, strlen(FTDI_DEV_NAME)) == 0) {
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
         printf("adc_init() Error : UIO Device did not Open, %s\n", uio_name);
         return CFG_ERROR_ADC;
      }
   }
   else {
      printf("adc_init() Error : UIO Device is not present\n");
      return CFG_ERROR_ADC;
   }

   // Map the hardware region
   regs = (padc_regs_t)mmap(NULL, p->maps[0].size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   if (regs == MAP_FAILED) {
      printf("adc_init() Error : Mmap failed to acquire region, %d\n", p->maps[0].size);
      close(fd);
      return CFG_ERROR_ADC;
   }

   // Acknowledge UIO Interrupt
   int_info = 1;
   if (write(fd, &int_info, sizeof(int_info)) < 0) {
      printf("adc_init() Error : Cannot acknowledge UIO device interrupt: %s\n",
          strerror(errno));
      return CFG_ERROR_ADC;
   }

   // Reset Control
   ctl.i = 0;
   regs->ctl = ctl.i;
   usleep(10);

   // Init Control register
   ctl.b.ramp        = 0;
   ctl.b.run         = 0;
   ctl.b.pkt_int_en  = 0;
   ctl.b.done_int_en = 0;
   regs->ctl         = ctl.i;

   // Init Circular Address
   regs->addr_beg    = ADC_FIFO_BASE;
   regs->addr_end    = ADC_FIFO_BASE + ADC_FIFO_SPAN - 1;

   // Init Registers
   regs->pkt_cnt     = 0;
   regs->pool_cnt    = ADC_POOL_CNT;
   regs->adc_rate    = ADC_SAM_RATE;

   // Clear any Pending Interrupts
   adc_intack(ADC_INT_ALL);

   // Start the Interrupt Polling Thread
   if (pthread_create(&thread_id, NULL, adc_thread, NULL)) {
      result = CFG_ERROR_ADC;
   }

   // Print Hardware Version to Serial Port
   if (gc.trace & CFG_TRACE_ID) {
      printf("%-10s (%s) base:rev %08lX:%d\n", uio_name, p->name, p->maps[0].addr, regs->version);
   }

   return result;

}  // end adc_init()


// ===========================================================================

// 7.2

static void *adc_thread(void *data) {

/* 7.2.1   Functional Description

   This thread will service the ADC Interrupt from the UIO framework.

   7.2.2   Parameters:

   data     Thread parameters

   7.2.3   Return Values:

   return   Thread exit status

-----------------------------------------------------------------------------
*/

// 7.2.4   Data Structures

   uint32_t    ret_code;
   int32_t     int_info;
   int32_t     ret;

   adc_int_reg_t  irq;

   struct pollfd fds = {
      .fd     = fd,
      .events = POLLIN,
   };

// 7.2.5   Code

   if (gc.trace & CFG_TRACE_ID) {
      printf("adc_thread() started, data:tid %08X:%lu\n", (uint32_t)data, syscall(SYS_gettid));
   }

   // Interrupt Polling loop
   while (1) {

      // Acknowledge UIO Interrupt
      int_info = 1;
      write(fd, &int_info, sizeof(int_info));

      // Prevent arbitrary cancellation point
      pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
      pthread_testcancel();
      pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

      // polling with 1000 mS timeout
      ret = poll(&fds, 1, 1000);

      // timeout
      if (ret == 0) {
      }
      // handle packet and done interrupt
      else if (ret >= 1) {
         // NOTE: fd must be read to clear UIO interrupt
         read(fd, &int_info, sizeof(int_info));
         // local copy
         irq.i = regs->irq;
         // report interrupt request
         if (gc.trace & CFG_TRACE_IRQ) {
            printf("adc_thread() irq = %02X\n", irq.i);
         }
         //
         // PACKET READY INTERRUPT
         //
         if (irq.b.pkt == 1) {
            // clear the interrupt
            adc_intack(ADC_INT_PKT);
            // send interrupt indication
            cm_local(CM_ID_DAQ_SRV, DAQ_INT_IND, DAQ_INT_FLAG_PKT, DAQ_OK);
         }
         //
         // DONE INTERRUPT
         //
         if (irq.b.done == 1) {
            // clear the interrupt
            adc_intack(ADC_INT_DONE);
            // send interrupt indication
            cm_local(CM_ID_DAQ_SRV, DAQ_INT_IND, DAQ_INT_FLAG_DONE, DAQ_OK);
         }
      }
      else {
         printf("adc_thread() Error : poll() returned error: %d\n", ret);
         ret_code = FTDI_ERR_POLL;
         break;
      }
   }

   return (void *)ret_code;

} // end adc_thread()


// ===========================================================================

// 7.3

void adc_intack(uint8_t int_type) {

/* 7.3.1   Functional Description

   This routine will Acknowledge specific ADC Interrupts, or ALL.

   7.3.2   Parameters:

   intType  ADC_INT_RDY:   ADC Block Ready Interrupt
            ADC_INT_DONE:  ADC Done Interrupt
            ADC_INT_ALL:   All ADC Interrupts

   7.3.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.3.4   Data Structures

   adc_int_reg_t  irq;

// 7.3.5   Code

   irq.i = regs->irq;

   if (int_type & ADC_INT_DONE) {
      irq.b.done = 1;
   }

   if (int_type & ADC_INT_PKT) {
      irq.b.pkt = 1;
   }

   if (int_type & ADC_INT_ALL) {
      irq.b.done = 1;
      irq.b.pkt = 1;
   }

   regs->irq = irq.i;

} // end adc_intack()


// ===========================================================================

// 7.4

void adc_run(uint32_t flags, uint32_t packets) {

/* 7.4.1   Functional Description

   This routine will start or stop the ADC acquisition.

   7.4.2   Parameters:

   flags    Start/Stop flags
   blocks   Number of 1024-Byte packets to acquire

   7.4.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.4.4   Data Structures

   adc_ctl_reg_t  ctl;

// 7.4.5   Code

   ctl.i = regs->ctl;

   //
   // Start ADC Acquisition
   //
   if (flags & DAQ_CMD_RUN) {
      ctl.b.enable   = 1;
      regs->ctl      = ctl.i;
      usleep(10);
      // write begin & end addresses
      regs->addr_beg = ADC_FIFO_BASE;
      regs->addr_end = ADC_FIFO_BASE + ADC_FIFO_SPAN - 1;
      // packet count
      regs->pkt_cnt  = packets;
      // pool count, port type dependent (see adc_init() above)
      regs->pool_cnt = ADC_POOL_CNT;
      // set run parameters
      regs->adc_rate = ADC_SAM_RATE;
      ctl.b.run      = 1;
      regs->ctl      = ctl.i;
      // prevent packet interrupts when using head/tail in hardware
      // packet interrupts are used when explicitly sending packets
      // from the interface, interrupt will occur every ADC_POOL_CNT
      ctl.b.pkt_int_en  = 1;
      ctl.b.done_int_en = 1;
      ctl.b.ramp        = (flags & DAQ_CMD_RAMP) ? 1 : 0;
      regs->ctl         = ctl.i;
   }
   //
   // Stop ADC Acquisition
   //
   else {
      ctl.b.enable      = 1;
      ctl.b.run         = 0;
      ctl.b.pkt_int_en  = 0;
      ctl.b.done_int_en = 0;
      regs->ctl      = ctl.i;
   }

} // end adc_run()


// ===========================================================================

// 7.5

uint32_t adc_version(void) {

/* 7.5.1   Functional Description

   This routine will return the FTDI VERSION register value.

   7.5.2   Parameters:

   NONE

   7.5.3   Return Values:

   return   VERSION register

-----------------------------------------------------------------------------
*/

// 7.5.4   Data Structures

// 7.5.5   Code

   return regs->version;

} // end adc_version()


// ===========================================================================

// 7.6

void adc_final(void) {

/* 7.6.1   Functional Description

   This routine will clean-up any allocated resources.

   7.6.2   Parameters:

   NONE

   7.6.3   Return Values:

   NONE

-----------------------------------------------------------------------------
*/

// 7.6.4   Data Structures

// 7.6.5   Code

   // Cancel Interrupt Thread
   pthread_cancel(thread_id);
   pthread_join(thread_id, NULL);

   // Disable Hardware
   regs->ctl = 0;

   // Release the memory map
   munmap((void*)regs, p->maps[0].size);

   // Close UIO
   close(fd);

} // end adc_final()

