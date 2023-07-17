library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adc_ctl is
   port (
      -- Avalon Clock, Reset & Interrupt
      clk                  : in    std_logic;
      reset_n              : in    std_logic;
      int                  : out   std_logic_vector(1 downto 0);
      -- Avalon Memory-Mapped Write Master
      m1_write             : out   std_logic;
      m1_wr_address        : out   std_logic_vector(31 downto 0);
      m1_writedata         : out   std_logic_vector(31 downto 0);
      m1_wr_waitreq        : in    std_logic;
      m1_wr_burstcount     : out   std_logic_vector(8 downto 0);
      -- Control Registers
      adc_CONTROL          : in    std_logic_vector(31 downto 0);
      adc_ADR_BEG          : in    std_logic_vector(31 downto 0);
      adc_ADR_END          : in    std_logic_vector(31 downto 0);
      adc_PKT_CNT          : in    std_logic_vector(31 downto 0);
      adc_POOL_CNT         : in    std_logic_vector(7 downto 0);
      adc_ADC_RATE         : in    std_logic_vector(15 downto 0);
      adc_XFER_SIZE        : in    std_logic_vector(7 downto 0);
      adc_STATUS           : out   std_logic_vector(31 downto 0);
      -- Block RAM I/F
      cpu_DIN              : out   std_logic_vector(31 downto 0);
      cpu_DOUT             : in    std_logic_vector(31 downto 0);
      cpu_ADDR             : in    std_logic_vector(10 downto 0);
      cpu_WE               : in    std_logic;
      cpu_RE               : in    std_logic;
      -- Memory Head-Tail Pointers
      head_addr            : out   std_logic_vector(15 downto 0);
      tail_addr            : in    std_logic_vector(15 downto 0);
      -- Exported Signals
      cs                   : out   std_logic;
      din                  : out   std_logic;
      dout                 : in    std_logic;
      sclk                 : out   std_logic
   );
end adc_ctl;

architecture rtl of adc_ctl is

--
-- TYPES
--
type   ad_state_t is (IDLE, HEADER, SCLK_LO, SCLK_HI, STORE, CHECK);
type   wr_state_t is (IDLE, WAIT_SLOT, DELAY, WR_SLOT);

type  AD_SV_t is record
   state       : ad_state_t;
   pkt_cnt     : unsigned(31 downto 0);
   delay       : integer range 0 to 65535;
   run         : std_logic;
   run_r0      : std_logic;
   busy        : std_logic;
   done        : std_logic;
   in_ptr      : unsigned(7 downto 0);
   ch_sel      : std_logic_vector(31 downto 0);
   ch_cnt      : unsigned(1 downto 0);
   out_dat     : std_logic_vector(31 downto 0);
   ramp        : unsigned(15 downto 0);
   head        : unsigned(1 downto 0);
   in_we       : std_logic;
   seq_id      : unsigned(31 downto 0);
   bit_cnt     : integer range 0 to 32;
   hdr_cnt     : integer range 0 to 10;
   cs          : std_logic;
   din         : std_logic;
   sclk        : std_logic;
end record AD_SV_t;

type  WR_SV_t is record
   state       : wr_state_t;
   run         : std_logic;
   run_r0      : std_logic;
   addr        : unsigned(31 downto 0);
   wrd_cnt     : unsigned(8 downto 0);
   pool_cnt    : unsigned(7 downto 0);
   head_addr   : unsigned(15 downto 0);
   head_cnt    : unsigned(15 downto 0);
   xfer_cnt    : unsigned(15 downto 0);
   tail        : unsigned(1 downto 0);
   master      : std_logic;
   burstcnt    : std_logic_vector(8 downto 0);
   busy        : std_logic;
   pkt_rdy     : std_logic;
end record WR_SV_t;

--
-- CONSTANTS
--

--
-- NOTE: ALL RATES ARE BASED ON A 100MHZ FPGA CLOCK
--

-- Minimum ADC Rate, 200 KSPS
constant C_ADC_RATE_MIN       : std_logic_vector(15 downto 0) := X"001E";

-- AD State Vector Initialization
constant C_AD_SV_INIT : AD_SV_t := (
   state       => IDLE,
   pkt_cnt     => (others => '0'),
   delay       => 0,
   run         => '0',
   run_r0      => '0',
   busy        => '0',
   done        => '0',
   in_ptr      => (others => '0'),
   ch_sel      => (others => '0'),
   ch_cnt      => (others => '0'),
   out_dat     => (others => '0'),
   ramp        => (others => '0'),
   head        => (others => '0'),
   in_we       => '0',
   seq_id      => (others => '0'),
   bit_cnt     => 0,
   hdr_cnt     => 0,
   cs          => '0',
   din         => '0',
   sclk        => '0'
);

-- WR State Vector Initialization
constant C_WR_SV_INIT : WR_SV_t := (
   state       => IDLE,
   run         => '0',
   run_r0      => '0',
   addr        => (others => '0'),
   wrd_cnt     => (others => '0'),
   pool_cnt    => (others => '0'),
   head_addr   => (others => '0'),
   head_cnt    => (others => '0'),
   xfer_cnt    => (others => '0'),
   tail        => (others => '0'),
   master      => '0',
   burstcnt    => (others => '0'),
   busy        => '0',
   pkt_rdy     => '0'
);

--
-- SIGNAL DECLARATIONS
--

-- State Machine Data Types
signal ad               : AD_SV_t;
signal wr               : WR_SV_t;

-- 32-Bit Machine Status
signal adc_stat         : std_logic_vector(31  downto 0);
alias  xl_HEAD_ADDR     : std_logic_vector(15 downto 0) is adc_stat(15 downto 0);
alias  xl_TAIL          : std_logic_vector(3 downto 0) is adc_stat(19 downto 16);
alias  xl_HEAD          : std_logic_vector(3 downto 0) is adc_stat(23 downto 20);
alias  xl_UNUSED        : std_logic_vector(5 downto 0) is adc_stat(29 downto 24);
alias  xl_DMA_BUSY      : std_logic is adc_stat(30);
alias  xl_ADC_BUSY      : std_logic is adc_stat(31);

-- 32-Bit Control Register
alias  xl_CH_SEL        : std_logic_vector(3 downto 0) is adc_CONTROL(3 downto 0);
alias  xl_CH_ALL        : std_logic is adc_CONTROL(26);
alias  xl_RAMP          : std_logic is adc_CONTROL(27);
alias  xl_RUN           : std_logic is adc_CONTROL(28);
alias  xl_PKT_INT_EN    : std_logic is adc_CONTROL(29);
alias  xl_DONE_INT_EN   : std_logic is adc_CONTROL(30);
alias  xl_ENABLE        : std_logic is adc_CONTROL(31);

signal writedata        : std_logic_vector(31 downto 0);
signal wr_clk_en        : std_logic;
signal out_addr         : std_logic_vector(9 downto 0);
signal ad_dout          : std_logic;
signal stamp            : unsigned(31 downto 0);
signal adc_rate         : unsigned(15 downto 0);

--
-- MAIN CODE
--
begin

   --
   -- COMBINATORIAL OUTPUTS
   --
   int(0)               <= ad.done and xl_DONE_INT_EN;
   int(1)               <= wr.pkt_rdy and xl_PKT_INT_EN;

   sclk                 <= ad.sclk;
   cs                   <= not ad.cs;
   din                  <= ad.din;
   ad_dout              <= dout;

   adc_STATUS           <= adc_stat;

   -- divided between SCLK HI/LO
   adc_rate             <= unsigned('0' & adc_ADC_RATE(15 downto 1)) when adc_ADC_RATE >= C_ADC_RATE_MIN
                           else unsigned('0' & C_ADC_RATE_MIN(15 downto 1));

   -- Master Wite
   m1_wr_address        <= std_logic_vector(wr.addr);
   m1_writedata         <= writedata;
   m1_write             <= wr.master;
   m1_wr_burstcount     <= wr.burstcnt;

   -- CPU Reads FIFO directly
   cpu_DIN              <= writedata;

   -- Shared Packet Address, tail_addr not used here.
   -- head_addr increments based on adc_XFER_SIZE which
   -- is related to the com port pipe message transfer size.
   -- For example, the FIFO transfer size is 8K where the
   -- OPTO transfer size is 1K.
   head_addr            <= std_logic_vector(wr.head_addr);

   --
   --   1024 32-Bit Dual-Port BLOCK RAM
   --   ADC => ONCHIP => FTDI
   --   Circular Buffer with 4 1K Byte slots
   --
   ADC_4K_I : entity work.adc_4k
   port map (
      clock_a     => clk,
      enable_a    => '1',
      address_a   => std_logic_vector(ad.head) & std_logic_vector(ad.in_ptr),
      data_a      => ad.out_dat,
      wren_a      => ad.in_we,
      q_a         => open,
      clock_b     => clk,
      enable_b    => wr_clk_en,
      address_b   => out_addr,
      data_b      => X"00000000",
      wren_b      => '0',
      q_b         => writedata
   );
   out_addr       <= cpu_ADDR(9 downto 0) when cpu_RE = '1' else
                     std_logic_vector(wr.tail) & std_logic_vector(wr.wrd_cnt(7 downto 0));

   wr_clk_en      <= '1' when wr.state = WAIT_SLOT else
                     '1' when wr.state = DELAY else
                     '1' when wr.state = WR_SLOT and m1_wr_waitreq = '0' else
                     '1' when cpu_RE = '1' else '0';

   --
   -- ADC READ FSM
   --
   process(all) begin
      if (reset_n = '0' or xl_ENABLE = '0') then

         -- Init the State Vector
         ad             <= C_AD_SV_INIT;

         -- status is shared by master write FSM
         xl_ADC_BUSY    <= '0';
         xl_HEAD        <= (others => '0');
         xl_UNUSED      <= (others => '0');

      elsif (rising_edge(clk)) then

         -- edge-detect
         ad.run         <= xl_RUN;
         ad.run_r0      <= ad.run;

         -- Status
         xl_ADC_BUSY    <= ad.busy;
         xl_HEAD        <= "00" & std_logic_vector(ad.head);
         xl_UNUSED      <= (others => '0');

         case ad.state is
            when IDLE =>
               -- Look for Run Bit Rising-Edge
               if (ad.run_r0 = '0' and ad.run = '1') then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= (others => '0');
                  ad.head     <= (others => '0');
                  ad.pkt_cnt  <= (others => '0');
                  ad.seq_id   <= (others => '0');
                  ad.hdr_cnt  <= 0;
                  ad.ch_sel   <= '0' & xl_CH_SEL & "000" & X"00" & '0' & xl_CH_SEL & "000" & X"00";
                  ad.out_dat  <= (others => '0');
                  ad.ramp     <= (others => '0');
                  ad.ch_cnt   <= (others => '0');
                  ad.busy     <= '1';
                  ad.delay    <= to_integer(adc_rate);
                  ad.cs       <= '1';
               else
                  ad.state    <= IDLE;
                  ad.busy     <= '0';
                  ad.cs       <= '0';
                  ad.done     <= '0';
                  ad.sclk     <= '1';
               end if;

            --
            --  CM_PIPE Message Header
            --
            -- // CM PIPE DAQ MESSAGE DATA STRUCTURE
            -- // USED FOR FIXED 1024-BYTE DAQ PIPE MESSAGES
            -- // SAME SIZE AS SLOTS IN WINDOWS DRIVER
            -- typedef struct _CM_PIPE_DAQ {
            --    uint8_t     dstCMID;        // Destination CM Address
            --    uint8_t     msgID;          // Pipe Message ID, CM_PIPE_DAQ_DATA = 0x10
            --    uint8_t     port;           // Destination Port
            --    uint8_t     flags;          // Message Flags
            --    uint32_t    msgLen;         // Message Length in 32-Bit words
            --    uint32_t    seqID;          // Sequence ID
            --    uint32_t    stamp;          // 32-Bit FPGA Clock Count
            --    uint32_t    stamp_us;       // 32-Bit Time Stamp in microseconds
            --    uint32_t    status;         // Current Machine Status
            --    uint32_t    adc_rate;       // ADC Rate
            --    uint32_t    magic;          // Magic Number
            --    uint16_t    samples[496];   // DAQ Samples
            -- } CM_PIPE_DAQ, *PCM_PIPE_DAQ;
            --
            when HEADER =>
               -- 0th 32-bits in CM_PIPE: dstCMID, msgID, port, flags
               if (ad.hdr_cnt = 0) then
                  ad.state    <= HEADER;
                  -- start sclk while writing header
                  ad.sclk     <= '0';
                  ad.din      <= ad.ch_sel(31);
                  ad.in_we    <= '1';
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"00001584";
               -- 1st 32-bits in CM_PIPE: msgLen
               elsif (ad.hdr_cnt = 1) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"00000100";
               -- 2nd 32-bits in CM_PIPE: seqID
               elsif (ad.hdr_cnt = 2) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= std_logic_vector(ad.seq_id);
                  ad.seq_id   <= ad.seq_id + 1;
               -- 3rd 32-bits in CM_PIPE: stamp
               elsif (ad.hdr_cnt = 3) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= std_logic_vector(stamp);
               -- 4th 32-bits in CM_PIPE: stamp_us
               elsif (ad.hdr_cnt = 4) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"00000000";
               -- 5th 32-bits in CM_PIPE: status
               elsif (ad.hdr_cnt = 5) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"00000000";
               -- 6th 32-bits in CM_PIPE: adc rate
               elsif (ad.hdr_cnt = 6) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"0000" & adc_ADC_RATE;
               -- 7th 32-bits in CM_PIPE: magic
               elsif (ad.hdr_cnt = 7) then
                  ad.state    <= HEADER;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.hdr_cnt  <= ad.hdr_cnt + 1;
                  ad.out_dat  <= X"123455AA";
               else
                  ad.state    <= SCLK_LO;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.in_we    <= '0';
                  ad.hdr_cnt  <= 0;
                  ad.delay    <= ad.delay - 9;
               end if ;

            when SCLK_LO =>
               ad.sclk        <= '0';
               ad.din         <= ad.ch_sel(31);
               if (ad.delay = 0) then
                  ad.state    <= SCLK_HI;
                  ad.bit_cnt  <= ad.bit_cnt + 1;
                  ad.delay    <= to_integer(adc_rate);
                  -- sample the serial ADC data output
                  ad.out_dat  <= ad.out_dat(30 downto 0) & ad_dout;
               else
                  ad.state    <= SCLK_LO;
                  ad.delay    <= ad.delay - 1;
               end if;

            when SCLK_HI =>
               ad.sclk        <= '1';
               if (ad.delay = 0 and ad.bit_cnt = 32 and xl_RAMP = '1') then
                  ad.state    <= STORE;
                  ad.bit_cnt  <= 0;
                  ad.in_we    <= '1';
                  ad.ch_cnt   <= ad.ch_cnt + 1;
                  ad.out_dat  <= std_logic_vector(ad.ramp + 1) & std_logic_vector(ad.ramp);
                  ad.ramp     <= ad.ramp + 2;
                  ad.delay    <= to_integer(adc_rate);
               elsif (ad.delay = 0 and ad.bit_cnt = 32) then
                  ad.state    <= STORE;
                  ad.bit_cnt  <= 0;
                  ad.in_we    <= '1';
                  ad.ch_cnt   <= ad.ch_cnt + 1;
                  ad.delay    <= to_integer(adc_rate);
               elsif (ad.delay = 0) then
                  ad.state    <= SCLK_LO;
                  ad.delay    <= to_integer(adc_rate);
                  ad.ch_sel   <= ad.ch_sel(30 downto 0) & '0';
               else
                  ad.state    <= SCLK_HI;
                  ad.delay    <= ad.delay - 1;
               end if;

            --
            -- STORE 32-BIT ADC WORD (TWO 16-BIT SAMPLES)
            -- AND NEXT CHANNEL SELECTION
            --
            when STORE =>
               if (ad.in_ptr = X"FF" and xl_CH_ALL = '1') then
                  ad.state    <= CHECK;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.pkt_cnt  <= ad.pkt_cnt + 1;
                  ad.head     <= ad.head + 1;
                  ad.in_we    <= '0';
                  ad.out_dat  <= (others => '0');
                  ad.ch_sel   <= "00" & std_logic_vector(ad.ch_cnt) & '0' & "000" & X"00" &
                                 "00" & std_logic_vector(ad.ch_cnt) & '1' & "000" & X"00";
               elsif (ad.in_ptr = X"FF" and xl_CH_ALL = '0') then
                  ad.state    <= CHECK;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.pkt_cnt  <= ad.pkt_cnt + 1;
                  ad.head     <= ad.head + 1;
                  ad.in_we    <= '0';
                  ad.out_dat  <= (others => '0');
                  ad.ch_sel   <= '0' & xl_CH_SEL & "000" & X"00" &
                                 '0' & xl_CH_SEL & "000" & X"00";
               elsif (xl_CH_ALL = '1') then
                  ad.state    <= SCLK_LO;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.in_we    <= '0';
                  ad.delay    <= ad.delay - 1;
                  ad.out_dat  <= (others => '0');
                  ad.ch_sel   <= "00" & std_logic_vector(ad.ch_cnt) & '0' & "000" & X"00" &
                                 "00" & std_logic_vector(ad.ch_cnt) & '1' & "000" & X"00";
               else
                  ad.state    <= SCLK_LO;
                  ad.in_ptr   <= ad.in_ptr + 1;
                  ad.in_we    <= '0';
                  ad.delay    <= ad.delay - 1;
                  ad.out_dat  <= (others => '0');
                  ad.ch_sel   <= '0' & xl_CH_SEL & "000" & X"00" &
                                 '0' & xl_CH_SEL & "000" & X"00";
               end if;

            --
            -- CHECK FOR ALL FRAMES ACQUIRED AND ABORT
            --
            when CHECK =>
               -- Abort
               if (ad.run = '0') then
                  ad.state    <= IDLE;
               -- all requested frames acquired
               elsif (unsigned(adc_PKT_CNT) /= 0 and ad.pkt_cnt = unsigned(adc_PKT_CNT)) then
                  ad.state    <= IDLE;
                  ad.done     <= '1';
               else
                  ad.state    <= HEADER;
                  -- account for extra clocks
                  ad.delay    <= ad.delay - 2;
               end if;

            when others =>
               ad.state       <= IDLE;

         end case;
      end if;
   end process;

   --
   --  MASTER WRITE BURST TRANSFER, WRITE TO ON-CHIP RAM OR SDRAM
   --
   --  NOTES:
   --    * Master read/write addresses are byte pointers.
   --    * Transfers are always 32-Bits.
   --    * adc_PKT_CNT is the number of 256 32-Bit transfers, a 1024 Byte packet
   --
   process(all) begin
      if (reset_n = '0' or xl_ENABLE = '0') then

         -- Init the State Vector
         wr             <= C_WR_SV_INIT;

         -- status is shared by the ADC FSM
         xl_DMA_BUSY    <= '0';
         xl_TAIL        <= (others => '0');
         xl_HEAD_ADDR   <= (others => '0');

      elsif (rising_edge(clk)) then

         -- edge-detect
         wr.run         <= xl_RUN;
         wr.run_r0      <= wr.run;

         -- update status
         xl_DMA_BUSY    <= wr.busy;
         xl_TAIL        <= "00" & std_logic_vector(wr.tail);
         xl_HEAD_ADDR   <= std_logic_vector(wr.head_addr);

         case wr.state is
            when IDLE =>
               -- Look for Run Bit Rising-Edge,
               if (wr.run_r0 = '0' and wr.run = '1') then
                  wr.state    <= WAIT_SLOT;
                  -- Address must be on 32-Bit boundary
                  wr.addr     <= unsigned(adc_ADR_BEG);
                  -- 256 32-Bit transfers (1K Bytes)
                  wr.burstcnt <= '1' & X"00";
                  wr.wrd_cnt  <= (others => '0');
                  wr.pool_cnt <= (others => '0');
                  wr.head_addr <= (others => '0');
                  wr.head_cnt <= (others => '0');
                  wr.xfer_cnt <= (others => '0');
                  wr.tail     <= (others => '0');
                  wr.busy     <= '1';
               -- Look for Run Bit Falling-Edge,
               -- clear head_addr when stop issued
               -- this prevents the head_addr from clearing
               -- before the packets have been sent
              elsif (wr.run_r0 = '1' and wr.run = '0') then
                  wr.state    <= WAIT_SLOT;
                  wr.head_addr <= (others => '0');
              else
                  wr.state    <= IDLE;
                  wr.busy     <= '0';
                  wr.pkt_rdy  <= '0';
                  wr.addr     <= (others => '0');
               end if;

            --
            -- Wait for a Packet in the Circular Buffer
            -- Also check for Abort
            --
            when WAIT_SLOT =>
               -- Abort Transfer
               if (wr.run = '0') then
                  wr.state    <= IDLE;
                  wr.head_addr <= (others => '0');
               -- Account for Circular Memory, Restart
               elsif (wr.addr >= unsigned(adc_ADR_END)) then
                  wr.state    <= WAIT_SLOT;
                  wr.addr     <= unsigned(adc_ADR_BEG);
               -- Increment head_cnt based on adc_XFER_SIZE
               elsif (unsigned(adc_XFER_SIZE) /= 0 and wr.xfer_cnt >= unsigned(adc_XFER_SIZE)) then
                  wr.state    <= WAIT_SLOT;
                  wr.head_cnt <= wr.head_cnt + 1;
                  wr.xfer_cnt <= (others => '0');
               -- Interrupt Pool Count Check, if non-zero
               elsif (unsigned(adc_POOL_CNT) /= 0 and wr.pool_cnt >= unsigned(adc_POOL_CNT)) then
                  wr.state    <= WAIT_SLOT;
                  wr.pool_cnt <= (others => '0');
                  -- packet ready
                  wr.pkt_rdy  <= '1';
                  -- the head_addr is updated every pool count
                  wr.head_addr <= wr.head_cnt;
               elsif (wr.tail /= ad.head) then
                  wr.state    <= DELAY;
                  wr.wrd_cnt  <= wr.wrd_cnt + 1;
               else
                  wr.state    <= WAIT_SLOT;
                  wr.pkt_rdy  <= '0';
               end if;

            --
            -- Account for Block RAM two-cycle latency
            --
            when DELAY =>
               wr.state       <= WR_SLOT;
               wr.master      <= '1';
               wr.wrd_cnt     <= wr.wrd_cnt + 1;

            --
            -- Wait for Burst Transfer to Complete
            --
            when WR_SLOT =>
               if (wr.wrd_cnt = X"101" and m1_wr_waitreq = '0') then
                  wr.state    <= WAIT_SLOT;
                  wr.tail     <= wr.tail + 1;
                  wr.wrd_cnt  <= (others => '0');
                  wr.master   <= '0';
                  wr.addr     <= wr.addr + X"400";
                  wr.pool_cnt <= wr.pool_cnt + 1;
                  wr.xfer_cnt <= wr.xfer_cnt + 1;
               elsif (m1_wr_waitreq = '0') then
                  wr.state    <= WR_SLOT;
                  wr.wrd_cnt  <= wr.wrd_cnt + 1;
                  wr.master   <= '1';
               else
                  wr.state    <= WR_SLOT;
                  wr.master   <= '1';
               end if;

            when others =>
               wr.state       <= IDLE;

         end case;

      end if;
   end process;

   --
   -- ADC 32-BIT STAMP
   --
   process(all) begin
      if (reset_n = '0' or xl_ENABLE = '0') then
         stamp          <= (others => '0');
      elsif (rising_edge(clk)) then
         stamp          <= stamp + 1;
      end if;
   end process;

end rtl;
