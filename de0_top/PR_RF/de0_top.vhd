library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.lib_pkg.all;

entity de0_top is
   port(
      -- Global Signals
      iCLK_50M             : in    std_logic;
      iRSTn                : in    std_logic;

      -- DRAM Interface
      oDRAM_ADDR           : out   std_logic_vector(12 downto 0);
      oDRAM_BA             : out   std_logic_vector(1 downto 0);
      oDRAM_CASn           : out   std_logic;
      oDRAM_CKE            : out   std_logic;
      oDRAM_CLK            : out   std_logic;
      oDRAM_CSn            : out   std_logic;
      ioDRAM_DQ            : inout std_logic_vector(15 downto 0);
      oDRAM_DQM            : out   std_logic_vector(1 downto 0);
      oDRAM_RASn           : out   std_logic;
      oDRAM_WEn            : out   std_logic;

      -- EPCS Interface
      oEPCS_DCLK           : out   std_logic;
      oEPCS_NCS0           : out   std_logic;
      oEPCS_ASD0           : out   std_logic;
      iEPCS_DATA0          : in    std_logic;

      -- FTDI ASYNC Serial Interface
      oCOM_TX              : out   std_logic;
      iCOM_RX              : in    std_logic;

      -- STDOUT UART
      oSTDOUT_UART_TX      : out   std_logic;
      iSTDOUT_UART_RX      : in    std_logic;
      oSTDOUT_UART_GND     : out   std_logic;

      -- ADC
      oADC_SCLK            : out   std_logic;
      oADC_CSn             : out   std_logic;
      oADC_DIN             : out   std_logic;
      iADC_DOUT            : in    std_logic;

      -- GPIO
      oLED                 : out   std_logic_vector(3 downto 0);
      ioGPX                : inout std_logic_vector(6 downto 0);
      iGPX                 : in    std_logic_vector(6 downto 0);

      -- Test Points
      oTP1                 : out   std_logic;
      oTP2                 : out   std_logic;
      oTP3                 : out   std_logic;
      oTP4                 : out   std_logic
   );
end de0_top;

architecture rtl of de0_top is

--
-- COMPONENTS
--
component de0_fpga is
   port (
      clk_clk        : in    std_logic;
      reset_reset_n  : in    std_logic;
      locked_export  : out   std_logic;
      watchdog_reset : out   std_logic;
      stdout_rxd     : in    std_logic;
      stdout_txd     : out   std_logic;
      epcs_dclk      : out   std_logic;
      epcs_sce       : out   std_logic;
      epcs_sdo       : out   std_logic;
      epcs_data0     : in    std_logic;
      com_tx_out     : out   std_logic;
      com_rx_in      : in    std_logic;
      com_head_addr  : in    std_logic_vector(15 downto 0);
      com_tail_addr  : out   std_logic_vector(15 downto 0);
      com_test_bit   : out   std_logic;
      com_debug      : out   std_logic_vector(3 downto 0);
      gpi_export     : in    std_logic_vector(6 downto 0);
      gpx_export     : inout std_logic_vector(6 downto 0);
      dram_clk       : out   std_logic;
      sdram_addr     : out   std_logic_vector(12 downto 0);
      sdram_ba       : out   std_logic_vector(1 downto 0);
      sdram_cas_n    : out   std_logic;
      sdram_cke      : out   std_logic;
      sdram_cs_n     : out   std_logic;
      sdram_dq       : inout std_logic_vector(15 downto 0);
      sdram_dqm      : out   std_logic_vector(1 downto 0);
      sdram_ras_n    : out   std_logic;
      sdram_we_n     : out   std_logic;
      adc_cs         : out   std_logic;
      adc_din        : out   std_logic;
      adc_dout       : in    std_logic;
      adc_sclk       : out   std_logic;
      adc_head_addr  : out   std_logic_vector(15 downto 0);
      adc_tail_addr  : in    std_logic_vector(15 downto 0)
   );
end component de0_fpga;

--
-- SIGNAL DECLARATIONS
--

signal async_pll_rst_n     : std_logic;
signal pll_locked          : std_logic;
signal watchdog            : std_logic;
signal watchdog_fired      : std_logic;
signal sys_rst_n           : std_logic;
signal sys_rst_vect        : unsigned(27 downto 0);
signal heartbeat_count     : unsigned(27 downto 0);
signal heartbeat           : std_logic;
signal sw_test_bit         : std_logic;
signal head_addr           : std_logic_vector(15 downto 0);
signal tail_addr           : std_logic_vector(15 downto 0);
signal debug               : std_logic_vector(3 downto 0);

--
-- MAIN CODE
--

   begin

   --
   -- FPGA LED
   --
   oLED(0)              <= heartbeat;
   oLED(1)              <= watchdog_fired;
   oLED(2)              <= '0';
   oLED(3)              <= '0';

   --
   -- TEST POINTS
   --
   oTP1                 <= sw_test_bit;
   oTP2                 <= '0';
   oTP3                 <= '0';
   oTP4                 <= '0';

   oSTDOUT_UART_GND     <= '0';

   --
   -- QSYS NIOS
   --
   u0 : de0_fpga
      port map (
         clk_clk           => iCLK_50M,
         reset_reset_n     => sys_rst_n,
         locked_export     => pll_locked,
         watchdog_reset    => watchdog,
         gpx_export        => ioGPX,
         gpi_export        => iGPX,
         stdout_rxd        => iSTDOUT_UART_RX,
         stdout_txd        => oSTDOUT_UART_TX,
         dram_clk          => oDRAM_CLK,
         sdram_addr        => oDRAM_ADDR,
         sdram_ba          => oDRAM_BA,
         sdram_cas_n       => oDRAM_CASn,
         sdram_cke         => oDRAM_CKE,
         sdram_cs_n        => oDRAM_CSn,
         sdram_dq          => ioDRAM_DQ,
         sdram_dqm         => oDRAM_DQM,
         sdram_ras_n       => oDRAM_RASn,
         sdram_we_n        => oDRAM_WEn,
         epcs_dclk         => oEPCS_DCLK,
         epcs_sce          => oEPCS_NCS0,
         epcs_sdo          => oEPCS_ASD0,
         epcs_data0        => iEPCS_DATA0,
         com_tx_out        => oCOM_TX,
         com_rx_in         => iCOM_RX,
         com_head_addr     => head_addr,
         com_tail_addr     => tail_addr,
         com_test_bit      => sw_test_bit,
         com_debug         => open,
         adc_cs            => oADC_CSn,
         adc_din           => oADC_DIN,
         adc_dout          => iADC_DOUT,
         adc_sclk          => oADC_SCLK,
         adc_head_addr     => head_addr,
         adc_tail_addr     => tail_addr
      );

   --
   -- System Reset, iCLK_50M
   --
   process(all) begin
      if (pll_locked = '0' or watchdog = '1' or iRSTn = '0') then
         sys_rst_vect   <= (others => '0');
         sys_rst_n      <= '0';
      elsif (rising_edge(iCLK_50M)) then
         if (sys_rst_n = '0') then
            sys_rst_vect <= sys_rst_vect + 1;
         end if;
         -- hold reset low for ~85 milliseconds then set high
         if (sys_rst_vect(22) = '1') then
            sys_rst_n <= '1';
         end if;
      end if;
   end process;

   --
   -- Heartbeat, iCLK_50M
   --
   process(all) begin
      if (sys_rst_n = '0') then
          heartbeat_count     <= (others => '0');
          heartbeat           <= '0';
      elsif (rising_edge(iCLK_50M)) then
          if(heartbeat_count(22) = '1') then
              heartbeat_count <= (others => '0');
              heartbeat       <= not heartbeat;
          else
              heartbeat_count <= heartbeat_count + 1;
          end if;
      end if;
   end process;

   --
   -- Capture Watchdog Reset
   --
   process(all) begin
      if (pll_locked = '0' or iRSTn = '0') then
         watchdog_fired       <= '0';
      elsif (rising_edge(iCLK_50M)) then
         if (watchdog_fired = '0' and watchdog = '1') then
            watchdog_fired <= '1';
         else
            watchdog_fired <= watchdog_fired;
         end if;
      end if;
   end process;

   --
   -- Qsys Register Map
   --
   -- pll                        : 0x1000_0000
   -- cpu                        : 0x1001_0000
   -- sdram                      : 0x0000_0000
   -- epcs                       : 0x1002_0000
   -- stamp                      : 0x1003_0000
   -- sysclk                     : 0x1004_0000
   -- systimer                   : 0x1005_0000
   -- watchdog                   : 0x1006_0000
   -- gpx                        : 0x1007_0000
   -- gpi                        : 0x1008_0000
   -- stdout                     : 0x1009_0000
   -- adc                        : 0x100A_0000
   -- com                        : 0x100B_0000
   -- update                     : 0x100C_0000
   --

end rtl;
