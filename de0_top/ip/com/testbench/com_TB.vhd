library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.textio.all;

use work.pck_fio.all;
use work.pck_tb.all;

-- Test Bench Entity
entity ftdi_com_tb is
end ftdi_com_tb;

architecture tb_arch of ftdi_com_tb is

-- Stimulus signals - signals mapped to the input and inout ports of tested entity
signal clk                 : std_logic := '0';
signal reset_n             : std_logic := '0';
signal m1_rd_waitreq       : std_logic := '0';
signal m1_writedata        : std_logic_vector(31 downto 0);
signal m1_rd_datavalid     : std_logic := '1';
signal read_n              : std_logic := '1';
signal write_n             : std_logic := '1';
signal address             : std_logic_vector(10 downto 0) := "000" & X"00";
signal writedata           : std_logic_vector(31 downto 0) := X"00000000";
signal sin                 : std_logic := '0';

-- Observed signals - signals mapped to the output ports of tested entity
signal m1_read             : std_logic;
signal m1_rd_address       : std_logic_vector(31 downto 0);
signal m1_rd_burstcount    : std_logic_vector(15 downto 0);
signal readdata            : std_logic_vector(31 downto 0);
signal irq                 : std_logic;
signal sout                : std_logic;
signal test_bit            : std_logic;
signal debug               : std_logic_vector(3 downto 0);

signal ramp                : unsigned(11 downto 0) := X"000";
signal sout                : unsigned(11 downto 0) := X"000";
signal sck_r0              : std_logic := '0';
signal ad_bit_cnt          : integer range 0 to 32 := 0;

-- constants
constant C_CLK_PERIOD:     TIME :=  10.000 ns;    -- 100 MHz

begin

   --
   -- Unit Under Test
   --
   FTDI_COM_I : entity work.ftdi_com
   port map (
      -- Avalon Memory-Mapped Slave
      clk                  => clk,
      reset_n              => reset_n,
      read_n               => read_n,
      write_n              => write_n,
      address              => address,
      readdata             => readdata,
      writedata            => writedata,
      irq                  => irq,
      -- Avalon Memory-Mapped Read Master
      m1_read              => m1_read,
      m1_rd_address        => m1_rd_address,
      m1_readdata          => m1_readdata,
      m1_wr_burstcount     => m1_rd_burstcount,
      m1_rd_datavalid      => m1_rd_datavalid,
      -- Memory Head-Tail Pointers
      head_addr            => X"0000",
      tail_addr            => open,
      -- Exported Signals
      sin                  => sin,
      sout                 => sout,
      test_bit             => test_bit
   );

   --
   -- Clocks
   --

   --
   -- 100 MHZ
   --
   process begin
      clk <= '1';
      wait for C_CLK_PERIOD/2;
      clk <= '0';
      wait for C_CLK_PERIOD/2;
   end process;

   --
   -- Reset
   --
   process begin
      reset_n <= '0';
      wait for 10*C_CLK_PERIOD;
      reset_n <= '1';
      wait;
   end process;

   --
   -- Generate ADC Serial Ramp Input
   --
   process begin
      wait until reset_n = '1';
      ramp  <= (others => '0');
      loop
         wait until rising_edge(clk);
         sck_r0         <= sck;
         if (cnvst = '1') then
            ad_bit_cnt  <= 0;
            sout        <= ramp(10 downto 0) & '0';
            sdo         <= std_logic(ramp(11));
         elsif (sck_r0 = '1' and sck = '0') then
            sdo         <= std_logic(sout(11));
            sout        <= sout(10 downto 0) & '0';
            ad_bit_cnt  <= ad_bit_cnt + 1;
         elsif (ad_bit_cnt = 12) then
            ad_bit_cnt  <= 0;
            ramp        <= ramp + 1;
         end if;
      end loop;
   end process;

   --
   -- Main Process
   --
   process

   procedure BUS_WR(addr: in std_logic_vector(11 downto 0);
                    data: in std_logic_vector(31 downto 0)) is
   begin
      wait until rising_edge(clk);
      wait for (1 ns);
      write_n     <= '0';
      address     <= addr(10 downto 0);
      writedata   <= data;
      wait until rising_edge(clk);
      wait for (1 ns);
      write_n     <= '1';
      address     <= (others => '0');
      writedata   <= (others => '0');
   end;

   procedure BUS_RD(addr: in std_logic_vector(11 downto 0)) is
   begin
      wait until rising_edge(clk);
      wait for (1 ns);
      read_n      <= '0';
      address     <= addr(10 downto 0);
      wait until rising_edge(clk);
      wait for (1 ns);
      read_n      <= '1';
      address     <= (others => '0');
   end;


   begin

      wait until reset_n = '1';

      wait;

   end process;

end tb_arch;

