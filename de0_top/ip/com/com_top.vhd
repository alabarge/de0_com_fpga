library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity com_top is
   generic (
      C_DWIDTH             : integer              := 32;
      C_NUM_REG            : integer              := 16
   );
   port (
      clk                  : in    std_logic;
      reset_n              : in    std_logic;
      read_n               : in    std_logic;
      write_n              : in    std_logic;
      address              : in    std_logic_vector(11 downto 0);
      readdata             : out   std_logic_vector(31 downto 0);
      writedata            : in    std_logic_vector(31 downto 0);
      irq                  : out   std_logic;
      m1_read              : out   std_logic;
      m1_rd_address        : out   std_logic_vector(31 downto 0);
      m1_readdata          : in    std_logic_vector(31 downto 0);
      m1_rd_waitreq        : in    std_logic;
      m1_rd_burstcount     : out   std_logic_vector(15 downto 0);
      m1_rd_datavalid      : in    std_logic;
      head_addr            : in    std_logic_vector(15 downto 0);
      tail_addr            : out   std_logic_vector(15 downto 0);
      rx_in                : in    std_logic;
      tx_out               : out   std_logic;
      test_bit             : out   std_logic;
      debug                : out   std_logic_vector(3 downto 0)
   );
end entity com_top;

architecture rtl of com_top is

--
-- SIGNAL DECLARATIONS
--
   signal com_CONTROL      : std_logic_vector(31 downto 0);
   signal com_ADDR         : std_logic_vector(31 downto 0);
   signal com_INT_REQ      : std_logic_vector(2 downto 0);
   signal com_INT_ACK      : std_logic_vector(2 downto 0);
   signal com_STATUS       : std_logic_vector(31 downto 0);
   signal com_ADR_BEG      : std_logic_vector(31 downto 0);
   signal com_ADR_END      : std_logic_vector(31 downto 0);
   signal com_PKT_CNT      : std_logic_vector(31 downto 0);
   signal com_TICKS        : std_logic_vector(15 downto 0);
   signal com_int          : std_logic_vector(2 downto 0);

   signal cpu_DIN          : std_logic_vector(31 downto 0);
   signal cpu_DOUT         : std_logic_vector(31 downto 0);
   signal cpu_ADDR         : std_logic_vector(11 downto 0);
   signal cpu_WE           : std_logic_vector(1 downto 0);
   signal cpu_RE           : std_logic_vector(2 downto 0);

--
-- MAIN CODE
--
begin

   --
   -- REGISTER FILE
   --
   COM_REGS_I: entity work.com_regs
   generic map (
      C_DWIDTH             => C_DWIDTH,
      C_NUM_REG            => C_NUM_REG
   )
   port map (
      clk                  => clk,
      reset_n              => reset_n,
      read_n               => read_n,
      write_n              => write_n,
      address              => address,
      readdata             => readdata,
      writedata            => writedata,
      cpu_DIN              => cpu_DIN,
      cpu_DOUT             => cpu_DOUT,
      cpu_ADDR             => cpu_ADDR,
      cpu_WE               => cpu_WE,
      cpu_RE               => cpu_RE,
      com_CONTROL          => com_CONTROL,
      com_INT_REQ          => com_INT_REQ,
      com_INT_ACK          => com_INT_ACK,
      com_STATUS           => com_STATUS,
      com_ADR_BEG          => com_ADR_BEG,
      com_ADR_END          => com_ADR_END,
      com_PKT_CNT          => com_PKT_CNT,
      com_TICKS            => com_TICKS,
      com_TEST_BIT         => test_bit
   );

   --
   -- FTDI MESSAGE STATE MACHINE
   --
   COM_CTL_I: entity work.com_ctl
   port map (
      clk                  => clk,
      reset_n              => reset_n,
      int                  => com_int,
      m1_read              => m1_read,
      m1_rd_address        => m1_rd_address,
      m1_readdata          => m1_readdata,
      m1_rd_waitreq        => m1_rd_waitreq,
      m1_rd_burstcount     => m1_rd_burstcount,
      m1_rd_datavalid      => m1_rd_datavalid,
      cpu_DIN              => cpu_DIN,
      cpu_DOUT             => cpu_DOUT,
      cpu_ADDR             => cpu_ADDR,
      cpu_WE               => cpu_WE,
      cpu_RE               => cpu_RE,
      com_CONTROL          => com_CONTROL,
      com_STATUS           => com_STATUS,
      com_ADR_BEG          => com_ADR_BEG,
      com_ADR_END          => com_ADR_END,
      com_PKT_CNT          => com_PKT_CNT,
      com_TICKS            => com_TICKS,
      head_addr            => head_addr,
      tail_addr            => tail_addr,
      rx_in                => rx_in,
      tx_out               => tx_out,
      debug                => debug
   );

   --
   -- INTERRUPTS
   --
   COM_IRQ_I: entity work.com_irq
   generic map (
      C_NUM_INT            => 3
   )
   port map (
      clk                  => clk,
      reset_n              => reset_n,
      int_req              => com_INT_REQ,
      int_ack              => com_INT_ACK,
      int                  => com_int,
      irq                  => irq
   );

end rtl;
