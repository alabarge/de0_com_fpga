library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity com_rx is
   port (
      clk                  : in    std_logic;
      reset                : in    std_logic;
      ticks                : in    std_logic_vector(15 downto 0);
      sin                  : in    std_logic;
      dat_rd               : in    std_logic;
      rdy                  : out   std_logic;
      dout                 : out   std_logic_vector(7 downto 0)
   );
end com_rx;

architecture rtl of com_rx is

--
-- TYPES
--
type  rx_state_t is (IDLE, RX_SAMPLE, RX_NEXT);

type  RX_SV_t is record
   state       : rx_state_t;
   dat         : std_logic_vector(9 downto 0);
   store       : std_logic_vector(7 downto 0);
   bit_cnt     : integer range 0 to 10;
   ticks       : unsigned(15 downto 0);
   done        : std_logic;
   sin         : std_logic;
   sin_r0      : std_logic;
end record RX_SV_t;

--
-- CONSTANTS
--

-- COM State Vector Initialization
constant C_RX_SV_INIT : RX_SV_t := (
   state       => IDLE,
   dat         => (others => '0'),
   store       => (others => '0'),
   bit_cnt     => 0,
   ticks       => (others => '0'),
   done        => '0',
   sin         => '0',
   sin_r0      => '0'
);

--
-- SIGNAL DECLARATIONS
--

--RX State Vector
signal rx                  : RX_SV_t;

signal rx_dout             : std_logic_vector(7 downto 0);
signal rx_rdy              : std_logic;

--
-- MAIN CODE
--
begin

   --
   -- COMBINATORIAL OUTPUTS
   --
   dout     <= rx_dout;
   rdy      <= rx_rdy;

   --
   --  SERIAL RECIEVE FSM
   --
   process(all) begin
      if (reset = '1') then

         -- Init the State Vector
         rx             <= C_RX_SV_INIT;

      elsif (rising_edge(clk)) then

         -- double-buffer sin input
         rx.sin_r0      <= sin;
         rx.sin         <= rx.sin_r0;

         case rx.state is
            --
            -- IDLE, WAIT FOR SERIAL START BIT
            --
            when IDLE =>
               --
               -- Look for Start Bit, Falling-Edge
               --
               if (rx.sin = '1' and rx.sin_r0 = '0') then
                  rx.state    <= RX_SAMPLE;
                  -- Sample half-way through the bit
                  rx.ticks    <= '0' & unsigned(ticks(15 downto 1));
                  rx.done     <= '0';
                  rx.dat      <= (others => '0');
                  rx.bit_cnt  <= 0;
               else
                  rx.state     <= IDLE;
                  rx.done      <= '0';
               end if;

            --
            -- Sample the Incoming Serial Bit at Mid-point
            --
            when RX_SAMPLE =>
               -- Check for time to sample
               if (rx.ticks = 0) then
                  rx.state    <= RX_NEXT;
                  rx.ticks    <= '0' & unsigned(ticks(15 downto 1));
                  -- LSB first
                  rx.dat      <= rx.sin & rx.dat(9 downto 1);
                  rx.bit_cnt  <= rx.bit_cnt + 1;
               else
                  rx.state    <= RX_SAMPLE;
                  rx.ticks    <= rx.ticks - 1;
               end if;

            --
            -- 10-Bits per Symbol, Wait Second-Half Bit Time
            --
            when RX_NEXT =>
               if (rx.bit_cnt = 10) then
                  rx.state    <= IDLE;
                  rx.bit_cnt  <= 0;
                  rx.done     <= '1';
                  -- drop start/stop bits
                  rx.store    <= rx.dat(8 downto 1);
               -- Check for expired bit period
               elsif (rx.ticks = 0) then
                  rx.state    <= RX_SAMPLE;
                  rx.ticks    <= '0' & unsigned(ticks(15 downto 1));
               else
                  rx.state    <= RX_NEXT;
                  rx.ticks    <= rx.ticks - 1;
               end if;

            when others =>
               rx.state       <= IDLE;

         end case;

      end if;
   end process;

   --
   --  LATCH SYMBOL FOR READING
   --
   process(all) begin
      if (reset = '1') then
         rx_rdy      <= '0';
         rx_dout     <= (others => '0');
      elsif (rising_edge(clk)) then
         if (rx.done = '1') then
            rx_dout     <= rx.store;
            rx_rdy      <= '1';
         elsif (dat_rd = '1') then
            rx_rdy      <= '0';
         end if;
      end if;
   end process;

end rtl;
