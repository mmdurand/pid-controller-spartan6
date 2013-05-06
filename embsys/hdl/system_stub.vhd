-------------------------------------------------------------------------------
-- system_stub.vhd
-------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library UNISIM;
use UNISIM.VCOMPONENTS.ALL;

entity system_stub is
  port (
    fpga_0_RS232_Uart_1_RX_pin : in std_logic;
    fpga_0_RS232_Uart_1_TX_pin : out std_logic;
    fpga_0_mem_bus_mux_0_MEM_ADDR_pin : out std_logic_vector(0 to 22);
    fpga_0_mem_bus_mux_0_DQ_pin : inout std_logic_vector(0 to 15);
    fpga_0_mem_bus_mux_0_MEM_OEN_pin : out std_logic;
    fpga_0_mem_bus_mux_0_MEM_WEN_pin : out std_logic;
    fpga_0_mem_bus_mux_0_RAM_CEN_O_pin : out std_logic;
    fpga_0_mem_bus_mux_0_RAM_BEN_O_pin : out std_logic_vector(0 to 1);
    fpga_0_mem_bus_mux_0_FLASH_ADDR_pin : out std_logic_vector(5 to 7);
    fpga_0_mem_bus_mux_0_FLASH_CEN_O_pin : out std_logic;
    fpga_0_mem_bus_mux_0_FLASH_RPN_O_pin : out std_logic;
    fpga_0_mem_bus_mux_0_QUAD_SPI_C_O_pin : out std_logic;
    fpga_0_mem_bus_mux_0_QUAD_SPI_S_O_pin : out std_logic;
    fpga_0_mem_bus_mux_0_MOSI_QUAD_SPI_pin : inout std_logic;
    fpga_0_clk_1_sys_clk_pin : in std_logic;
    fpga_0_rst_1_sys_rst_pin : in std_logic;
    n3eif_0_btn_east_pin : in std_logic;
    n3eif_0_lcd_rs_pin : out std_logic;
    n3eif_0_rotary_press_pin : in std_logic;
    n3eif_0_rotary_a_pin : in std_logic;
    n3eif_0_btn_west_pin : in std_logic;
    n3eif_0_lcd_rw_pin : out std_logic;
    n3eif_0_rotary_b_pin : in std_logic;
    n3eif_0_leds_out_pin : out std_logic_vector(7 downto 0);
    n3eif_0_sw_pin : in std_logic_vector(3 downto 0);
    n3eif_0_lcd_e_pin : out std_logic;
    n3eif_0_lcd_d_pin : out std_logic_vector(7 downto 0);
    n3eif_0_btn_north_pin : in std_logic;
    xps_gpio_0_GPIO_IO_O_pin : out std_logic_vector(0 to 7);
    xps_timer_0_PWM0_pin : out std_logic;
    xps_spi_0_SCK_O_pin : out std_logic;
    xps_spi_0_MOSI_O_pin : out std_logic;
    xps_spi_0_SS_O_pin : out std_logic;
    xps_spi_0_MISO_I_pin : in std_logic
  );
end system_stub;

architecture STRUCTURE of system_stub is

  component system is
    port (
      fpga_0_RS232_Uart_1_RX_pin : in std_logic;
      fpga_0_RS232_Uart_1_TX_pin : out std_logic;
      fpga_0_mem_bus_mux_0_MEM_ADDR_pin : out std_logic_vector(0 to 22);
      fpga_0_mem_bus_mux_0_DQ_pin : inout std_logic_vector(0 to 15);
      fpga_0_mem_bus_mux_0_MEM_OEN_pin : out std_logic;
      fpga_0_mem_bus_mux_0_MEM_WEN_pin : out std_logic;
      fpga_0_mem_bus_mux_0_RAM_CEN_O_pin : out std_logic;
      fpga_0_mem_bus_mux_0_RAM_BEN_O_pin : out std_logic_vector(0 to 1);
      fpga_0_mem_bus_mux_0_FLASH_ADDR_pin : out std_logic_vector(5 to 7);
      fpga_0_mem_bus_mux_0_FLASH_CEN_O_pin : out std_logic;
      fpga_0_mem_bus_mux_0_FLASH_RPN_O_pin : out std_logic;
      fpga_0_mem_bus_mux_0_QUAD_SPI_C_O_pin : out std_logic;
      fpga_0_mem_bus_mux_0_QUAD_SPI_S_O_pin : out std_logic;
      fpga_0_mem_bus_mux_0_MOSI_QUAD_SPI_pin : inout std_logic;
      fpga_0_clk_1_sys_clk_pin : in std_logic;
      fpga_0_rst_1_sys_rst_pin : in std_logic;
      n3eif_0_btn_east_pin : in std_logic;
      n3eif_0_lcd_rs_pin : out std_logic;
      n3eif_0_rotary_press_pin : in std_logic;
      n3eif_0_rotary_a_pin : in std_logic;
      n3eif_0_btn_west_pin : in std_logic;
      n3eif_0_lcd_rw_pin : out std_logic;
      n3eif_0_rotary_b_pin : in std_logic;
      n3eif_0_leds_out_pin : out std_logic_vector(7 downto 0);
      n3eif_0_sw_pin : in std_logic_vector(3 downto 0);
      n3eif_0_lcd_e_pin : out std_logic;
      n3eif_0_lcd_d_pin : out std_logic_vector(7 downto 0);
      n3eif_0_btn_north_pin : in std_logic;
      xps_gpio_0_GPIO_IO_O_pin : out std_logic_vector(0 to 7);
      xps_timer_0_PWM0_pin : out std_logic;
      xps_spi_0_SCK_O_pin : out std_logic;
      xps_spi_0_MOSI_O_pin : out std_logic;
      xps_spi_0_SS_O_pin : out std_logic;
      xps_spi_0_MISO_I_pin : in std_logic
    );
  end component;

  attribute BOX_TYPE : STRING;
  attribute BOX_TYPE of system : component is "user_black_box";

begin

  system_i : system
    port map (
      fpga_0_RS232_Uart_1_RX_pin => fpga_0_RS232_Uart_1_RX_pin,
      fpga_0_RS232_Uart_1_TX_pin => fpga_0_RS232_Uart_1_TX_pin,
      fpga_0_mem_bus_mux_0_MEM_ADDR_pin => fpga_0_mem_bus_mux_0_MEM_ADDR_pin,
      fpga_0_mem_bus_mux_0_DQ_pin => fpga_0_mem_bus_mux_0_DQ_pin,
      fpga_0_mem_bus_mux_0_MEM_OEN_pin => fpga_0_mem_bus_mux_0_MEM_OEN_pin,
      fpga_0_mem_bus_mux_0_MEM_WEN_pin => fpga_0_mem_bus_mux_0_MEM_WEN_pin,
      fpga_0_mem_bus_mux_0_RAM_CEN_O_pin => fpga_0_mem_bus_mux_0_RAM_CEN_O_pin,
      fpga_0_mem_bus_mux_0_RAM_BEN_O_pin => fpga_0_mem_bus_mux_0_RAM_BEN_O_pin,
      fpga_0_mem_bus_mux_0_FLASH_ADDR_pin => fpga_0_mem_bus_mux_0_FLASH_ADDR_pin,
      fpga_0_mem_bus_mux_0_FLASH_CEN_O_pin => fpga_0_mem_bus_mux_0_FLASH_CEN_O_pin,
      fpga_0_mem_bus_mux_0_FLASH_RPN_O_pin => fpga_0_mem_bus_mux_0_FLASH_RPN_O_pin,
      fpga_0_mem_bus_mux_0_QUAD_SPI_C_O_pin => fpga_0_mem_bus_mux_0_QUAD_SPI_C_O_pin,
      fpga_0_mem_bus_mux_0_QUAD_SPI_S_O_pin => fpga_0_mem_bus_mux_0_QUAD_SPI_S_O_pin,
      fpga_0_mem_bus_mux_0_MOSI_QUAD_SPI_pin => fpga_0_mem_bus_mux_0_MOSI_QUAD_SPI_pin,
      fpga_0_clk_1_sys_clk_pin => fpga_0_clk_1_sys_clk_pin,
      fpga_0_rst_1_sys_rst_pin => fpga_0_rst_1_sys_rst_pin,
      n3eif_0_btn_east_pin => n3eif_0_btn_east_pin,
      n3eif_0_lcd_rs_pin => n3eif_0_lcd_rs_pin,
      n3eif_0_rotary_press_pin => n3eif_0_rotary_press_pin,
      n3eif_0_rotary_a_pin => n3eif_0_rotary_a_pin,
      n3eif_0_btn_west_pin => n3eif_0_btn_west_pin,
      n3eif_0_lcd_rw_pin => n3eif_0_lcd_rw_pin,
      n3eif_0_rotary_b_pin => n3eif_0_rotary_b_pin,
      n3eif_0_leds_out_pin => n3eif_0_leds_out_pin,
      n3eif_0_sw_pin => n3eif_0_sw_pin,
      n3eif_0_lcd_e_pin => n3eif_0_lcd_e_pin,
      n3eif_0_lcd_d_pin => n3eif_0_lcd_d_pin,
      n3eif_0_btn_north_pin => n3eif_0_btn_north_pin,
      xps_gpio_0_GPIO_IO_O_pin => xps_gpio_0_GPIO_IO_O_pin,
      xps_timer_0_PWM0_pin => xps_timer_0_PWM0_pin,
      xps_spi_0_SCK_O_pin => xps_spi_0_SCK_O_pin,
      xps_spi_0_MOSI_O_pin => xps_spi_0_MOSI_O_pin,
      xps_spi_0_SS_O_pin => xps_spi_0_SS_O_pin,
      xps_spi_0_MISO_I_pin => xps_spi_0_MISO_I_pin
    );

end architecture STRUCTURE;

