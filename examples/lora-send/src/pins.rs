use ariel_os::hal::spi;

#[cfg(context = "nrf52840")]
pub type Spi = spi::main::TWISPI1;
#[cfg(context = "nrf52840")]
ariel_os::hal::define_peripherals!(Peripherals {
    spi_sck: P1_11,
    spi_miso: P1_13,
    spi_mosi: P1_12,
    nss: P1_10,
    reset: P1_06,
    busy: P1_14,
    dio1: P1_15,
    rf_switch_rx: P1_05,
    rf_switch_tx: P1_07,
});
