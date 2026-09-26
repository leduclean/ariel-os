use ariel_os::hal::spi;

pub type Spi = spi::main::SPI0;
ariel_os::hal::define_peripherals!(Peripherals {
    spi_sck: PIN_2,
    spi_miso: PIN_4,
    spi_mosi: PIN_3,
    nss: PIN_5,
    reset: PIN_13,
    busy: PIN_16,
    dio1: PIN_17,
    rxen: PIN_26
});
