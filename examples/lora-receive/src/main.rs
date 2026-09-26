#![no_main]
#![no_std]

mod pins;

use ariel_os::{
    cell::StaticCell,
    gpio::{Input, Level, Output, Pull},
    hal,
    log::{debug, info, transport::register_custom_transport},
    reexports::embassy_usb::{
        class::cdc_acm::{CdcAcmClass, CdcAcmError, State},
        driver::EndpointError,
    },
    spi::main::{Kilohertz, SpiDevice, highest_freq_in},
    time::{Delay, Timer},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, pipe::Pipe};
use embedded_io_async::Write as _;

use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx126x, Sx1262, TcxoCtrlVoltage};
use lora_phy::{mod_params::*, sx126x};

const LORA_FREQUENCY_IN_HZ: u32 = 868_100_000; // EU868

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) {
    let mut spi_config = hal::spi::main::Config::default();
    spi_config.frequency = const { highest_freq_in(Kilohertz::kHz(1000)..=Kilohertz::kHz(2000)) };
    debug!("Selected frequency: {:?}", spi_config.frequency);

    let nss = Output::new(peripherals.nss, Level::High);
    let reset = Output::new(peripherals.reset, Level::High);
    let rxen = Output::new(peripherals.rxen, Level::Low);

    // Sx126 bind's interrupt on those pin
    let dio1 = Input::builder(peripherals.dio1, Pull::Down)
        .build_with_interrupt()
        .unwrap();
    let busy = Input::builder(peripherals.busy, Pull::None)
        .build_with_interrupt()
        .unwrap();

    let spi_bus = pins::Spi::new(
        peripherals.spi_sck,
        peripherals.spi_miso,
        peripherals.spi_mosi,
        spi_config,
    );

    let spi_bus = Mutex::new(spi_bus);
    let spi = SpiDevice::new(&spi_bus, nss);

    // The Wio-SX1262 feeds its TCXO from DIO3 at 1.8V. Get this wrong and tx() hangs forever.
    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: false,
    };

    // rf_switch_rx = RXEN, rf_switch_tx = None: DIO2 drives the TX side inside the module.
    // We need interrupt pin handler on the GenericSx126xInterfaceVariant
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, Some(rxen), None).unwrap();
    let mut lora = LoRa::new(Sx126x::new(spi, iv, config), false, Delay)
        .await
        .unwrap();
    info!("radio up: SPI, BUSY and RESET are wired correctly");

    let mdltn_params = lora
        .create_modulation_params(
            SpreadingFactor::_7,
            Bandwidth::_125KHz,
            CodingRate::_4_5,
            LORA_FREQUENCY_IN_HZ,
        )
        .unwrap();

    let mut receiving_buffer = [00u8; 100];
    let rx_pkt_params = lora
        .create_rx_packet_params(
            8,
            false,
            receiving_buffer.len() as u8,
            true,
            false,
            &mdltn_params,
        )
        .unwrap();

    match lora
        .prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params)
        .await
    {
        Ok(()) => {}
        Err(err) => {
            info!("Radio error = {:?}", err);
            return;
        }
    };

    loop {
        info!("Waiting for a new package");
        receiving_buffer = [00u8; 100];
        match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
            Ok((received_len, _rx_pkt_status)) => {
                info!(
                    "rx successful {:?}",
                    core::str::from_utf8(&receiving_buffer[..received_len as usize])
                        .unwrap_or("<non utf8>")
                );
                Timer::after_secs(2).await;
            }
            Err(err) => info!("rx unsuccessful = {:?}", err),
        }
    }
}

// Logs over the Pico's own USB CDC-ACM port: `logging-over-custom-transport` only provides the
// hook, the app has to carry the bytes. Copied from examples/usb-serial-logger.
const MAX_FULL_SPEED_PACKET_SIZE: u8 = 64;
const LOG_BUFFER_SIZE: usize = 1024;
static LOG_BUFFER: Pipe<CriticalSectionRawMutex, LOG_BUFFER_SIZE> = Pipe::new();

#[ariel_os::task(autostart, usb_builder_hook)]
async fn usb_logger() {
    register_custom_transport(write_bytes, flush);

    static STATE: StaticCell<State<'_>> = StaticCell::new();

    let class = USB_BUILDER_HOOK
        .with(|builder| {
            CdcAcmClass::new(
                builder,
                STATE.init_with(State::new),
                MAX_FULL_SPEED_PACKET_SIZE.into(),
            )
        })
        .await;

    let (mut sender, mut receiver) = class.split();
    let sender_fut = async {
        sender.wait_connection().await;
        let mut buffer = [0; LOG_BUFFER_SIZE];
        loop {
            let len = LOG_BUFFER.read(&mut buffer).await;
            if matches!(
                sender.write_all(&buffer[..len]).await,
                Err(CdcAcmError::NotConnected)
            ) {
                sender.wait_connection().await;
            }
        }
    };

    // The serial has to be read otherwise the USB device hangs on some MCUs.
    let receiver_fut = async {
        receiver.wait_connection().await;
        let mut buffer = [0; MAX_FULL_SPEED_PACKET_SIZE as usize];
        loop {
            if matches!(
                receiver.read_packet(&mut buffer).await,
                Err(EndpointError::Disabled)
            ) {
                receiver.wait_connection().await;
            }
        }
    };

    embassy_futures::join::join(sender_fut, receiver_fut).await;
}

fn write_bytes(bytes: &[u8]) {
    let mut total = 0;
    while total < bytes.len() {
        match LOG_BUFFER.try_write(&bytes[total..]) {
            Ok(n) => total += n,
            // Pipe full, drop the rest.
            Err(_) => return,
        }
    }
}

fn flush() {}
