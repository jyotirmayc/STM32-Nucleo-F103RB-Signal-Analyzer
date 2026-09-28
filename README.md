# STM32 Nucleo-F103RB Signal Analyzer

A comprehensive, real-time, single-channel signal analyzer built on the STM32 Nucleo-F103RB development board. The firmware continuously samples an analog input on pin **PA0** with the on-chip 12-bit ADC and streams calibrated voltage readings over the board's USB Virtual COM Port to a host PC, where they can be viewed as a live waveform, logged to a file, or analyzed further.

This guide walks through the whole project: wiring, toolchain installation, CubeMX configuration, firmware, verification, host-side plotting, error handling, troubleshooting, accuracy tuning, and a roadmap toward a higher-performance design.

> **Who is this for?** Students, hobbyists, and engineers who want a low-cost data logger / low-frequency oscilloscope, and a practical introduction to the STM32 HAL, the ADC peripheral, and UART streaming.

## Table of Contents
1. [Project Overview & Specifications](#project-overview--specifications)
2. [System Architecture](#system-architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Circuit & Wiring](#circuit--wiring)
5. [How the Measurement Works](#how-the-measurement-works)
6. [Installation Tutorial](#installation-tutorial)
7. [CubeMX Configuration](#cubemx-configuration)
8. [Robust Code Implementation](#robust-code-implementation)
9. [Build, Flash & Verify](#build-flash--verify)
10. [Host-Side Visualization](#host-side-visualization)
11. [System Error Handling](#system-error-handling)
12. [Comprehensive Troubleshooting](#comprehensive-troubleshooting)
13. [Accuracy & Noise-Reduction Tips](#accuracy--noise-reduction-tips)
14. [Future Scope](#future-scope)
15. [Appendix](#appendix)

---

## Project Overview & Specifications

### Feature Summary

* Continuous sampling of one analog channel (PA0 / ADC1 channel 0).
* Conversion of raw ADC codes to volts on the MCU; the host receives ready-to-plot numbers.
* Plain-text output (one value per line), compatible with the Arduino Serial Plotter, terminal programs, spreadsheets, and custom scripts.
* Runtime error checking with timeouts, so a faulty peripheral can never freeze the main loop.
* Optional hardening: ADC self-calibration, averaging, fixed-period scheduling, automatic ADC recovery, and a watchdog.

### Specifications

| Parameter | Value | Notes |
|---|---|---|
| Microcontroller | STM32F103RBT6 | Arm Cortex-M3, up to 72 MHz, 128 KB Flash, 20 KB SRAM. **No FPU**: floating-point math runs in software. |
| Input channel | ADC1_IN0 on PA0 | Single-ended, one channel. Arduino header pin **A0**. |
| Operating range | **0 to 3.3 V DC maximum** | See the warning below. |
| Resolution | 12-bit (codes 0 to 4095) | 1 LSB ≈ 0.806 mV at a 3.3 V reference. |
| Voltage reference | VREF+ = VDDA ≈ 3.3 V | Taken from the board's 3.3 V regulator; **not** a precision reference. |
| Sampling architecture | Software-polled ADC using blocking HAL calls | Simple and easy to debug; see [Future Scope](#future-scope) for timer/DMA upgrades. |
| Nominal sample rate | ≈ 100 Hz (10 ms period) | Baseline code lands at roughly 93–95 Hz because of processing overhead. |
| Usable signal bandwidth | DC to ≈ 10 Hz for clean waveforms | Nyquist limit is ≈ 50 Hz; at least 5–10 samples per cycle are needed for a recognizable shape. |
| Output format | ASCII, one voltage per line, `\r\n` terminated | Example: `1.650\r\n` |
| Communication | USART2 → ST-LINK Virtual COM Port → USB | 115200 bps, 8 data bits, no parity, 1 stop bit (8N1). |
| Power | USB, through the on-board ST-LINK | No external supply required. |

> **⚠️ Warning – electrical limits.** The ADC input tolerates roughly **0 V to 3.3 V** (absolute maximum is about VSS − 0.3 V to VDD + 0.3 V). Applying negative voltages or signals above 3.3 V can permanently damage the MCU. Sensors powered from 5 V (for example, many MQ-series gas sensor modules) can output more than 3.3 V, so they must be scaled down first. See [Circuit & Wiring](#circuit--wiring).

### Design Decisions & Trade-offs

| Decision | Why | Trade-off |
|---|---|---|
| Software polling instead of timer + DMA | Fewest moving parts; ideal for learning and debugging | Sample timing depends on CPU load; limited to low rates |
| ASCII text output | Works with any serial tool, human readable | Uses more bandwidth than binary (7 bytes per sample vs. 2) |
| Blocking HAL calls **with timeouts** | Predictable code flow without risking an endless hang | A failing call costs up to the timeout duration |
| 115200 baud | Universally supported default | Caps the text stream at roughly 1,600 samples/s |
| Voltage computed on the MCU | Host tools can plot directly | Software floating point is slow on a Cortex-M3 (irrelevant at 100 Hz) |

### Known Limitations

* Single channel, DC-coupled, unipolar (0–3.3 V) input only. AC signals such as audio need a bias circuit (see [Future Scope](#future-scope)).
* Accuracy is limited by the nominal 3.3 V reference, ADC offset/gain error, and noise. Uncalibrated results are typically good to a few tens of millivolts.
* Sample timing has some jitter and is not suitable for spectral analysis in the baseline design.
* One-way communication: the PC only listens, and the firmware does not process any host commands.

---

## System Architecture

### Block Diagram

```text
 Analog source (0 – 3.3 V)
        │
        ▼  PA0 (Arduino header A0)
 ┌───────────────────────────────┐
 │  STM32F103RB                  │
 │  ADC1 ─▶ CPU ─▶ USART2        │
 │  12-bit  math   115200 8N1    │
 └───────────────┬───────────────┘
                 │ PA2 (TX) / PA3 (RX)
                 ▼
 ┌───────────────────────────────┐
 │  On-board ST-LINK/V2-1        │
 │  (Virtual COM Port bridge)    │
 └───────────────┬───────────────┘
                 │ USB (Mini-B cable)
                 ▼
 ┌───────────────────────────────┐
 │  Host PC                      │
 │  Serial Plotter / Python /    │
 │  terminal / data logger       │
 └───────────────────────────────┘
```

### Firmware Data Flow

1. **Start** an ADC conversion (`HAL_ADC_Start`).
2. **Wait** for it to finish, with a 10 ms timeout (`HAL_ADC_PollForConversion`).
3. **Read** the 12-bit result (`HAL_ADC_GetValue`).
4. **Convert** to volts: `voltage = raw / 4095 × 3.3`.
5. **Format** the number as text (`sprintf` / `snprintf`).
6. **Transmit** the text on USART2 with a 10 ms timeout (`HAL_UART_Transmit`).
7. **Wait** until the next sample is due (`HAL_Delay` in the baseline code).

### Pin Assignment

| Function | MCU pin | Board connector / label | Notes |
|---|---|---|---|
| Analog input (ADC1_IN0) | PA0 | Arduino header **A0** (CN8) | The only signal input |
| USART2_TX | PA2 | Hard-wired to the ST-LINK VCP (also on header **D1**) | Do not connect anything else here |
| USART2_RX | PA3 | Hard-wired to the ST-LINK VCP (also on header **D0**) | Unused by the firmware for now |
| User LED **LD2** | PA5 | On-board green LED (also **D13**) | Used as an error indicator in the robust code |
| User button **B1** | PC13 | On-board blue button | Unused |
| SWDIO / SWCLK | PA13 / PA14 | ST-LINK debug interface | Keep reserved (`Debug = Serial Wire`) |
| 3.3 V supply | – | Power header CN6, pin 4 | Feed a potentiometer from here |
| GND | – | Power header CN6, pins 6 and 7 | Common ground for the signal source |

### On-Board Indicators

| LED | Meaning |
|---|---|
| **LD3** (red) | Board power present |
| **LD1** (red/green) | ST-LINK communication activity. It flickers while flashing or while the PC talks to the virtual COM port. |
| **LD2** (green) | User LED on PA5. The robust code toggles it when a UART error occurs. |

---

## Hardware Requirements

### Bill of Materials

| Qty | Item | Notes |
|---|---|---|
| 1 | **STM32 Nucleo-F103RB** development board (STM32F103RBT6) | ST-LINK/V2-1 debugger and virtual COM port are built in |
| 1 | **Mini-USB (Mini-B) cable**, data-capable | Used for both programming and serial data. Charge-only cables will **not** work. |
| 1 | Standard **female-to-male jumper wires** (assorted) | For the Nucleo headers |
| 1 | **Test signal source** | A 10 kΩ potentiometer (best for first tests), an analog sensor, or an external DC supply that shares ground with the Nucleo |

### Recommended Extras

| Qty | Item | Purpose |
|---|---|---|
| 1 | Solderless breadboard | Holds the potentiometer and filter parts |
| 1 | 100 nF ceramic capacitor | Input filter capacitor at PA0 |
| 2 | 10 kΩ, 1 % resistors | Build a precise 1.65 V mid-scale reference for verification |
| 1 | 1 kΩ–10 kΩ resistor | Series protection resistor for the input |
| 1 | Digital multimeter | Verify the 3.3 V rail and cross-check readings |
| 2–3 | Resistors for a voltage divider (see below) | Only needed for 5 V or higher signals |
| optional | Dual Schottky diode (e.g. BAT54S) | Clamp against over/under-voltage |

### Test Signal Sources

* **Potentiometer (recommended):** a 10 kΩ linear pot gives a smooth 0–3.3 V sweep and is impossible to over-drive when wired as shown below.
* **Analog sensor (e.g. MQ-135 gas sensor):** check the module's supply voltage and output range first. Many modules run their heater from 5 V and their analog output can approach 5 V. Use a divider (next section). Gas sensors also need a warm-up period before readings stabilize.
* **External DC supply / function generator:** only DC in the 0–3.3 V range. The negative terminal **must** connect to a Nucleo `GND` pin (common ground).

> **Tip – leave the board's factory jumpers alone.** The Nucleo ships with jumpers set for USB power, the ST-LINK connection to the MCU (both `CN3` jumpers fitted), and the current-measurement jumper (`IDD`, labelled `JP6` on this board) fitted. Removing any of them can cause "target not responding" errors or power the MCU down. See the Nucleo-64 user manual (UM1724) for details.

---

## Circuit & Wiring

### Potentiometer Test Circuit

```text
   3V3 (CN6 pin 4) ────────┐
                           │
                        ┌──┴──┐
                        │     │
   A0 / PA0 ◄───────────┤ 10k │  wiper (middle pin)
   (optional 100 nF     │ pot │
    from A0 to GND)     │     │
                        └──┬──┘
                           │
   GND (CN6 pin 6 or 7) ───┘
```

1. Connect one outer pin of the potentiometer to **3V3**, the other outer pin to **GND**.
2. Connect the middle pin (the wiper) to **A0**.
3. Optionally place a 100 nF capacitor between A0 and GND, close to the header, to suppress noise.

### External DC Source

* Connect the source's negative terminal to any Nucleo `GND` pin **before** connecting the positive lead.
* Keep the signal within 0–3.3 V at all times, including during power-up and power-down of the source.
* Add a 1 kΩ–10 kΩ series resistor between the source and A0 to limit current if an accident happens.

### Scaling Signals Above 3.3 V (Voltage Divider)

A resistive divider reduces a higher voltage to a level the ADC can safely read:

```text
   Vin ──┬── R1 ──┬── to A0 (PA0)
         │        │
         │        R2
         │        │
        (source)  GND
```

`V_adc = V_in × R2 / (R1 + R2)` and `V_in = V_adc × (R1 + R2) / R2`

| Maximum input | R1 (top) | R2 (bottom) | ADC voltage at max input | Multiply firmware reading by |
|---|---|---|---|---|
| 5 V | 10 kΩ | 15 kΩ | 3.00 V | 1.667 |
| 10 V | 22 kΩ | 10 kΩ | 3.13 V | 3.2 |
| 12 V | 33 kΩ | 10 kΩ | 2.79 V | 4.3 |

* Use 1 % resistors if you care about absolute accuracy. The divider's parallel resistance is the ADC's **source impedance**, so raise the ADC sampling time as described in [How the Measurement Works](#how-the-measurement-works).
* Set `INPUT_SCALE` in the robust firmware to the multiplier from the table so the host sees the real input voltage.
* Never connect mains voltage or anything above what your divider is rated for.

### Input Protection & Filtering (Recommended)

```text
   Signal ──► R_series (1–10 kΩ) ──┬──► PA0
                                    │
                             C (100 nF) to GND
```

* **Series resistor:** limits fault current into the pin's internal protection diodes (keep injected current within the datasheet limit of a few mA).
* **Schottky clamp (optional):** a BAT54S from the input node to 3V3 and GND catches over/under-shoot.
* **RC low-pass filter:** the cutoff frequency is `f_c = 1 / (2π·R·C)`. For example, 4.7 kΩ with 1 µF gives ≈ 34 Hz, which is a sensible anti-aliasing filter for a ~100 Hz sample rate.

### Grounding & Layout Practices

* Always share ground between the signal source and the Nucleo. A floating ground causes wildly fluctuating readings.
* Keep analog wires short and away from switching signals (motors, relays, PWM lines).
* Twist the signal and ground wires together for longer runs.
* Power everything from the same USB port/hub when possible to avoid ground loops.

---

## How the Measurement Works

Understanding the ADC makes it much easier to pick the right settings and to interpret what you see on the plot.

### The Conversion Chain

1. During the **sampling time**, the voltage on PA0 charges a small internal sample-and-hold capacitor through the ADC's input switch.
2. A **successive-approximation (SAR)** converter then resolves 12 bits, which takes 12.5 ADC clock cycles.
3. The result is placed in the ADC data register; `HAL_ADC_GetValue()` reads it.
4. The CPU converts the code into volts and prints it.

`conversion time = (sampling cycles + 12.5) / ADC clock`

The STM32F103 ADC clock must not exceed **14 MHz**. CubeMX picks a prescaler for you (typically giving 12 MHz on a 72 MHz system clock).

### Sampling Time and Source Impedance

The ADC needs enough time to charge its sample capacitor through your signal source's resistance. A high-impedance source with a short sampling time gives **inaccurate, too-low, or cross-talking readings**. CubeMX defaults to the shortest option (1.5 cycles), which is only suitable for very low-impedance sources.

| Sampling time (cycles) | Total conversion time @ 12 MHz | Approx. max source impedance* |
|---|---|---|
| 1.5 | 1.2 µs | ≈ 0.4 kΩ |
| 7.5 | 1.7 µs | ≈ 5.9 kΩ |
| 13.5 | 2.2 µs | ≈ 11 kΩ |
| 28.5 | 3.4 µs | ≈ 25 kΩ |
| 41.5 | 4.5 µs | ≈ 37 kΩ |
| 55.5 | 5.7 µs | ≈ 50 kΩ |
| 71.5 | 7.0 µs | > 50 kΩ (datasheet lists "N/A") |
| 239.5 | 21 µs | > 50 kΩ (datasheet lists "N/A") |

\*Values follow the STM32F103x8/xB datasheet (DS5319) for a 14 MHz ADC clock; check the current datasheet for exact figures.

**Recommendation for this project:** use **71.5 cycles** (or even 239.5). At 100 samples per second the extra microseconds cost nothing, and a 10 kΩ potentiometer (worst-case wiper impedance ≈ 2.5 kΩ) or a resistive divider will read correctly.

### Transfer Function and Resolution

```text
voltage = (raw / 4095) × VREF          (used by this project)
1 LSB   = VREF / 4096 ≈ 0.806 mV       (ideal step size at VREF = 3.3 V)
```

Dividing by 4095 maps code 4095 to exactly 3.300 V; dividing by 4096 is the "ideal" transfer function. The difference is under one LSB and is well inside the ADC's specified error, so either choice is acceptable.

| Raw code | Voltage (÷ 4095) |
|---|---|
| 0 | 0.000 V |
| 1 | 0.001 V (0.806 mV) |
| 2048 | 1.650 V |
| 4095 | 3.300 V |

**Text resolution matters:** printing with `%.2f` produces 10 mV steps, which is *coarser* than the ADC (0.8 mV). Use `%.3f` to keep the full resolution.

### Sample Rate, Nyquist, and Aliasing

* The sample rate is `f_s = 1 / period`. With a 10 ms period, f_s ≈ 100 Hz.
* By the Nyquist theorem, signals above `f_s / 2` (≈ 50 Hz) **alias** and appear as false low-frequency signals.
* For a plot that looks like the real waveform, aim for at least 5–10 samples per cycle. At 100 Hz sampling that means signals up to roughly 10–20 Hz.
* Filtering out frequencies above `f_s / 2` **before** the ADC (see the RC filter earlier) prevents aliasing.

### UART Throughput Budget

Each byte on an 8N1 UART takes 10 bit-times (start + 8 data + stop).

| Item | Value |
|---|---|
| Link capacity at 115200 baud | 11,520 bytes/s |
| Bytes per sample (`"1.650\r\n"`) | 7 |
| Maximum sample rate (text) | ≈ 1,645 samples/s |
| Transmit time per line (blocking) | ≈ 0.61 ms |
| Link utilization at 100 samples/s | ≈ 6 % |

The MCU spends about 0.6 ms of each 10 ms period blocked in `HAL_UART_Transmit`, plus the time spent formatting the float.

### Floating-Point Cost

The Cortex-M3 has no floating-point unit, so `float` arithmetic and `printf("%f")` are emulated in software. This is perfectly fine at 100 Hz, but for kHz-range sampling use integer math (for example, millivolts as an integer) or a binary protocol.

---

## Installation Tutorial

This project requires a decoupled STMicroelectronics toolchain: one tool to configure the hardware (CubeMX) and one to write, build, and flash the code (CubeIDE).

> **Note:** Some versions of STM32CubeIDE bundle the CubeMX configurator, so you can create the `.ioc` file directly inside the IDE. This guide uses standalone CubeMX for consistency across versions; the settings are identical either way.

### System Requirements

* Windows 10/11 (64-bit), a recent Linux distribution, or macOS.
* Several GB of free disk space for the IDE, the STM32Cube F1 firmware package, and generated projects.
* An internet connection for the initial downloads (CubeMX fetches the STM32F1 firmware package on first use).
* A free USB port and a data-capable Mini-USB cable.

### Step 1: Install STM32CubeMX (Hardware Configurator)

1. Navigate to the [STM32CubeMX Download Page](https://www.st.com/en/development-tools/stm32cubemx.html).
2. Scroll to **Get Software** and download the standard **STM32CubeMX** (do not download STM32CubeMX2, as it is only for the C5 series).
3. Create an ST account or check out as a guest to receive the download link.
4. Extract the archive and run the installer. Leave the default shortcut settings checked.
5. On first launch, let CubeMX download the **STM32Cube MCU Package for STM32F1** when prompted (also available under *Help → Manage embedded software packages*).

### Step 2: Install STM32CubeIDE (Development Environment)

1. Navigate to the [STM32CubeIDE Download Page](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Download Version 2.0 (or latest).
3. Run the installer. Ensure you install the included **ST-LINK drivers** when prompted during the installation wizard.
4. Linux users: accept the installation of the ST-LINK udev rules, otherwise the debugger may need root privileges.

### Step 3: Verify the ST-LINK Connection

1. Plug the Nucleo into the PC with the Mini-USB cable. The red power LED (**LD3**) should light.
2. Confirm the operating system sees the virtual COM port:
   * **Windows:** *Device Manager → Ports (COM & LPT)* shows "STMicroelectronics STLink Virtual COM Port (COMx)". If it is missing or flagged with a warning icon, install the ST-LINK USB driver from ST (package **STSW-LINK009**).
   * **Linux:** run `ls /dev/ttyACM*` (or `dmesg | tail`). Add yourself to the serial group so you can open the port without `sudo`: `sudo usermod -aG dialout $USER`, then log out and back in. (The group is `uucp` on Arch-based distributions.)
   * **macOS:** run `ls /dev/cu.usbmodem*`.
3. Keep the ST-LINK firmware up to date with STM32CubeProgrammer or the ST-LINK upgrade option in CubeIDE's debug configuration. An outdated ST-LINK is a common cause of connection problems.

### Step 4: Install a Serial Viewer (Host PC)

To view the data, install a serial plotting tool such as:

* **Arduino IDE:** features a built-in "Serial Plotter" (details in [Host-Side Visualization](#host-side-visualization)).
* **Tera Term / PuTTY:** for viewing raw text data.
* **Python (optional):** `pip install pyserial matplotlib` for the custom live-plot and logging script provided in this guide.

Only **one** program can hold the COM port at a time, so close one viewer before opening another.

---

## CubeMX Configuration

### 1. Create the Project from the Board

Open **STM32CubeMX**, choose **File → New Project**, and open the **Board Selector** tab. Search for `NUCLEO-F103RB`, select it, and click **Start Project**. When asked *"Initialize all peripherals with their default mode?"*, answer **Yes**. This pre-configures USART2 for the virtual COM port, the LD2 LED, the B1 button, and the SWD debug pins.

### 2. System Core → SYS

Navigate to `SYS` and set **Debug** to **Serial Wire**. This is critical to prevent locking out the ST-LINK debugger on subsequent flashes. Leave the **Timebase Source** on **SysTick** (the default).

### 3. Analog → ADC1

Enable **IN0**. This maps the ADC input to pin **PA0** (it turns green in the pinout view). Ensure "Continuous Conversion Mode" remains **disabled**. Then review the **Parameter Settings** tab:

| Setting | Value | Why |
|---|---|---|
| Data Alignment | Right alignment | Result is a plain 0–4095 number |
| Scan Conversion Mode | Disabled | Single channel |
| Continuous Conversion Mode | **Disabled** | Software triggers each conversion |
| Discontinuous Conversion Mode | Disabled | Not needed |
| Number of Conversions | 1 | One channel |
| External Trigger Conversion Source | Regular Conversion launched by software | No timer trigger (yet) |
| Rank 1 → Channel | Channel 0 | PA0 |
| Rank 1 → **Sampling Time** | **71.5 Cycles** (see the table above) | Handles potentiometers and dividers; the 1.5-cycle default is too short |

### 4. Connectivity → USART2

Set Mode to **Asynchronous**. Verify the settings:

| Setting | Value |
|---|---|
| Baud Rate | **115200** bits/s |
| Word Length | 8 Bits (including parity) |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |

The pins should read **PA2 = USART2_TX** and **PA3 = USART2_RX**. *(Note: a yellow warning triangle here is normal due to shared pins. Just confirm that PA2/PA3 are assigned and not flagged in red.)*

### 5. Clock Configuration

Open the **Clock Configuration** tab and confirm there are no red/highlighted errors. The **ADC prescaler** must produce an ADC clock **≤ 14 MHz** (for example, PCLK2 ÷ 6). If you change the system clock later, revisit this tab; a wrong clock also changes the UART baud rate accuracy.

### 6. GPIO (Optional, Recommended)

With default peripherals enabled, **PA5** is already configured as an output labelled **LD2**. The robust firmware uses `LD2_GPIO_Port` / `LD2_Pin` to signal errors. If you skipped the default initialization, set PA5 to *GPIO_Output* and give it the user label `LD2`.

### 7. Project Manager

* **Project Name:** choose a name with no spaces or special characters (for example, `NucleoSignalAnalyzer`).
* **Project Location:** set a valid, short directory path (long paths can break builds on Windows).
* **Toolchain / IDE:** set to **STM32CubeIDE**.
* **Code Generator tab:** keep *"Copy only the necessary library files"* and *"Keep User Code when re-generating"* enabled.

### 8. Generate the Code

Click **GENERATE CODE** in the top right. When it finishes, open the project in STM32CubeIDE (*File → Open Projects from File System…*, or use the prompt from CubeMX).

> **Golden rule:** only write your own code between the `USER CODE BEGIN` and `USER CODE END` comments. CubeMX preserves those regions when you regenerate; everything outside them is overwritten.

### Expected Project Layout

```text
NucleoSignalAnalyzer/
├── Core/
│   ├── Inc/            main.h, stm32f1xx_hal_conf.h, stm32f1xx_it.h
│   └── Src/            main.c, stm32f1xx_hal_msp.c, stm32f1xx_it.c, ...
├── Drivers/            STM32F1xx_HAL_Driver, CMSIS
├── NucleoSignalAnalyzer.ioc     ← CubeMX configuration (re-open to change hardware settings)
└── STM32F103RBTX_FLASH.ld       ← linker script
```

---

## Robust Code Implementation

Import the generated project into STM32CubeIDE.

### Mandatory Step: Enable Floating-Point Formatting

By default the toolchain uses *newlib-nano*, which strips `%f` support from `printf` to save memory. Enable it:

Right-click your project in the Explorer window → **Properties** → **C/C++ Build** → **Settings** → **Tool Settings** tab → **MCU Settings** (called *MCU/MPU Settings* in some versions). Check the box for **"Use float with printf from newlib-nano"**, apply, and rebuild.

> Prefer not to spend the extra Flash (several KB)? See the integer-only variant later in this section.

### Version A: Baseline Loop

This is the minimal implementation and is a good first test. Navigate to `Core/Src/main.c`, add the standard I/O includes and variables, then implement the following loop. This version includes runtime error checking.

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
char uart_buf[50];
/* USER CODE END PV */

/* ... Inside main() ... */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. Start the ADC
    HAL_ADC_Start(&hadc1);

    // 2. Wait for conversion with a 10 ms timeout to prevent infinite hangs
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      // 3. Retrieve the raw 12-bit value
      uint16_t raw_adc_value = HAL_ADC_GetValue(&hadc1);

      // 4. Convert digital value to analog voltage (3.3 V reference)
      float voltage = ((float)raw_adc_value / 4095.0f) * 3.3f;

      // 5. Format the data for the serial monitor
      int len = sprintf(uart_buf, "%.2f\r\n", voltage);

      // 6. Transmit data. Check if UART is busy/fails.
      if (HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 10) != HAL_OK)
      {
          // Handle UART transmission error (e.g., toggle an error LED here)
      }
    }
    else
    {
       // Handle ADC timeout error (e.g., reset ADC peripheral)
    }

    // 7. Delay to establish a steady sampling rate
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```

**Limitations of Version A:** the 2-decimal output has only 10 mV resolution; the loop period is "10 ms plus processing time" and therefore drifts; there is no ADC calibration; and the error branches are empty placeholders. Version B addresses all of these.

### Version B: Recommended Implementation

Version B adds ADC self-calibration, averaging, a drift-free sample period, error counters with an LED indicator, and automatic ADC recovery. Paste each block into the matching `USER CODE` region of `main.c`.

**1. Includes and constants**

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define VREF_VOLTS             3.300f  /* Measure your 3V3 rail with a multimeter and update this   */
#define ADC_FULL_SCALE         4095.0f
#define INPUT_SCALE            1.0f    /* 1.0 = direct input; 1.667 for a 10k/15k divider           */
#define SAMPLE_PERIOD_MS       10U     /* 100 Hz nominal                                            */
#define OVERSAMPLE_COUNT       8U      /* 1 = no averaging; keep small to bound the loop time       */
#define ADC_TIMEOUT_MS         10U
#define UART_TIMEOUT_MS        10U
#define ADC_MAX_CONSEC_ERRORS  5U
/* USER CODE END PD */
```

**2. Variables and prototypes**

```c
/* USER CODE BEGIN PV */
static char     uart_buf[32];
static uint32_t adc_error_count   = 0;  /* lifetime ADC failures  - watch in Live Expressions */
static uint32_t uart_error_count  = 0;  /* lifetime UART failures                              */
static uint8_t  adc_consec_errors = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ADC_ReadAveraged(uint16_t *result);
static void              ADC_Recover(void);
/* USER CODE END PFP */
```

**3. One-time initialization (after the `MX_..._Init()` calls)**

```c
  /* USER CODE BEGIN 2 */
  /* One-time ADC self-calibration (STM32F1 series). Run it while the ADC is idle. */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  uint32_t next_tick = HAL_GetTick();
  /* USER CODE END 2 */
```

**4. Main loop body** (inside `while (1)`, in the `USER CODE BEGIN 3` region)

```c
    /* USER CODE BEGIN 3 */
    uint16_t raw;

    if (ADC_ReadAveraged(&raw) == HAL_OK)
    {
      adc_consec_errors = 0;

      float voltage = ((float)raw / ADC_FULL_SCALE) * VREF_VOLTS * INPUT_SCALE;
      int   len     = snprintf(uart_buf, sizeof(uart_buf), "%.3f\r\n", voltage);

      if (len > 0 &&
          HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, UART_TIMEOUT_MS) != HAL_OK)
      {
        uart_error_count++;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* visible error indicator */
      }
    }
    else
    {
      /* Skip the sample entirely: never transmit garbage. */
      adc_error_count++;
      if (++adc_consec_errors >= ADC_MAX_CONSEC_ERRORS)
      {
        ADC_Recover();
        adc_consec_errors = 0;
      }
    }

    /* Fixed-period scheduling: absolute deadlines avoid the drift of HAL_Delay(). */
    next_tick += SAMPLE_PERIOD_MS;
    if ((int32_t)(HAL_GetTick() - next_tick) > 0)
    {
      next_tick = HAL_GetTick();                      /* we fell behind: resynchronize */
    }
    while ((int32_t)(HAL_GetTick() - next_tick) < 0)
    {
      /* wait for the next sample instant */
    }
  }
  /* USER CODE END 3 */
```

**5. Helper functions** (in the `USER CODE BEGIN 4` region, below `main()`)

```c
/* USER CODE BEGIN 4 */

/* Take OVERSAMPLE_COUNT conversions and return their average. */
static HAL_StatusTypeDef ADC_ReadAveraged(uint16_t *result)
{
  uint32_t sum = 0;

  for (uint32_t i = 0; i < OVERSAMPLE_COUNT; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    sum += HAL_ADC_GetValue(&hadc1);
  }

  *result = (uint16_t)(sum / OVERSAMPLE_COUNT);
  return HAL_OK;
}

/* Full ADC reset: stop, de-initialize, re-initialize with CubeMX settings, recalibrate. */
static void ADC_Recover(void)
{
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_DeInit(&hadc1);
  MX_ADC1_Init();
  (void)HAL_ADCEx_Calibration_Start(&hadc1);
}

/* USER CODE END 4 */
```

### Code Walkthrough

| Element | Purpose |
|---|---|
| `HAL_ADCEx_Calibration_Start()` | Removes the ADC's internal offset error. Recommended on STM32F1 after each power-up. |
| `OVERSAMPLE_COUNT` averaging | Reduces random noise by about `√N` (8 samples ≈ 2.8×) at a cost of a few tens of microseconds. |
| `snprintf` with `%.3f` | Bounds-checked formatting and 1 mV text resolution. |
| Timeouts on ADC and UART | A broken peripheral can never freeze the main loop. |
| Skipping the transmit on ADC failure | The PC never receives garbage or stale values. |
| `next_tick` absolute deadlines | Removes cumulative drift; sample instants stay aligned to the 1 ms SysTick. |
| Resynchronization branch | After a long stall (for example, ADC recovery), the loop does not fire a burst of catch-up samples. |
| `adc_error_count`, `uart_error_count` | Add them to **Live Expressions** during a debug session to watch health metrics. |
| `ADC_Recover()` | Automatic self-healing after several consecutive ADC failures. |

### Output Format Options

| Goal | Format string / approach | Example line |
|---|---|---|
| Volts, 1 mV resolution (default) | `"%.3f\r\n"` | `1.650` |
| Raw ADC code | `"%u\r\n"` with `raw` | `2048` |
| Multiple traces in the Arduino Serial Plotter | `"%.3f,%u\r\n"` (comma-separated) | `1.650,2048` |
| Timestamped CSV | `"%lu,%.3f\r\n"` with `HAL_GetTick()` | `12340,1.650` |

<details>
<summary><strong>Integer-only variant (no float printf, saves Flash)</strong></summary>

Compute millivolts in integer math and print the decimal point manually. This works even with the float-printf option disabled:

```c
uint32_t mv  = ((uint32_t)raw * 3300U) / 4095U;          /* millivolts */
int      len = snprintf(uart_buf, sizeof(uart_buf), "%lu.%03lu\r\n",
                        (unsigned long)(mv / 1000U),
                        (unsigned long)(mv % 1000U));
```

Resolution is 1 mV, which is close to the ADC's 0.8 mV step.
</details>

---

## Build, Flash & Verify

### Build and Flash

1. Click **Project → Build Project** (or press `Ctrl+B`). The Console should end with `0 errors`.
2. Connect the Nucleo and choose **Run → Run** (or click the green *Run* button). On the first run, accept the default *STM32 C/C++ Application* launch configuration with the **ST-LINK (ST-LINK GDB server)** debug probe.
3. Wait for `Download verified successfully` in the Console. The LD1 LED flickers during the download.
4. The program starts automatically. If it does not, press the black **RESET (B2)** button on the board.

> Menu names vary slightly between CubeIDE versions, but the workflow is the same.

### Check the Serial Stream

Open a terminal at **115200 8N1** on the virtual COM port. You should see a stream of numbers scrolling at about 100 lines per second:

```text
1.650
1.651
1.649
1.650
```

Quick command-line checks:

```bash
# Linux
stty -F /dev/ttyACM0 115200 && cat /dev/ttyACM0

# macOS / Linux with screen (exit with Ctrl+A, then K)
screen /dev/ttyACM0 115200

# Any OS with Python
python -m serial.tools.miniterm <PORT> 115200
```

### Validation Checklist

Run these tests once to confirm the whole chain works and to learn the system's accuracy.

| # | Test | Expected result |
|---|---|---|
| 1 | Connect **A0 to GND** | ≈ 0.000 V (a few mV of offset is normal) |
| 2 | Connect **A0 to 3V3** | ≈ 3.300 V (raw ≈ 4095) |
| 3 | Sweep the potentiometer slowly | Smooth ramp from ≈ 0 V to ≈ 3.3 V with no jumps |
| 4 | Two equal 10 kΩ resistors in series between 3V3 and GND, midpoint to A0 | ≈ 1.65 V (raw ≈ 2048) |
| 5 | Compare against a multimeter at several voltages | Agreement within a few tens of mV; if not, see [Accuracy & Noise-Reduction Tips](#accuracy--noise-reduction-tips) |
| 6 | Hold a steady voltage and log for 30 s | Noise of only a few LSBs (a few mV) peak-to-peak |
| 7 | Leave A0 disconnected | Readings wander randomly. This is expected for a floating input, not a bug. |

---

## Host-Side Visualization

### Option 1: Arduino IDE Serial Plotter

1. Close any other program that has the COM port open.
2. In the Arduino IDE, choose **Tools → Port** and select the *STMicroelectronics STLink Virtual COM Port*. If the IDE insists on a board selection, choose any board (for example, Arduino Uno); the plotter only uses the serial port.
3. Open **Tools → Serial Plotter**.
4. Set the baud rate to **115200**.

Each line is plotted as one point, so a single number per line gives a single trace. Comma- or space-separated values on one line give multiple traces.

### Option 2: Python Live Plot with CSV Logging

<details>
<summary><strong>live_plot.py (click to expand)</strong></summary>

Install the dependencies first: `pip install pyserial matplotlib`

```python
#!/usr/bin/env python3
"""Live plot of voltage samples streamed by the STM32 Nucleo signal analyzer.

Usage:
    python live_plot.py COM5                       # Windows
    python live_plot.py /dev/ttyACM0 --csv log.csv # Linux / macOS, with logging
"""
import argparse
import collections
import csv
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial


def main():
    parser = argparse.ArgumentParser(description="Live plot for the STM32 signal analyzer")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=500, help="samples shown on screen")
    parser.add_argument("--period", type=float, default=10.0,
                        help="nominal sample period in ms (used to scale the x-axis)")
    parser.add_argument("--vmax", type=float, default=3.3, help="upper y-axis limit in volts")
    parser.add_argument("--csv", help="optional CSV file to log samples to")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    ser.reset_input_buffer()

    log_file = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(log_file) if log_file else None
    if writer:
        writer.writerow(["host_time_s", "voltage_v"])

    samples = collections.deque([0.0] * args.window, maxlen=args.window)
    t_axis = [i * args.period / 1000.0 for i in range(-args.window + 1, 1)]  # newest = 0 s
    rx_buf = bytearray()

    fig, ax = plt.subplots()
    (trace,) = ax.plot(t_axis, list(samples), lw=1.2)
    ax.set_xlim(t_axis[0], 0)
    ax.set_ylim(-0.05, args.vmax + 0.1)
    ax.set_xlabel("Time (s, nominal)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("STM32 Signal Analyzer")
    ax.grid(True, alpha=0.3)

    def update(_frame):
        nonlocal rx_buf
        rx_buf += ser.read(ser.in_waiting or 1)
        *lines, rx_buf = rx_buf.split(b"\n")      # last element is an incomplete line
        for raw in lines:
            try:
                volts = float(raw.decode("ascii", "ignore").strip())
            except ValueError:
                continue                           # ignore start-up garbage
            samples.append(volts)
            if writer:
                writer.writerow([f"{time.time():.3f}", f"{volts:.4f}"])
        trace.set_ydata(list(samples))
        return (trace,)

    ani = animation.FuncAnimation(fig, update, interval=30, blit=True,
                                  cache_frame_data=False)  # keep a reference to `ani`
    try:
        plt.show()
    finally:
        ser.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
```
</details>

The x-axis uses the *nominal* sample period, because USB buffering makes host-side arrival times uneven. If you need true timing, add `HAL_GetTick()` timestamps to the firmware output (see the format table above).

### Option 3: Other Tools

* **Terminal programs** (Tera Term, PuTTY, `screen`, `minicom`): raw text inspection and simple logging.
* **SerialPlot, Serial Studio, Teleplot (VS Code extension):** dedicated real-time plotting and dashboards; all handle simple text streams like this one.
* **Spreadsheets / data science:** log to a `.csv` file and analyze in Excel, LibreOffice, or Python (pandas).

---

## System Error Handling

The robust code includes specific mechanisms to gracefully handle hardware states:

* **ADC Timeout Handling:** Instead of using `HAL_MAX_DELAY` (which can freeze the entire MCU if the ADC peripheral crashes), we use a 10 ms timeout. If the conversion fails, the code skips the UART transmission, preventing garbage data from being sent to the PC. After several consecutive failures, the ADC is fully reset and recalibrated.
* **UART Congestion:** `HAL_UART_Transmit` is also given a strict timeout. If the transmit buffer gets clogged, the system will drop the packet rather than freezing the main loop, maintaining real-time behavior for the next sample.

### Failure Mode Summary

| Failure | How it is detected | Firmware response | What you should do |
|---|---|---|---|
| ADC conversion timeout | `HAL_ADC_PollForConversion()` returns `HAL_TIMEOUT` | Skip the sample, increment `adc_error_count` | Check clock configuration and wiring; inspect the counter in the debugger |
| Repeated ADC failures | 5 consecutive errors | `ADC_Recover()` resets and recalibrates the ADC | If it keeps happening, re-check the ADC clock (≤ 14 MHz) |
| UART transmit error | `HAL_UART_Transmit()` ≠ `HAL_OK` | Drop the packet, toggle **LD2**, increment `uart_error_count` | Verify USART2 settings and the ST-LINK connection |
| Peripheral initialization failure | `Error_Handler()` is called | Halts; with the snippet below, LD2 blinks rapidly | Re-run CubeMX, check the clock tree, rebuild |
| Input at or beyond the limits | `raw == 0` or `raw == 4095` | Not detected automatically; the ADC simply saturates | Reduce/scale the input. A pinned reading often means the signal is out of range. |
| Firmware hang | Watchdog timeout (optional) | Automatic MCU reset | Investigate the cause using the debugger |

> **Why UART errors are rare here.** The UART has no hardware flow control and the ST-LINK bridge accepts data whether or not a PC program has the port open. A blocking transmit therefore normally finishes in well under a millisecond. A failure usually points to a configuration problem (or `HAL_BUSY` if you later add interrupt- or DMA-based transmit).

### Visible Fatal-Error Indicator

Replace the body of the generated `Error_Handler()` so a fatal error is obvious on the board:

```c
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (volatile uint32_t i = 0; i < 1000000U; i++) { __NOP(); }   /* rapid blink */
  }
  /* USER CODE END Error_Handler_Debug */
}
```

### Optional: Independent Watchdog (IWDG)

A watchdog resets the MCU if the main loop ever stops running:

1. In CubeMX, enable **Timers → IWDG → Activated** with *Prescaler = 32* and *Reload = 1250* (≈ 1 s with the ~40 kHz LSI clock; LSI is imprecise, so treat the value as approximate).
2. Refresh the watchdog once per loop iteration, e.g. at the top of the `USER CODE BEGIN 3` block:

```c
HAL_IWDG_Refresh(&hiwdg);
```

3. While debugging, freeze it during breakpoints by adding `__HAL_DBGMCU_FREEZE_IWDG();` in `USER CODE BEGIN 2`, otherwise the MCU may reset every time you pause.

---

## Comprehensive Troubleshooting

### Quick Diagnostic Checklist ("Nothing on the plot")

Work through these in order and stop at the first failure:

1. **LD3** (red) lit? If not, check the USB cable and port.
2. Does the PC list a *STLink Virtual COM Port*? If not, try another cable (it must carry data), then install the driver and update the ST-LINK firmware.
3. Did the flash finish with `Download verified successfully`? If not, see problem 2 below.
4. Is any *other* program holding the port open? Close it.
5. Does a plain terminal at 115200 show text? If yes, the firmware is fine and the issue is in your plotter settings. If the terminal shows nothing, see problem 1.

### 1. Serial Plotter is completely blank
* **Cause:** The IDE is stripping out the `float` parsing library to save memory, resulting in empty string transmissions.
* **Fix:** Ensure the "Use float with printf from newlib-nano" setting is checked in the project properties (see the Code Implementation section). Rebuild and flash. Or use the integer-only variant.
* **Other causes to rule out:**
  * The wrong COM port is selected, or another program is holding it. Only one program can open the port at a time.
  * The plotter's baud rate is not 115200.
  * The firmware is not running. Press RESET (B2) and check whether LD1 flickers.
  * USART2 pins were changed in CubeMX. They must be PA2 (TX) and PA3 (RX) to reach the ST-LINK. The solder bridges `SB13`/`SB14` on the back of the board connect them by default; do not remove them.

### 2. "Target is not responding" during flash
* **Cause:** The currently running firmware disabled the SWD pins (PA13/PA14), locking out the programmer.
* **Fix (Hardware Reset Override):**
  1. Hold down the black **RESET (B2)** button on the board.
  2. Click "Run/Flash" in STM32CubeIDE.
  3. Release the RESET button the exact moment you see "Erasing..." or "Download in Progress" in the console.
  4. Ensure `SYS -> Debug` is set to `Serial Wire` in CubeMX so this doesn't happen again.
* **Also try:**
  * In *Debug Configurations → Debugger*, set **Reset behavior** to **Connect under reset**.
  * Confirm the two `CN3` jumpers and the `IDD` jumper are still fitted.
  * Update the ST-LINK firmware and try a different USB cable or port.

### 3. Build Error: `implicit declaration of function 'HAL_UART_Transmit'`
* **Cause:** USART2 was left disabled in CubeMX. A related symptom is `'huart2' undeclared`.
* **Fix:** Return to CubeMX, set USART2 mode to "Asynchronous", and regenerate the code. Accept the prompt to reload files in CubeIDE.
* **Similar errors:** `'hadc1' undeclared` means ADC1 is not enabled; `undefined reference to '_printf_float'` style problems mean float printf is not enabled.

### 4. Garbage characters in Serial Monitor (e.g., `��`)
* **Cause:** Baud rate mismatch between the STM32 and the host PC.
* **Fix:** Check your Serial Plotter/Monitor settings and ensure it is set exactly to **115200 baud**.
* **Also check:** a changed system clock in CubeMX (which shifts the actual baud rate), a non-8N1 frame setting in the terminal, or a terminal opened while the board was mid-transmission (the first line may be corrupted; that is harmless).

### 5. Voltage reads consistently incorrect or fluctuating wildly
* **Cause:** Floating ground.
* **Fix:** Ensure your external voltage source shares a common ground with the Nucleo board. Connect the negative terminal of your source to any `GND` pin on the Nucleo.
* **Other causes:**
  * **Disconnected input:** an unconnected A0 floats and produces random readings.
  * **Sampling time too short for the source impedance:** raise it to 71.5 or 239.5 cycles (readings look too low, drift, or depend on the previous channel or sample).
  * **Reference is not exactly 3.300 V:** measure the 3V3 pin with a multimeter and update `VREF_VOLTS`.
  * **Noise:** add the RC filter, keep wires short, and use averaging.

### 6. Readings stick at 0.000 V or 3.300 V
* **Cause:** The input is at or beyond the range limits, or the wiring is wrong.
* **Fix:** Measure the voltage at A0 with a multimeter. A sensor powered from 5 V may exceed 3.3 V (use a divider). If the value is pinned at 0, check that the potentiometer wiper is on A0 and not on an outer pin.

### 7. Plot is noisy or has random spikes
* **Fix:** Enable averaging (`OVERSAMPLE_COUNT`), add a 100 nF capacitor at A0, use a short twisted signal/ground pair, and route away from motors, relays, and PWM lines. See [Accuracy & Noise-Reduction Tips](#accuracy--noise-reduction-tips).

### 8. Sample rate is not what I expected
* **Cause:** In Version A, the period is `HAL_Delay(10)` plus the ADC, formatting, and UART time, so the real rate is slightly below 100 Hz.
* **Fix:** Use Version B's fixed-period scheduler, or move to timer-triggered sampling (see [Future Scope](#future-scope)).

### 9. Some lines are missing or the plot occasionally freezes
* **Cause:** Dropped packets after a UART timeout, a busy host, or a serial tool that stops reading while its window is minimized or being dragged.
* **Fix:** Check `uart_error_count` in Live Expressions. Keep the plotter window active and avoid running heavy tasks on the PC during capture.

### 10. The COM port disappears or changes number after flashing
* **Cause:** Re-flashing or resetting the ST-LINK re-enumerates the USB device.
* **Fix:** Close and reopen the serial viewer, and re-select the port if necessary.

### 11. The debugger cannot connect at all ("No ST-LINK detected")
* **Fix:** Try a different cable and port, reinstall the ST-LINK driver (**STSW-LINK009** on Windows), close any program that may already be using the ST-LINK (for example, another IDE instance or STM32CubeProgrammer), and update the ST-LINK firmware.

---

## Accuracy & Noise-Reduction Tips

| Technique | Effect | Effort |
|---|---|---|
| Measure the 3V3 rail and set `VREF_VOLTS` | Removes the largest systematic error (the nominal-reference assumption) | Very low |
| Long sampling time (71.5 or 239.5 cycles) | Correct readings from high-impedance sources | Very low (CubeMX) |
| ADC self-calibration | Removes internal offset error | Very low (already in Version B) |
| Averaging (`OVERSAMPLE_COUNT` = 8–16) | Random noise reduced by about `√N` | Low |
| 100 nF capacitor at A0 | Suppresses high-frequency pickup; provides a charge reservoir | Low |
| RC anti-alias filter (e.g., 4.7 kΩ + 1 µF ≈ 34 Hz) | Prevents aliasing; smooths noise | Low |
| Short, twisted signal/ground wires | Less interference | Low |
| Two-point software calibration | Removes gain and offset error | Medium |

### Two-Point Calibration

1. Feed two known voltages, `V_ref1` and `V_ref2` (for example, ≈ 0.5 V and ≈ 3.0 V measured with a good multimeter), and note the values `V_meas1` and `V_meas2` reported by the firmware.
2. Compute:

```text
gain   = (V_ref2 − V_ref1) / (V_meas2 − V_meas1)
offset = V_ref1 − gain × V_meas1
V_corrected = gain × V_meas + offset
```

3. Add the constants to the firmware and apply them before printing:

```c
#define CAL_GAIN    1.0000f    /* replace with your measured values */
#define CAL_OFFSET  0.0000f

float corrected = voltage * CAL_GAIN + CAL_OFFSET;
```

Even with calibration, remember that the on-chip 3.3 V supply drifts with temperature, load, and USB voltage, so the result is only as stable as VREF+.

---

## Future Scope

While the current software-polling method is excellent for low-frequency signals and debugging, it ties up CPU cycles and introduces jitter. Future roadmap enhancements include:

1. **Hardware Timers:** Replacing `HAL_Delay` with a hardware timer (e.g., TIM2/TIM3) to trigger ADC conversions at microsecond-perfect intervals.
2. **Direct Memory Access (DMA):** Utilizing the DMA controller to move ADC readings directly into memory arrays in the background, freeing the CPU entirely and enabling high-frequency audio or RF signal analysis.
3. **Biasing Circuitry:** Designing a 1.65 V DC offset shield to allow for AC signal (audio) measurement without damaging the 0–3.3 V limited ADC.

### Roadmap at a Glance

| Enhancement | Benefit | Difficulty |
|---|---|---|
| Timer-triggered ADC | Precise, jitter-free sample timing | Medium |
| DMA circular buffer | Zero CPU load; kHz–tens-of-kHz sampling | Medium |
| Binary framing + higher baud rate | 10× or more throughput | Medium |
| Multi-channel scan mode | Several traces at once | Medium |
| On-MCU statistics (min/max/mean/RMS/frequency) | Meter-like measurements | Medium |
| FFT spectrum display | Frequency-domain analysis | Hard |
| AC-coupling bias circuit | Audio and other AC signals | Easy (hardware) |
| Host GUI with triggering and cursors | Oscilloscope-like experience | Hard |

### Timer-Triggered Sampling with DMA (Outline)

**CubeMX changes**

* **TIM3:** Clock Source = *Internal Clock*; *Trigger Event Selection* = **Update Event**. Choose a prescaler and period to set the sample rate: `f_trigger = TIMCLK / ((PSC + 1) × (ARR + 1))`. Example for a 72 MHz timer clock: `PSC = 71`, `ARR = 999` → 1 kHz.
* **ADC1:** *External Trigger Conversion Source* = **Timer 3 Trigger Out event**; keep Continuous Conversion disabled.
* **ADC1 → DMA Settings:** add a request on `DMA1 Channel 1`, **Mode = Circular**, Data Width = **Half Word** (peripheral and memory), *Increment Address* on memory only.

**Firmware sketch**

```c
#define ADC_BUF_LEN 512U                          /* two halves of 256 samples */
static uint16_t          adc_buf[ADC_BUF_LEN];
static volatile uint8_t  half_ready = 0, full_ready = 0;

/* Startup (USER CODE BEGIN 2) */
HAL_ADCEx_Calibration_Start(&hadc1);
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
HAL_TIM_Base_Start(&htim3);

/* Callbacks (USER CODE BEGIN 4): keep them short, they run in interrupt context */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) { if (hadc->Instance == ADC1) half_ready = 1; }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)     { if (hadc->Instance == ADC1) full_ready = 1; }

/* Main loop */
if (half_ready) { half_ready = 0; process(&adc_buf[0],               ADC_BUF_LEN / 2); }
if (full_ready) { full_ready = 0; process(&adc_buf[ADC_BUF_LEN / 2], ADC_BUF_LEN / 2); }
```

`process()` is your function for filtering, statistics, or transmission. While one half of the buffer fills, you work on the other half, so no samples are lost.

### Binary Streaming Protocol

Text costs 7 bytes per sample. Raw 16-bit binary needs only 2 bytes and lets you detect losses:

| Field | Size | Description |
|---|---|---|
| Sync | 2 bytes | `0xAA 0x55` marks the start of a frame |
| Sequence | 1 byte | Rolling counter; gaps reveal dropped frames |
| Count *N* | 1 byte | Number of samples in the frame |
| Samples | 2 × *N* bytes | Little-endian raw 12-bit codes stored as `uint16_t` |
| Checksum | 1 byte | XOR of all preceding bytes |

Payload capacity: ≈ 5,760 samples/s at 115200 baud, and ≈ 46,000 samples/s at 921600 baud (the ST-LINK virtual COM port generally handles this rate, but confirm on your board). A binary stream needs a custom host decoder instead of a text plotter.

### AC-Coupled Input (1.65 V Bias)

To measure a signal centered on 0 V (audio, for example), superimpose it on a DC midpoint so it stays inside the ADC's 0–3.3 V window:

```text
                 3V3
                  │
                 ┌┴┐
                 │ │ R1 10 kΩ
                 └┬┘
 Vin ──┤├─────────┼──────────► PA0 (A0)
     C1 1–10 µF   │
                 ┌┴┐
                 │ │ R2 10 kΩ
                 └┬┘
                  │
                 GND
```

* The node sits at ≈ 1.65 V; the input swing must stay within about ±1.65 V (3.3 V peak-to-peak).
* The high-pass corner is `f_c = 1 / (2π · (R1‖R2) · C1)`. With 5 kΩ (R1‖R2) and 1 µF it is ≈ 32 Hz; with 10 µF it is ≈ 3.2 Hz.
* Add a 100 nF capacitor from the node to GND and consider a series resistor plus Schottky clamps for protection.
* Audio bandwidth needs a much higher sample rate, so combine this with the timer + DMA + binary streaming upgrades above.

### Other Ideas

* **Multi-channel capture:** enable scan mode on IN0–IN3 with DMA and print `v0,v1,v2,v3` for multi-trace plots in the Arduino Serial Plotter.
* **Signal statistics on the MCU:** min, max, mean, RMS, and zero-crossing frequency computed per block.
* **FFT:** use CMSIS-DSP fixed-point functions (for example, `arm_rfft_q15`); the Cortex-M3 has no FPU, so avoid floating-point FFTs at high rates.
* **Command interface:** use USART2 RX (PA3) to change the sample rate, channel, or output format at runtime.
* **Native USB:** the F103's USB peripheral can implement a CDC device for higher speed, but the Nucleo does not route it to a connector, so you would wire your own USB connector to PA11/PA12.
* **Triggering and cursors:** on the host side, add edge-trigger logic and measurement cursors for an oscilloscope feel.

---

## Appendix

### A. Quick Reference

| Item | Value |
|---|---|
| ADC input pin | PA0 (ADC1_IN0), header A0 |
| UART | USART2, PA2 (TX) / PA3 (RX), 115200 8N1 |
| Voltage from code | `V = raw / 4095 × VREF` |
| 1 LSB | `VREF / 4096 ≈ 0.806 mV` |
| Conversion time | `(sampling cycles + 12.5) / f_ADC` |
| Max ADC clock | 14 MHz |
| Nyquist frequency | `f_s / 2` |
| UART capacity at 115200 8N1 | 11,520 bytes/s |
| RC filter cutoff | `f_c = 1 / (2π R C)` |
| Divider output | `V_out = V_in × R2 / (R1 + R2)` |
| Reset button | Black **B2** |
| User LED / button | LD2 (PA5) / B1 (PC13) |

### B. Glossary

| Term | Meaning |
|---|---|
| **ADC** | Analog-to-Digital Converter; turns a voltage into a number |
| **LSB** | Least Significant Bit; the smallest voltage step the ADC can resolve |
| **VREF+ / VDDA** | The ADC's reference / analog supply voltage (3.3 V here) |
| **SAR** | Successive-approximation register; the ADC's conversion method |
| **HAL** | Hardware Abstraction Layer; ST's driver library |
| **VCP** | Virtual COM Port; the serial-over-USB bridge in the ST-LINK |
| **SWD** | Serial Wire Debug; the two-pin programming/debug interface (PA13/PA14) |
| **DMA** | Direct Memory Access; moves data without CPU involvement |
| **Nyquist frequency** | Half the sample rate; the highest frequency that can be represented |
| **Aliasing** | Signals above the Nyquist frequency masquerading as lower frequencies |
| **Source impedance** | The resistance the ADC "sees" looking back into the signal source |
| **IWDG** | Independent Watchdog; resets the MCU if not refreshed in time |

### C. Reference Documents

Search for these document numbers on [st.com](https://www.st.com) to find the current versions:

* **RM0008**: STM32F101/102/103/105/107 reference manual (ADC, UART, DMA, timers)
* **DS5319**: STM32F103x8/xB datasheet (electrical characteristics, ADC limits)
* **UM1724**: STM32 Nucleo-64 boards user manual (jumpers, connectors, ST-LINK)
* **UM1718**: STM32CubeMX user manual
* **UM1850**: Description of STM32F1 HAL and low-layer drivers
* **UM2609**: STM32CubeIDE user guide
