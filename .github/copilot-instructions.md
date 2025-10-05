This repository supports the LilyGo T-Display-S3 family (ESP32-S3) and contains multiple Arduino/PlatformIO examples, board data, and helper libraries.

Quick orientation (what matters most)
- Root-level PlatformIO project. The active example is controlled by `platformio.ini: src_dir = examples/${platformio.default_envs}`.
- Board description files live in `boards/` (see `boards/lilygo-t-displays3.json`).
- Display and touch configuration is centralized in the included `TFT_eSPI` library under `lib/TFT_eSPI/` and its `User_Setups` files. Many examples require `User_Setups/Setup206_LilyGo_T_Display_S3.h` to be selected via `User_Setup_Select.h`.
- Examples are full sketches under `examples/` (each example is an independent sketch folder). Default env in `platformio.ini` selects which example is built.

Build, upload and debug (explicit commands)
- Recommended: use PlatformIO (VSCode PlatformIO extension). The README contains step-by-step PlatformIO setup.
- To build a single example via CLI (PlatformIO):
  - pio run -e <ENV>
  - Example: `pio run -e PCBClock` (runs build for `examples/PCBClock` since env is named `PCBClock`)
- To upload via USB (PlatformIO):
  - pio run -e <ENV> -t upload
  - Example: `pio run -e PCBClock -t upload`
- OTA example uses `upload_protocol = espota` and requires `upload_port` set to the board IP; see `platformio.ini` [env:ota].
- Debugging: `debug_tool = esp-builtin` is configured in `platformio.ini`. On-board USB-JTAG is used when enabled.

Project conventions and gotchas (important for edits)
- Only one `default_envs` line should be uncommented in `platformio.ini`; the project sets `src_dir` to use the selected example. When making edits, note this influences which example is compiled.
- Many examples contain preprocessor checks and #error messages requiring `TFT_eSPI/User_Setup_Select.h` to include `Setup206_LilyGo_T_Display_S3.h`. Do not alter examples to bypass this; instead ensure the library config is correct.
- USB serial vs battery mode: the project toggles printing behavior via build flags `-DARDUINO_USB_CDC_ON_BOOT=1` or `-UARDUINO_USB_CDC_ON_BOOT`. Changing these affects where Serial prints appear (USB-C vs IO43/IO44) and boot blocking behavior. See README and `platformio.ini` build_flags.
- Library isolation: The repo includes local copies of several libraries under `lib/` (for example `TFT_eSPI`, `TouchLib`, `lvgl`). Editing these libraries changes behavior for all examples that import them. Prefer keeping changes minimal and documented.

Where to look for common tasks and examples
- Pin/board definitions: `boards/lilygo-t-displays3.json` and `lib/TFT_eSPI/User_Setups/Setup206_LilyGo_T_Display_S3.h`.
- Example sketches: `examples/<ExampleName>/<ExampleName>.ino`. Examples often create a TFT_eSPI instance with `TFT_eSPI tft = TFT_eSPI();` and assume `User_Setup_Select.h` points to the S3 setup.
- Firmware images: `firmware/` contains prebuilt binaries useful for quick verification.
- Docs and CI: top-level `README.md` contains setup notes and FAQs; GitHub Actions workflow `/.github/workflows/platformio.yml` is configured to run builds per-example.

Editing rules for AI agents
- If you change board or pin mappings, update `lib/TFT_eSPI/User_Setups/Setup206_LilyGo_T_Display_S3.h` and add a short note in `README.md` explaining the change.
- Do not assume external library versions — the project pins PlatformIO platform `espressif32@6.5.0` in `platformio.ini`. If you propose updating platform or library versions, include a compatibility note and test at least one example.
- Prefer non-invasive fixes: update a single example or library file and run a build for the corresponding env (see `platformio.ini` env name). Mark any breaking changes explicitly in commit messages.

Examples to copy/paste (PlatformIO)
- Build the PCBClock example: `pio run -e PCBClock`
- Build and upload PCBClock via USB: `pio run -e PCBClock -t upload`
- Build the OTA example: `pio run -e ota -t upload` (ensure `upload_port` in `platformio.ini` is the device IP)

Limitations — what the repo does not document
- There is no centralized test harness — examples are intended to be flashed and run on hardware.
- Hardware-specific boot-mode instructions are in `README.md` (manual BOOT/RST sequence). Automated serial flash scripts are not provided.

If anything in these instructions is unclear, tell me which workflows you want expanded (e.g., automated CI per-example, how to run a specific example locally, or editing the TFT_eSPI configuration) and I will update this file.
