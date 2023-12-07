import os
import subprocess
import serial

# Replace these values with your ESP32's Bluetooth COM port
bluetooth_port = "/dev/rfcomm1"

# Baud rate for communication with the ESP32
baud_rate = 115200


def flash_bootloader():
    print("Flashing ESP32 bootloader over Bluetooth...")

    # Path to the ESP32 bootloader binary
    bootloader_path = "./bootloader.bin"

    # Flashing the bootloader
    bootloader_command = [
        "esptool.py",
        "--chip",
        "esp32",
        "--port",
        bluetooth_port,
        "--baud",
        str(baud_rate),
        "write_flash",
        "--flash_mode",
        "dout",
        "--flash_freq",
        "40m",
        "--flash_size",
        "detect",
        "0xe000",
        bootloader_path,
    ]
    subprocess.run(bootloader_command, check=True)

    print("Bootloader flash completed.")


def compile_firmware(ino_path):
    print("Compiling Arduino code to firmware binary...")

    # Use the Arduino CLI to compile the sketch with dependencies
    compile_command = [
        "arduino-cli",
        "compile",
        "--fqbn",
        "esp32:esp32:esp32",
        ino_path,
    ]
    subprocess.run(compile_command, check=True)

    print("Compilation completed.")


def flash_esp32(firmware_path):
    print("Flashing ESP32 firmware over Bluetooth...")

    # Flashing the firmware
    firmware_command = [
        "esptool.py",
        "--chip",
        "esp32",
        "--port",
        bluetooth_port,
        "--baud",
        str(baud_rate),
        "write_flash",
        "--flash_mode",
        "dout",
        "0x1000",
        firmware_path,
    ]
    subprocess.run(firmware_command, check=True)

    print("Firmware flash completed.")


def monitor_serial():
    print("Monitoring serial port...")

    with serial.Serial(bluetooth_port, baud_rate, timeout=1) as ser:
        while True:
            line = ser.readline().decode("utf-8").strip()
            if line:
                print(line)


if __name__ == "__main__":
    # Specify the path to the Arduino sketch (.ino file)
    ino_path = "./test_binary/test_binary.ino"

    if not os.path.isfile(ino_path) or not ino_path.endswith(".ino"):
        print("Invalid or non-existent .ino file.")
        exit(1)

    try:
        # Flash the ESP32 bootloader
        flash_bootloader()

        # Compile the Arduino code
        compile_firmware(ino_path)

        # Generate the firmware binary path
        firmware_path = os.path.join(
            os.path.dirname(ino_path),
            "build",
            "esp32",
            os.path.basename(ino_path.replace(".ino", ".bin")),
        )

        # Flash the ESP32 firmware
        flash_esp32(firmware_path)

        # Monitor the serial port for output
        monitor_serial()

    except KeyboardInterrupt:
        print("Process interrupted by user.")
