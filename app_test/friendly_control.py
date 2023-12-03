import sys
import pygame

# import bluetooth
import subprocess
from serial import Serial


# def connect_to_device(device_mac):
#     # Search for the device with the specified MAC address
#     nearby_devices = bluetooth.discover_devices(lookup_names=True, lookup_class=True)

#     device_found = False
#     for addr, name, _ in nearby_devices:
#         if addr == device_mac:
#             device_found = True
#             break

#     if not device_found:
#         print(f"Device with MAC address {device_mac} not found.")
#         return None

#     # Connect to the Bluetooth device
#     port = 1  # RFCOMM port number
#     socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
#     socket.connect((device_mac, port))
#     subprocess.run(["sudo", "rfcomm", "bind", str(port), device_mac])

#     print(f"Connected to {name} ({device_mac})")
#     return socket


def open_serial_port(device_mac, port=1):
    # Open a serial communication port using the connected Bluetooth socket
    subprocess.run(["sudo", "rfcomm", "bind", str(port), device_mac])
    ser = Serial("/dev/rfcomm0", baudrate=115200)
    return ser


def main():
    """
    Define the robot control unit to run the program.
    """
    device_mac = "78:21:84:B9:0A:1E"
    # device_mac = "E0:5A:1B:E4:66:56"
    port = 0  # RFCOMM port number

    serial_port = open_serial_port(device_mac, port)

    # Initialize Pygame
    pygame.init()

    # Set up display
    width, height = 400, 300
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Robot control unit")

    # Set up variables
    forward_control = 0
    turn_control = 0
    battery_voltage = 0.0
    font = pygame.font.Font(None, 36)
    last_increment_time_forward = 0
    last_increment_time_turn = 0
    decrement_interval = 25  # milliseconds
    counter = 0

    # Main game loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                subprocess.run(["sudo", "rfcomm", "release", str(port)])
                serial_port.close()
                # Close the Bluetooth socket
                pygame.display.quit()
                sys.exit()

            # Check for key releases
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w or event.key == pygame.K_s:
                    print("Forward control reset to 0")
                if event.key in (pygame.K_a, pygame.K_d):
                    turn_control = 0
                    print("Turn control reset to 0")
        data = [forward_control, turn_control]
        if counter == 2:
            counter = 0
            print((",".join(map(str, data)) + "\n").encode())
            try:
                with serial_port as s:
                    s.write((",".join(map(str, data)) + "\n").encode())
                    battery_voltage = round(int(s.readline().decode()) * 2 / 1158, 2)
            except:
                pass
        counter += 1

        # Check for key presses
        keys = pygame.key.get_pressed()

        if not keys[pygame.K_w] and not keys[pygame.K_s]:
            if forward_control > 120:
                forward_control -= 11
            elif forward_control < -120:
                forward_control += 11
            if abs(forward_control) <= 120:
                forward_control = 0
        elif keys[pygame.K_w]:
            current_time = pygame.time.get_ticks()
            if current_time - last_increment_time_forward >= decrement_interval:
                forward_control = 255
                if forward_control < 120:
                    forward_control = 120
                forward_control = min(255, forward_control)
                print("Forward control:", forward_control)
                last_increment_time_forward = current_time
        # Check for key presses
        elif keys[pygame.K_s]:
            current_time = pygame.time.get_ticks()
            if current_time - last_increment_time_forward >= decrement_interval:
                forward_control = -255
                if forward_control > -120:
                    forward_control = -120
                forward_control = max(-255, forward_control)
                print("Forward control:", forward_control)
                last_increment_time_forward = current_time

        # Check for key presses
        if keys[pygame.K_d] and keys[pygame.K_a]:
            turn_control = turn_control
            print("Turn control:", turn_control)
        elif keys[pygame.K_d]:
            current_time = pygame.time.get_ticks()
            if current_time - last_increment_time_turn >= decrement_interval:
                turn_control += 5
                if turn_control < 255:
                    turn_control = 255
                turn_control = min(255, turn_control)
                print("Turn control:", turn_control)
                last_increment_time_turn = current_time
        elif keys[pygame.K_a]:
            current_time = pygame.time.get_ticks()
            if current_time - last_increment_time_turn >= decrement_interval:
                turn_control -= 5
                if turn_control > -255:
                    turn_control = -255
                turn_control = max(-255, turn_control)
                print("Turn control:", turn_control)
                last_increment_time_turn = current_time

        # Clear the screen
        screen.fill((255, 255, 255))

        # Display the controls on the screen
        text_forward = font.render(
            f"Forward control: {forward_control}", True, (0, 0, 0)
        )
        text_turn = font.render(f"Turn control: {turn_control}", True, (0, 0, 0))
        text_battery = font.render(f"Battery: {battery_voltage} V", True, (0, 0, 0))
        screen.blit(text_forward, (10, 10))
        screen.blit(text_turn, (10, 50))
        screen.blit(text_battery, (10, 90))

        # Update the display
        pygame.display.flip()

        # Control the frame rate
        pygame.time.Clock().tick(60)


if __name__ == "__main__":
    main()
