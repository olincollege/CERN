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
    device_mac = "78:21:84:B9:0A:1E" #stubby
    # device_mac = "E0:5A:1B:E4:66:56" #girthy
    # device_mac = "E0:5A:1B:E4:9D:D6" #kirby
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
    battery_voltage = 4.2
    font = pygame.font.Font(None, 36)
    last_increment_time_forward = 0
    last_increment_time_turn = 0
    decrement_interval = 25  # milliseconds
    r_screen = 255
    g_screen = 255
    b_screen = 255
    r_text = 0
    g_text = 0
    b_text = 0
    counter = 0
    led_code = ""

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
                if event.key == pygame.K_s or event.key == pygame.K_w:
                    print("Forward control reset to 0")
                if event.key in (pygame.K_a, pygame.K_d):
                    turn_control = 0
                    print("Turn control reset to 0")
        data = [forward_control, turn_control]
        if counter == 2:
            counter = 0
            try:
                with serial_port as s:
                    message = (",".join(map(str, data)) + led_code + "\n").encode()
                    print(message)
                    s.write(message)
                    led_code = ""
                    reading=s.readline().decode()
                    battery_voltage = round(float(reading), 2)
                    print(battery_voltage)
            except:
                pass
        counter += 1

        # Check for key presses
        keys = pygame.key.get_pressed()

        if keys[pygame.K_0]:
            led_code = ",0,0,0,0,1,0,0,0,0,1"
            r_screen = 0
            g_screen = 0
            b_screen = 0
            r_text = 255
            g_text = 255
            b_text = 255
        if keys[pygame.K_1]:
            led_code = ",255,0,0,1,1,0,0,255,1,1"
            r_screen = 255
            g_screen = 0
            b_screen = 0
            r_text = 0
            g_text = 0
            b_text = 255
        if keys[pygame.K_2]:
            led_code = ",255,255,255,1,0,255,255,255,1,0"
            r_screen = 255
            g_screen = 255
            b_screen = 255
            r_text = 0
            g_text = 0
            b_text = 0
        if keys[pygame.K_3]:
            led_code = ",255,255,0,1,0,255,255,0,1,0"
            r_screen = 255
            g_screen = 255
            b_screen = 0
            r_text = 0
            g_text = 0
            b_text = 255
        if keys[pygame.K_4]:
            led_code = ",255,0,255,1,0,255,0,255,1,0"
            r_screen = 255
            g_screen = 0
            b_screen = 255
            r_text = 0
            g_text = 255
            b_text = 0
        if keys[pygame.K_5]:
            led_code = ",0,255,255,1,0,0,255,255,1,0"
            r_screen = 0
            g_screen = 255
            b_screen = 255
            r_text = 255
            g_text = 0
            b_text = 0
        if keys[pygame.K_6]:
            led_code = ",255,0,0,1,0,255,0,0,1,0"
            r_screen = 255
            g_screen = 0
            b_screen = 0
            r_text = 0
            g_text = 255
            b_text = 255
        if keys[pygame.K_7]:
            led_code = ",0,255,0,1,0,0,255,0,1,0"
            r_screen = 0
            g_screen = 255
            b_screen = 0
            r_text = 255
            g_text = 0
            b_text = 255
        if keys[pygame.K_8]:
            led_code = ",0,0,255,1,0,0,0,255,1,0"
            r_screen = 0
            g_screen = 0
            b_screen = 255
            r_text = 255
            g_text = 255
            b_text = 0
        if keys[pygame.K_9]:
            led_code = ",255,255,255,4,20,255,255,255,4,20"
            r_screen = 255
            g_screen = 255
            b_screen = 255
            r_text = 0
            g_text = 0
            b_text = 0
        if keys[pygame.K_k] or float(battery_voltage)<3.1:
            led_code = ",shutdown"
            r_screen = 0
            g_screen = 0
            b_screen = 0
            r_text = 0
            g_text = 0
            b_text = 0

        if not keys[pygame.K_s] and not keys[pygame.K_w]:
            if forward_control > 120:
                forward_control -= 11
            elif forward_control < -120:
                forward_control += 11
            if abs(forward_control) <= 120:
                forward_control = 0
        elif keys[pygame.K_s]:
            current_time = pygame.time.get_ticks()
            if current_time - last_increment_time_forward >= decrement_interval:
                forward_control = 255
                if forward_control < 120:
                    forward_control = 120
                forward_control = min(255, forward_control)
                print("Forward control:", forward_control)
                last_increment_time_forward = current_time
        # Check for key presses
        elif keys[pygame.K_w]:
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
        screen.fill((r_screen, g_screen, b_screen))

        # Display the controls on the screen
        text_forward = font.render(
            f"Forward control: {forward_control}", True, (r_text, g_text, b_text)
        )
        text_turn = font.render(
            f"Turn control: {turn_control}", True, (r_text, g_text, b_text)
        )
        text_battery = font.render(
            f"Battery: {battery_voltage} V", True, (r_text, g_text, b_text)
        )
        screen.blit(text_forward, (10, 10))
        screen.blit(text_turn, (10, 50))
        screen.blit(text_battery, (10, 90))

        # Update the display
        pygame.display.flip()

        # Control the frame rate
        pygame.time.Clock().tick(60)


if __name__ == "__main__":
    main()
