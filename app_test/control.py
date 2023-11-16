import sys
import pygame
from serial import Serial


# Connect to the Bluetooth device

# Open a serial communication port
serial_port = Serial("/dev/rfcomm1", baudrate=115200)

# Initialize Pygame
pygame.init()

# Set up display
width, height = 400, 300
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Robot control unit")

# Set up variables
forward_control = 0
turn_control = 0
font = pygame.font.Font(None, 36)
last_increment_time_forward = 0
last_increment_time_turn = 0
decrement_interval = 25  # milliseconds
counter = 0

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            # Close the Bluetooth socket
            pygame.display.quit()
            sys.exit()

        # Check for key releases
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w or event.key == pygame.K_s:
                forward_control = 0
                print("Forward control reset to 0")
            if event.key in (pygame.K_a, pygame.K_d):
                turn_control = 0
                print("Turn control reset to 0")
    data = [forward_control, turn_control]
    if counter == 5:
        counter = 0
        print((",".join(map(str, data)) + "\n").encode())
        try:
            with serial_port as s:
                s.write((",".join(map(str, data)) + "\n").encode())
        except:
            pass
    counter += 1

    # Check for key presses
    keys = pygame.key.get_pressed()

    if keys[pygame.K_w] and keys[pygame.K_s]:
        forward_control = forward_control
        print("Forward control:", forward_control)
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
            if turn_control < 120:
                turn_control = 120
            turn_control = min(255, turn_control)
            print("Turn control:", turn_control)
            last_increment_time_turn = current_time
    elif keys[pygame.K_a]:
        current_time = pygame.time.get_ticks()
        if current_time - last_increment_time_turn >= decrement_interval:
            turn_control -= 5
            if turn_control > -120:
                turn_control = -120
            turn_control = max(-255, turn_control)
            print("Turn control:", turn_control)
            last_increment_time_turn = current_time

    # Clear the screen
    screen.fill((255, 255, 255))

    # Display the controls on the screen
    text_forward = font.render(f"Forward control: {forward_control}", True, (0, 0, 0))
    text_turn = font.render(f"Turn control: {turn_control}", True, (0, 0, 0))
    screen.blit(text_forward, (10, 10))
    screen.blit(text_turn, (10, 50))

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(60)
