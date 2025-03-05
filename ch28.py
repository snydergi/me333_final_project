# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
ser = serial.Serial('/dev/ttyUSB0', 230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options
    print('\td: Read Encoder (deg) \tq: Quit')
    print('\tc: Read Encoder (counts) \te: Reset Encoder')
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection + '\n'

    # send the command to the PIC32
    ser.write(selection_endline.encode())  # .encode() turns the string into a char array

    if (selection == 'd'):  # Get Encoder Degrees
        n_str = ser.read_until(b'\n')
        print('Got back: ', float(n_str))
    elif (selection == 'c'):  # Get Encoder Count
        n_str = ser.read_until(b'\n')
        print('Got back: ', float(n_str))
    elif (selection == 'e'):  # Reset Encoder
        print('Encoder Reset')
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True  # exit client
        ser.close()  # close the serial port
    elif (selection == 'r'):  # Get Mode
        n_str = ser.read_until(b'\n')
        print('Got back: ', n_str[0:-1])  # remove the newline character while printing
    else:
        print('Invalid Selection ' + selection_endline)

# # example operation
# n_str = input('Enter number: ')  # get the number to send
# n_int = int(n_str)  # turn it into an int
# print('number = ' + str(n_int))  # print it to the screen to double check

# ser.write((str(n_int) + '\n').encode())  # send the number
# n_str = ser.read_until(b'\n')   # get the incremented number back
# n_int = int(n_str)  # turn it into an int
# print('Got back: ' + str(n_int) + '\n')  # print it to the screen
