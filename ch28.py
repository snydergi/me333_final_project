import serial
import matplotlib.pyplot as plt
from statistics import mean
from genref import genRef


def read_plot_matrix():
    n_str = ser.read_until(b'\n')  # get the number of data points to receive
    n_int = int(n_str)  # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n')  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float, dat_str.split()))  # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref, data)
    meanlist = []
    for i, j in meanzip:
        meanlist.append(abs(i - j))
    score = mean(meanlist)
    t = range(len(ref))  # index array
    plt.plot(t, ref, 'r*-', t, data, 'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('index')
    plt.show()


def read_plot_position_matrix():
    n_str = ser.read_until(b'\n')  # get the number of data points to receive
    n_int = int(n_str)  # turn it into an int
    print('Data length = ' + str(n_int))
    # Lists to store the four values from each line
    refCur = []
    refAngle = []
    actualCur = []
    actualAng = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n')  # get the data as a string, ints separated by spaces
        dat_f = list(map(float, dat_str.split()))  # now the data is a list of floats
        # Ensure there are exactly four numbers in the line
        if len(dat_f) == 4:
            refCur.append(dat_f[0])
            refAngle.append(dat_f[1])
            actualCur.append(dat_f[2])
            actualAng.append(dat_f[3])
            data_received += 1
        else:
            print(f"Skipping malformed line: {dat_str}")
    # Plotting
    t = range(len(refCur))  # index array
    plt.plot(t, refCur, 'r*-', label='Reference Current')
    plt.plot(t, actualCur, 'b*-', label='Actual Current')
    plt.plot(t, refAngle, 'g*-', label='Reference Angle')
    plt.plot(t, actualAng, 'y*-', label='Actual Angle')
    plt.ylabel('value')
    plt.xlabel('index')
    plt.legend()
    plt.show()


ser = serial.Serial('/dev/ttyUSB0', 230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options
    print('\tCOMMAND OPTIONS:')
    print('\tEncoder:')
    print('\t  c: Read Encoder (counts)    d: Read Encoder (deg)')
    print('\t  e: Reset Encoder')
    print('\tCurrent Sensor:')
    print('\t  b: Read Current Sensor (mA) a: Read Current Sensor (ADC counts)')
    print('\tPWM Control:')
    print('\t  f: Set PWM (-100 to 100)    p: Unpower Motor')
    print('\tGains:')
    print('\t  i: Set Position Gains       j: Get Position Gains')
    print('\t  g: Set Current Gains        h: Get Current Gains')
    print('\tControl:')
    print('\t  k: Test Current Control     l: Go to Angle (deg)')
    print('\tTrajectory:')
    print('\t  m: Load Step Trajectory     n: Load Cubic Trajectory')
    print('\t  o: Execute Trajectory')
    print('\tMisc:')
    print('\t  r: Get Mode                 q: Quit')
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
    elif (selection == 'a'):  # read current sensor (ADC counts)
        n_str = ser.read_until(b'\n')
        print('Got back: ', float(n_str))
    elif (selection == 'b'):  # read current sensor (mA)
        n_str = ser.read_until(b'\n')
        print('Got back: ', float(n_str))
    elif (selection == 'p'):  # unpowered the motor
        print('Unpowered Motor.')
    elif (selection == 'f'):  # set PWM (-100 to 100)
        n_str = input('Enter PWM Frequency (-100 to 100): ')  # get the number to send
        n_int = int(n_str)  # turn it into an int
        ser.write((str(n_int) + '\n').encode())  # send the number
        print('Set PWM to: ' + (n_str))
    elif (selection == 'i'):  # set position gains
        n1_str = input('Enter kp_deg: ')  # get the number to send
        n1_flt = float(n1_str)  # turn it into an int
        n2_str = input('Enter ki_deg: ')  # get the second number
        n2_flt = float(n2_str)  # convert it to an int
        n3_str = input('Enter kd_deg: ')  # get the third number
        n3_flt = float(n3_str)  # convert it to an int
        combined_str = f"{n1_flt} {n2_flt} {n3_flt}\n"  # Combine the two numbers into a single string
        ser.write(combined_str.encode())  # send the number
        print(f'Set kp_deg to: {n1_flt}, ki_deg to: {n2_flt}, kd_deg to: {n3_flt}')
    elif (selection == 'j'):  # get position gains
        n_str = ser.read_until(b'\n')
        print('Got back kp_deg, ki_deg, kd_deg: ', n_str[0:-2])
    elif (selection == 'l'):  # go to angle (deg)
        n_str = input('Enter Angle: ')  # get the number to send
        n_int = int(n_str)  # turn it into an int
        ser.write((str(n_int) + '\n').encode())  # send the number
        read_plot_position_matrix()
    elif (selection == 'g'):  # set current gains
        n1_str = input('Enter kp_mA: ')  # get the number to send
        n1_flt = float(n1_str)  # turn it into an int
        n2_str = input('Enter ki_mA: ')  # get the second number
        n2_flt = float(n2_str)  # convert it to an int
        combined_str = f"{n1_flt} {n2_flt}\n"  # Combine the two numbers into a single string
        ser.write(combined_str.encode())  # send the number
        print(f'Set kp_mA to: {n1_flt} and ki_mA to: {n2_flt}')
    elif (selection == 'h'):  # get current gains
        n_str = ser.read_until(b'\n')
        print('Got back kp_mA, ki_mA: ', n_str[0:-2])
    elif (selection == 'k'):  # test current control
        read_plot_matrix()
    elif (selection == 'm'):  # load step trajectory
        print('')
    elif (selection == 'n'):  # load cubic trajectory
        ref = genRef('cubic')
        # print(len(ref))
        t = range(len(ref))
        plt.plot(t, ref, 'r*-')
        plt.ylabel('ange in degrees')
        plt.xlabel('index')
        plt.show()
        ser.write((str(len(ref)) + '\n').encode())
        for i in ref:
            ser.write((str(i) + '\n').encode())
    elif (selection == 'o'):  # execute trajectory
        read_plot_matrix()
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
