def GetPosition(self):
    import busio
    import math
    import board

    from board import SDA, SCL

    i2c0 = busio.I2C(SCL, SDA) # define I2C object
    #D = ????? will be the one byte of data we recieve in the form of aninterger
    #-9 through 9
    #D represents the delay between the hydrophones

    #calculate 2D position angle theta
    c = 1480 #speed of waves in water
    Fs = 100000 #ADC sample rate
    L = .138 #length between hydrophones

    x = (d*c)/(Fs*L)
    theta = math.acos(x)
    return(theta)
