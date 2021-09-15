# LPS22HH
Arduino library to support the LPS22HH 260-1260 hPa absolute digital output barometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LPS22HHSensor PressTemp(&dev_i2c);
    PressTemp.begin();
    PressTemp.Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LPS22HHSensor PressTemp(&dev_spi, CS_PIN);
    PressTemp.begin();
    PressTemp.Enable();

The access to the sensor values is done as explained below:  

  Read pressure and temperature.  

    float pressure;
    float temperature;
    PressTemp.GetPressure(&pressure);  
    PressTemp.GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/LPS22HH

The LPS22HH datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/lps22hh.html
