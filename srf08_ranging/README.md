# srf08 sonar sensor i2c address change linux command
## i2cset -y <bus no> <Device old address> <memory address> <value>
## i2cset -y 0 0x70 0x0 0xA0
## i2cset -y 0 0x70 0x0 0xAA
## i2cset -y 0 0x70 0x0 0xA5
## i2cset -y 0 0x70 0x0 0xE2  
## The last commad write the new address
