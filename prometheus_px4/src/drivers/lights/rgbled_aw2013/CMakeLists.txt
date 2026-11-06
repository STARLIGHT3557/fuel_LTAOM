px4_add_module(
        MODULE drivers__rgbled_aw2013
        MAIN rgbled_aw2013
        SRCS
        rgbled_aw2013.cpp
        DEPENDS
        led
        drivers__device
        )

