=== Grolly Watering Robot firmware ===

This is a project made using CooCox IDE (v1.7.8)
GCC arm-none-eabi toolchain used for compilation

Compilation flags (Check View-Configuration in CooCox):
-DUSE_LCD - this flag enables the support of 1602 (HD44780) series LCD displays with I2C expander (default is DFROBOT one, but could be used MJKDZ instead, see MJKDZ defines in the souce code)

-DGROLLYSN001 - this flag overrides the default valves pinouts to the ones, used in Grolly 2 testing device, having Serial No. 2G001. Without this flag, the pinouts are used as for 2G002 and up.
