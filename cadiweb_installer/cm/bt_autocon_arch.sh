#!/bin/sh

# usage example: './bt_autocon_arch.sh 00:11:22:33:44:55 0'

MAC=$1
RFNUM=$2
VERSION=$3

rm -rf /dev/cadi
rm -rf /dev/frcomm$RFNUM

#kill -9 $(pidof rfcomm)
RFTMP=$(rfcomm)
RFNUM_EX=${RFTMP:6:1}
echo '...RFNUM_EX=$RFNUM_EX'
echo '...releasing it'
rfcomm release $RFNUM_EX
echo '...hci0 down'
hciconfig hci0 down
echo '...stopping bluetoth'
systemctl stop bluetooth
echo '...rmmod btusb'
rmmod btusb
sleep 1
echo '...modprobe btusb'
modprobe btusb
echo '...starting bluetooth service'
systemctl start bluetooth
echo '...hci0 up'
hciconfig hci0 up

echo '...binding rfcomm${RFNUM} for ${MAC}'
rfcomm -r bind /dev/rfcomm$RFNUM $MAC 1
echo '... config for rfcomm stty 2'
stty -F /dev/rfcomm0 echo echoe echok raw
ln -s /dev/rfcomm$RFNUM /dev/cadi

echo '...streaming >>>>>>'

if [ $VERSION = 1 ]; then
	# start Cadi Communication Daemon
    /srv/http/cm/ccd > /dev/null &
else
	# streaming using linux cat for rfcomm port
    cat /dev/rfcomm$RFNUM > serialresp.out &
fi




LOGGER_PID=$!

echo 'streamer pid='$LOGGER_PID



echo 'bt_restart Archlinux edition executed'

