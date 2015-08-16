#!/bin/bash
# this script "installs" Tor for Cadiweb on Arch Linux for Raspberry Pi system

echo 'installing Tor daemon'
pacman -S --noconfirm tor

mkdir /srv/http/tor
mkdir /srv/http/tor/hidden
chown tor:tor /srv/http/tor/hidden/
chmod 700 /srv/http/tor/hidden/
systemctl enable tor



# check if Hidden Service config line already exists
grep -Fxq "HiddenServicePort 80 127.0.0.1:80" /etc/tor/torrc
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
#	sudo sed -i -e '$i/bin/sh /var/www/cm/btds/btd_start.sh' /etc/rc.local ;
	awk '$0 ~ str{print NR-1 FS b}{b=$0}' str="This section is just for location-hidden services" /etc/tor/torrc
	LINENR=$?
	# use LINENR+9 to get desired line add position

	sed -i -e '70i\HiddenServicePort 80 127.0.0.1:80' /etc/tor/torrc
	sed -i -e '70i\HiddenServiceDir /srv/http/tor/hidden' /etc/tor/torrc

	echo 'Tor hidden service config lines added to /etc/tor/torrc';
else
	echo 'Tor Hidden service config string already exists in /etc/tor/torrc' ;
fi

systemctl start tor


exit 0
