#!/bin/bash
# this script installs Cadiweb on Archlinux for Raspberry Pi
# Nginx is used as web-server daemon
 
echo 'Refreshing pacman databases'
pacman -Syu --noconfirm
 
echo 'Installing RPi watchdog (http://blog.ricardoarturocabral.com/2013/01/auto-reboot-hung-raspberry-pi-using-on.html)'
echo "bcm2708_wdog" | tee /etc/modules-load.d/bcm2708_wdog.conf
pacman -S --noconfirm watchdog
systemctl enable watchdog
echo 'Uncomment the line that starts with #watchdog-device'
echo 'Uncomment the line that says #max-load-1 = 24'
# nano /etc/watchdog.conf
systemctl start watchdog.service
echo 'END OF Watchdog install'
 
echo 'installing Expect for autoatic Bluetooth pairing with blutoothctl'
pacman -S --noconfirm expect

echo '... installing WGET'
pacman -S --noconfirm wget
 
cd /tmp/
mkdir cwi
cd cwi
 
echo "Downloading latest Cadiweb package"
wget http://gcl.engineering/files/cadiweb/cadiweb_latest.tar.gz
tar xvf cadiweb_latest.tar.gz
cd cadiweb_latest/
cp -rf * /srv/http/
 
 
echo '... Installing Nginx server'
pacman -S --noconfirm nginx
echo '... starting Nginx daemon'
systemctl start nginx
echo '... adding Nginx daemon into startup'
systemctl enable nginx
 
echo '... installing PHP and PHP-FPM'
pacman -S --noconfirm php php-fpm
echo '... adding PHP-FPM into systemctl startup'
systemctl enable php-fpm
echo 'replacing default Nginx config with Cadiweb default'
cp -f /srv/http/install/nginx.conf /etc/nginx/nginx.conf
 
 
echo '=== configuring bluetooth'
 
pacman -S --noconfirm bluez bluez-utils bluez-libs dbus
systemctl enable bluetooth
systemctl start bluetooth
 


sleep 10
cd /srv/http/cm/
php /srv/http/cm/bt_daemon.php >> btds/btd_output &
systemctl start httpd.service
systemctl start bluetooth
hciconfig hci0 up

cd /var/lib/bluetooth/
echo 'listing available adapters (hciconfig -a)'
hciconfig -a
HCIMAC=$(hciconfig | grep -o -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}')
cd $HCIMAC
echo -e 'adding PIN codes to /var/lib/bluetooth/'$HCIMAC
CADIMAC=$(hcitool scan | grep -o -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}')
echo 'adding '
echo -e $CADIMAC' 1234' | tee pincodes
echo 'pincode 1234 added for device '$CADIMAC

echo 'chmod for autopair.sh to execute it...'
chmod 777 /srv/http/install/autopair.sh

echo 'running bluetoothctl autopairing script...'
/srv/http/install/autopair.sh $CADIMAC

echo 'Adding date timezone to php.ini: Europe/Madrid'
 

 
echo 'changing permissions for Cadi BTDaemon (/srv/http/cm/bt_daemon.php)'
chmod 777 /srv/http/cm/bt_daemon.php
 
echo 'create directory /srv/http/cm (ArchLinux Raspberry typical)'
mkdir /srv/http/cm
echo 'creating directory /srv/http/cm/btds for Bluetooth Daemon data and settings'
mkdir /srv/http/cm/btds/
echo 'changing ownership to http for directory /srv/http'
chown http:http /srv/http
echo 'changing ownership to http for directory /srv/http/cm'
chown http:http /srv/http/cm
 
echo 'flushes file before downloading new settings from Cadi'
echo > /srv/http/cm/cadi_settings_dump
echo 'setting permissions for Cadi settings dump file'
chmod 777 /srv/http/cm/cadi_settings_dump
echo 'setting permissions for Cadi settings csv file'
chmod 777 /srv/http/cm/cadi_settings_conf.csv
 
# add BT daemon startup script call line into /etc/rc.local
echo 'create /etc/rc.local if not exist'
echo >> /etc/rc.local
grep -Fxq "/bin/sh /srv/http/cm/btds/btd_start.sh" /etc/rc.local
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
	sed -i -e '$i/bin/sh /srv/http/cm/btds/btd_start.sh\n /bin/sh /srv/http/cm/bt_restart_arch.sh' /etc/rc.local ;
	echo 'startup string added to /etc/rc.local';
else
	echo 'startup string already exists in /etc/rc.local' ;
fi
 
 
# Cadi BTDaemon startup script and systemd service
echo 'generating Cadi BTDaemon startup script'
echo -e '#!/bin/sh\nsleep 10\ncd /srv/http/cm/\nphp /srv/http/cm/bt_daemon.php >> btds/btd_output &\nhciconfig hci0 up' > /usr/lib/systemd/scripts/btd_start.sh
 
echo '... creating systemd service'
echo -e '[Unit]\n
Description=Cadi BTDaemon\n
\n
[Service]\n
Type=oneshot\n
ExecStart=/usr/lib/systemd/scripts/btd_start.sh\n
RemainAfterExit=yes\n
\n
[Install]\n
WantedBy=multi-user.target\n' > /usr/lib/systemd/system/cbtd.service


echo '... creating/flushing Cadi serial data dump log file (/srv/http/cm/serialresp.out)'
echo > /srv/http/cm/serialresp.out
 
echo '... changing ownership for Cadi serial data dump log file'
chown http:http /srv/http/cm/serialresp.out

 
echo '... enabling Cadi BTDaemon service @boot startup'
systemctl enable cbtd.service

echo '... starting Cadi BTDaemon service'
systemctl start cbtd.service
 
echo '... changing permissions for Cadi BTDaemon startup script (/srv/http/html/cm/btds/btd_start.sh)'
chmod 777 /srv/http/cm/btds/btd_start.sh
 
echo '... backing up cadi_settings file';
cp -rf /srv/http/cm/cadi_settings /tmp
 
echo '... changing permissions for directory /srv/http/cm/btds'
chmod 777 /srv/http/cm/btds
 
echo '... changing ownership for directory /srv/http/cm/btds'
chown http:http /srv/http/cm/btds
 
echo '... creating/flushing /srv/http/cm/daemon_cmd file (Cadi BT Daemon command log)'
echo > /srv/http/cm/daemon_cmd
 
echo '... changing ownership to http for /srv/http/cm/daemon_cmd'
chown http:http /srv/http/cm/daemon_cmd
 
echo '... changing permissions for file daemon_cmd'
chmod 777 /srv/http/cm/daemon_cmd
 
echo '... creating/flushing /srv/http/cm/btds/btd_output file (Cadi BTD output log)'
echo > /srv/http/cm/btds/btd_output
 
echo '... changing permissions for /srv/http/cm/btds/btd_output'
chmod 777 /srv/http/cm/btds/btd_output
 
echo '... creating/flushing Cadi status CSV file (/srv/http/cm/cadi_status.csv)'
echo > /srv/http/cm/cadi_status.csv
 
echo '... changing Cadi status CSV file ownership'
chown http:http /srv/http/cm/cadi_status.csv
 
echo '... changing Cadi status CSV file permissions'
chmod 777 /srv/http/cm/cadi_status.csv
chmod 777 /srv/http/cm/status_view_1.php
chmod 777 /srv/http/cm/status_view_2.php
 

 
echo '... changing permissions for Cadi serial data dump log file'
chmod 755 /srv/http/cm/serialresp.out
 
echo '... changing ownership for Cadi parent directory'
chown http:http /srv/http
 
echo '... changing permissions for Cadiweb parent directory (/srv/http/html)'
chmod 777 /srv/http
 
echo '... changing permissions for all files inside Cadiweb modules files (/srv/http/cm)'
cd /srv/http/cm
chmod 755 *
 
echo '... removing default Apache index.html page'
rm -rf /srv/http/index.html
 
 
 
echo 'creating default BTDaemon config file'

# hcitool -i hci0 info <bdaddr>
 
echo 'creating default SVG config file'
echo '12,100,728,193,9,50,586,393' > /srv/http/cm/svg.conf
chmod 777 /srv/http/cm/svg.conf
 
echo 'recovering backed up settings file'
cp -rf /tmp/cadi_settings /srv/http/cm/
chown http:http /srv/http/cm cadi_settings
 
chown -R http:http /srv/http/*
chmod 700 /usr/lib/systemd/scripts/btd_start.sh

chmod 700 /srv/http/cm/btds/btd_start_arch.sh

cp /srv/http/install/ccd_armv6 /srv/http/cm/


echo 'enabling Shared Memory module for PHP (copying installer php.ini into /etc/php/php.ini)'
cp /srv/http/install/php.ini /etc/php/php.ini

echo 'restarting PHP-FPM and Nginx services and bt_daemon.php helper'
systemctl restart php-fpm
systemctl restart nginx
systemctl start cbtd.service
echo '### IT IS RECOMMENDED TO RESTART RASPBERRY PI COMPUTER ###'
 
exit 0


