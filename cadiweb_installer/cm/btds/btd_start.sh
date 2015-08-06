#!/bin/sh
sleep 10
hciconfig hci0 up
cd /var/www/html/cm/
php /var/www/html/cm/bt_daemon.php >> btds/btd_output &
