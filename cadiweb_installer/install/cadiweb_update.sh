#!/bin/sh

# this script updates your Cadiweb, using Grolly Git repository


echo 'CU> This is Cadiweb Update script'


# flush cadiweb temporary folder if exists
rm -rf /tmp/cadiwebtmp

#recreate cadiweb temp folder
mkdir /tmp/cadiwebtmp

#enter cadiweb temp folder
cd /tmp/cadiwebtmp

# backup Cadiweb config file
echo 'CU> backing up Cadiweb config file'
cp /srv/http/cm/cadi_settings /tmp/cadiwebtmp/cadi_settings

#backup Grolly eeprom setttings structure file
cp /srv/http/cm/cadi_settings_conf.csv /tmp/cadiwebtmp/cadi_settings_conf.csv

# backup Grolly eeprom dump
cp /srv/http/cm/cadi_settings_dump /tmp/cadiwebtmp/cadi_settings_dump




# install Git (--needed to skip reinstall if already installed)
echo 'CU> Installing Git'
pacman -S --needed --noconfirm git

# clone Grolly repository locally
echo 'CU> cloning Grolly 2 repository locally'
git clone https://github.com/grolly-gcl-website/grolly2.git repo

# enter Cadiweb installer directory
cd /tmp/cadiwebtmp/repo/cadiweb_installer

# copy cadiweb modules
echo 'CU> ...copying Cadiweb modules...'
cp -rf cm/* /srv/http/cm/

# javascript 
echo 'CU> ...some JavaScript...'
cp -rf js/* /srv/http/js/

# stylesheets
echo 'CU> ...stylesheets...'
cp -rf css/* /srv/http/css/

# includes
echo 'CU> ...includes...'
cp -rf includes/* /srv/http/includes/

# resources
echo 'CU> ...resources (eg SVG)...'
cp -rf res/* /srv/http/res/

# installation files and stuff
echo 'CU> ...installation routines and stuff...'
cp -rf install/* /srv/http/install/

# index page
echo 'CU> ...index.php...'
cp -rf index.php /srv/http/index.php

# restore config backup
echo 'CU> restoring config backup'
cp -rf /tmp/cadiwebtmp/cadi_settings /srv/http/cm/cadi_settings

# recover Grolly eeprom setttings structure file
cp -rf /tmp/cadiwebtmp/cadi_settings_conf.csv /srv/http/cm/cadi_settings_conf.csv

# recover Grolly eeprom dump
cp -rf /tmp/cadiwebtmp/cadi_settings_dump /srv/http/cm/cadi_settings_dump

#change permissions for daemon_cmd (autoconnect fix)
chmod 777 /srv/http/cm/daemon_cmd


if [ "$1" == "forceccd" ]; then
	echo 'CU> stopping CBTD service'
	systemctl stop cbtd
	echo 'CU> compiling CCD'
	gcc -lrt /srv/http/install/ccd.c -o /srv/http/install/ccd
	systemctl start cbtd
	
else
	echo 'skipping CCD update'

fi



