#!/bin/bash
 
echo -e "...creating cmdline.txt file"
sudo echo -e "root=/dev/sda2 rw rootwait console=ttyAMA0,115200 console=tty1 selinux=0 plymouth.enable=0 smsc95xx.turbo_mode=N dwc_otg.lpm_enable=0 kgdboc=ttyAMA0,115200 elevator=noop fsck.mode=force" > cmdline.txt
 
echo -e "...creating fstab file"
sudo echo -e "#" > fstab 
sudo echo -e "# /etc/fstab: static file system information" >> fstab
sudo echo -e "#" >> fstab
sudo echo -e "# <file system> <dir>   <type>  <options>       <dump>  <pass>" >> fstab
sudo echo -e "/dev/mmcblk0p1  /boot   vfat    defaults,ro        0       0" >> fstab
 
 
sudo umount boot root
sudo rm -rf boot/
sudo rm -rf root/
 
echo -e "\n\n\n\n\n\n\n\n=== GCL Engineering tools ===\n"
echo -e "** ArchLinux for RaspberryPi installer\n"
echo -e "** Supposed, you have inserted USB flash stick and microSD\n"
echo -e "** card into this PC\n"
echo -e "If not, stop the script pressing CTRL+C\n"
 
echo -e ">>> Installing BSDTAR if needed'n"
sudo apt-get -q -y install bsdtar
 
echo -e "\n*********\nHere are the disks available\n***********************\n"
 
sudo fdisk -l


echo -e "Which Raspberry you have? (v1 or v2? Enter '1' or '2'):\n"
read rpiv


echo -e "Enter the drive letter for installing /BOOT partition"
echo -e "\n of ArchLinux for Raspberry Pi (like 'b') : \n"
read drive_letter_boot
 
echo -e "and drive letter for /ROOT partition"
echo -e "\n of ArchLinux for Raspberry Pi (like 'c') : \n"
read drive_letter_root
 
 
echo -e "\n\nDrive letter for /boot is: "
echo $drive_letter_boot
echo -e "\n Drive letter for /root is: "
echo $drive_letter_root
echo -e '\n\n\n\n'
 
echo -e "unmounting partitions to be reformatted\n"
sudo umount /dev/sd"$drive_letter_boot"1
sudo umount /dev/sd"$drive_letter_root"1
sudo umount /dev/sd"$drive_letter_root"2
 
echo -e "o\np\nn\np\n1\n\n+100M\nt\nc\nw" | fdisk /dev/sd$drive_letter_boot
sudo mkfs.vfat /dev/sd"$drive_letter_boot"1
sudo mkdir boot_sd
sudo chown root:root boot_sd
sudo chmod 700 boot_sd
sudo mount /dev/sd"$drive_letter_boot"1 boot_sd
 
 
echo -e "o\np\nn\np\n1\n\n+100M\nt\nc\nn\np\n2\n\n\nw" | fdisk /dev/sd$drive_letter_root
sudo mkfs.vfat /dev/sd"$drive_letter_root"1
sudo mkdir boot
sudo chown root:root boot
sudo chmod 700 boot
sudo mount /dev/sd"$drive_letter_root"1 boot
 
sudo mkfs.ext4 /dev/sd"$drive_letter_root"2
sudo mkdir root
sudo chown root:root root
sudo chmod 700 root
sudo mount /dev/sd"$drive_letter_root"2 root
sudo rm -rf root/srv/http/*
if [ $rpiv -eq "1" ]; then
  sudo wget -c http://archlinuxarm.org/os/ArchLinuxARM-rpi-latest.tar.gz
fi

if [ $rpiv -eq "2" ]; then
  sudo wget -c http://archlinuxarm.org/os/ArchLinuxARM-rpi-2-latest.tar.gz
fi


if [ $rpiv -eq "1" ]; then
  sudo bsdtar -vxpf ArchLinuxARM-rpi-latest.tar.gz -C root
fi


if [ $rpiv -eq "2" ]; then
  sudo bsdtar -vxpf ArchLinuxARM-rpi-2-latest.tar.gz -C root
fi

 
echo -e "\n...\n ...copying fstab to /root partition\n"
sudo rm -rf root/etc/fstab
sudo cp fstab root/etc/fstab
 
echo -e "...copying files to USB /boot partition\n"
sudo cp -rf root/boot/* boot
echo -e "...copying files to microSD /boot partition \n"
sudo mv root/boot/* boot_sd
echo -e "...copying cmdline.txt to microSD /boot partition\n"
sudo cp -f cmdline.txt boot_sd/


echo -e "...syncing\n"
sudo sync

echo -e "...unmounting partitions\n"
sudo umount root boot boot_sd
 
echo -e "Finished creating file system on /dev/sd"$drive_letter_root
echo -e "\n ... and /dev/sd"$drive_letter_boot
echo -e "=== COMPLETE! ==="


