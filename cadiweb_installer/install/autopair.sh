#!/bin/expect -f

#### Sample bluetoothctl interaction log

# [bluetooth]# power on
# Changing power on succeeded
# [bluetooth]# agent on
# Agent registered
# [bluetooth]# default-agent
# Default agent request successful
# [bluetooth]# remove $mac
# [DEL] Device 20:13:06:19:15:11 HC-06
# Device has been removed
# [bluetooth]# scan on
# Discovery started
# [CHG] Controller 00:1A:7D:DA:71:11 Discovering: yes
# [NEW] Device 20:13:06:19:15:11 HC-06
# [bluetooth]# pair 20:13:06:19:15:11
# Attempting to pair with 20:13:06:19:15:11
# [CHG] Device 20:13:06:19:15:11 Connected: yes
# Request PIN code
# [agent] Enter PIN code: 1234
# [CHG] Device 20:13:06:19:15:11 UUIDs: 00001101-0000-1000-8000-00805f9b34fb
# [CHG] Device 20:13:06:19:15:11 Paired: yes
# Pairing successful

set mac [lindex $argv 0];
set timeout 10
set attmpts 10

spawn "bluetoothctl"

while {$attmpts > 0} {

# 20:13:06:19:15:11
send "power on\r"
expect "succeeded"
send "agent on\r"
expect "registered"
send "default-agent\r"
expect "Default"

	send "remove $mac\r"
	expect {
	    "Device $mac" {
	        send "\r"
	    }
	    "available" {
	        send "\r"
	    }
	}
#	expect $

	send "scan on\r"
	sleep 7

	expect {
		"Device $mac" {
			puts "EXPECT FOUND DEVICE\n"
			# pair device found
			send "pair $mac\r"
			set timeout 30
			expect { 
				"Enter PIN code" {
					puts "EXPECT ENTERING PINCODE\n"
					send "1234\r"
					expect {
						"Pairing successful" {
							set attmpts 0
						}
					}
				}
				"Failed" {
					send "scan off\r"
					puts "EXPECT FAILED PAIRING"
					set attmpts [expr $attmpts-1];
					send "agent off\r"
					send "power off\r"
					expect "Changing power off succeeded"
					sleep 15;
					if {$attmpts<2} {
						exit 99;
					}
				}
				"Device $mac not available" {
					send "scan off\r"
					puts "EXPECT FAILED PAIRING"
					set attmpts [expr $attmpts-1];
					send "agent off\r"
					send "power off\r"
					expect "Changing power off succeeded"
					sleep 15;
					if {$attmpts<2} {
						exit 99;
					}
				}
			
		     	}
			set timeout 10
		}
#		"Device $mac" {
#			puts "SINGLE STRING SCAN\r"
#		}
		"Failed" {
			send "scan off\r"
                        puts "EXPECT FAILED PAIRING"
                        set attmpts [expr $attmpts-1];
                        send "agent off\r"
                        send "power off\r"
                        expect "Changing power off succeeded"
			sleep 5
		}
		"Error" {
			send "scan off\r"
                        puts "EXPECT FAILED PAIRING"
                        set attmpts [expr $attmpts-1];
                        send "agent off\r"
                        send "power off\r"
                        expect "Changing power off succeeded"
			sleep 5
		}
	}
}



# Now exit the bluetoothctl
send "exit\r"
expect $


exit 0
