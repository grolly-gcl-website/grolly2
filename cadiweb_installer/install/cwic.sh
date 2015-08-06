#!/bin/sh

# cadiweb installer creator script

tar cvzf cadiweb_latest.tar.gz cadiweb_installer/
# ftp pass Gyboq54Lqrw9GtOT

ftp -n ftp.gcl.engineering <<End-Of-Session
# -n option disables auto-logon

user gclengi "Gyboq54Lqrw9GtOT"
binary
cd public_html/files/cadiweb
put "cadiweb_latest.tar.gz"
bye
End-Of-Session
