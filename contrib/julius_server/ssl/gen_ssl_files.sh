#!/bin/bash
/usr/bin/openssl genrsa -des3 -out server.orig.key 2048
/usr/bin/openssl rsa -in server.orig.key -out server.key
/usr/bin/openssl req -new -key server.key -out server.csr
/usr/bin/openssl x509 -req -days 365 -in server.csr -signkey server.key -out server.crt
