#!/bin/bash
mv $1.ioc $2.ioc
mv "$1 Run.cfg" "$2 Run.cfg"

sed -i.bak -e "s/$1/$2/g" .mxproject
sed -i.bak -e "s/$1/$2/g" .cproject
sed -i.bak -e "s/$1/$2/g" .project
