#!/bin/bash
mkdir -p $1

if [ ! -z "$2" ]
then
    cp -r ${2}/* "${1}/"
fi

echo -n $1
