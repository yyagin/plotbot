#!/bin/bash

PACKAGE_NAME="jq"

if ! dpkg -s $PACKAGE_NAME >/dev/null 2>&1; then
  echo "$PACKAGE_NAME is not installed. Installing..."
  sudo apt-get install $PACKAGE_NAME
fi

jq -s 'map(.[])' ../../build/**/compile_commands.json > ../../compile_commands.json
echo "Compile Commands merged!"
