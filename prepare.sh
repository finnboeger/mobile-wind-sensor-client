#!/bin/bash

# check for python 3.11 or higher
if ! printf '%s\n' "3.11" "$(python3 -V | awk '{print $2}')" | sort -C -V; then
    echo "Python 3.11 or higher is required."
    exit 1
fi

# create virtual environment
echo "Creating virtual environment..."
python3 -m venv .venv

# install dependencies
source .venv/bin/activate
pip3 install --upgrade pip
pip3 install -r requirements.dev.txt

# setup git hooks
pre-commit install
