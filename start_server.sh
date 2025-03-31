#!/bin/bash

cd "$(dirname "$0")"  # Change to the directory of the script

# Start the Python HTTP server
py -m http.server 8000 &  # The '&' runs the server in the background

# Wait for the server to start
sleep 2

# Open the default browser
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xdg-open "http://localhost:8000"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    open "http://localhost:8000"
elif [[ "$OSTYPE" == "cygwin" || "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    start "" "http://localhost:8000"
else
    echo "Unsupported OS"
fi