#!/bin/bash

# Start the Python HTTP server
py -m http.server 8000 &  # The '&' runs the server in the background

# Wait for the server to start
sleep 2

# Open the default browser
# For Windows, you can use 'start', and for Linux/macOS, you can use 'xdg-open' or 'open' (macOS)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xdg-open "http://localhost:8000"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    open "http://localhost:8000"
elif [[ "$OSTYPE" == "cygwin" || "$OSTYPE" == "msys" ]]; then
    start "http://localhost:8000"
else
    echo "Unsupported OS"
fi
