#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y python3 python3-tk python3-pip
python3 -m pip install --user -r "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/requirements.txt"

echo "Dependencies installed."
