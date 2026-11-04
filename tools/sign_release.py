"""Sign an ESP-IDF software-verified OTA release using an external RSA-3072 key.

Run with the ESP-IDF Python environment after building with sdkconfig.defaults.signed.
The initial signed application must be installed over trusted USB, not an untrusted OTA.
"""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--key', type=Path, required=True)
parser.add_argument('--build', type=Path, default=Path('build-s3'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
key = args.key.resolve()
if key.is_relative_to(root) or not key.is_file():
    parser.error('The signing key must exist outside the source repository.')
build = args.build.resolve()
config = (build / 'config/sdkconfig.h').read_text()
for flag in ('CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT', 'CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT'):
    if f'#define {flag} 1' not in config:
        parser.error(f'Missing {flag}; build with sdkconfig.defaults.signed first.')
if '#define CONFIG_SECURE_BOOT 1' in config:
    parser.error('This workflow deliberately does not provision hardware secure boot.')
source = build / 'justride_firmware.bin'
output = build / 'signed/justride_firmware.bin'
output.parent.mkdir(exist_ok=True)
subprocess.run([sys.executable, '-m', 'espsecure', 'sign_data', '--version', '2',
                '--keyfile', str(key), '--output', str(output), str(source)], check=True)
subprocess.run([sys.executable, '-m', 'espsecure', 'verify_signature', '--version', '2',
                '--keyfile', str(key), str(output)], check=True)
print(f'Signed and verified: {output}')
