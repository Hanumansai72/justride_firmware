"""Administrator: generate a factory partition from an external registration JSON.

Input JSON must contain setup_code returned by device registration or setup-code
issuance. Keep input and output outside the repository. Flash the output only
to the matching board's jr_factory partition at 0x3E0000 (12288 bytes).
"""
import argparse
import csv
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--registration', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
source, output = args.registration.resolve(), args.output.resolve()
if source.is_relative_to(root) or output.is_relative_to(root):
    parser.error('Keep provisioning credentials outside the repository')
code = json.loads(source.read_text())['setup_code']
if not isinstance(code, str) or not re.fullmatch(r'[A-F0-9]{32}', code):
    parser.error('Invalid setup_code')
idf = os.environ.get('IDF_PATH')
if not idf:
    parser.error('Run from an activated ESP-IDF terminal')
generator = Path(idf) / 'components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py'
with tempfile.TemporaryDirectory() as temp:
    data = Path(temp) / 'factory.csv'
    with data.open('w', newline='') as stream:
        csv.writer(stream).writerows([
            ['key', 'type', 'encoding', 'value'],
            ['link', 'namespace', '', ''],
            ['setup', 'data', 'string', code],
        ])
    subprocess.run([sys.executable, str(generator), 'generate', str(data),
                    str(output), '0x3000'], check=True)
print('Factory image generated. Provision only the matching registered board.')
