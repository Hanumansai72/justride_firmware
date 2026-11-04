"""Verify a signed release and prove tampered images and another signing key fail.

Run using ESP-IDF Python. All temporary private keys stay outside the repository.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--key',type=Path,required=True)
parser.add_argument('--image',type=Path,default=Path('build-s3/signed/justride_firmware.bin'))
args=parser.parse_args()
def verify(key,image,expected):
    result=subprocess.run([sys.executable,'-m','espsecure','verify_signature','--version','2',
        '--keyfile',str(key),str(image)],capture_output=True,text=True)
    assert (result.returncode==0)==expected,result.stdout+result.stderr

verify(args.key,args.image,True)
with tempfile.TemporaryDirectory(prefix='justride-signature-') as directory:
    temporary=Path(directory)
    wrong_key=temporary/'wrong.pem'
    subprocess.run([sys.executable,'-m','espsecure','generate_signing_key','--version','2',str(wrong_key)],
        check=True,capture_output=True)
    verify(wrong_key,args.image,False)
    corrupted=bytearray(args.image.read_bytes());corrupted[64]^=1
    bad_image=temporary/'corrupted.bin';bad_image.write_bytes(corrupted)
    verify(args.key,bad_image,False)
    bad_image.write_bytes(args.image.read_bytes()[:-100])
    verify(args.key,bad_image,False)
print('Signed image accepted; wrong key, tampering, and truncation rejected by native ESP-IDF tooling.')
