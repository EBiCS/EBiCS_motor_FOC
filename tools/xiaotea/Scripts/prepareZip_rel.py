#!/usr/bin/python
from sys import argv, exit
from os.path import getsize
from xiaotea import XiaoTea
import zipfile
import hashlib
import time
import shutil
import os


cry = XiaoTea()
source='..\debug\SmartESC_v3.bin'
destination='FIRM.bin'
shutil.copyfile(source, destination)


hfi = open('FIRM.bin', 'rb')
hfo = open('FIRM.bin.enc', 'wb')

hfo.write(cry.encrypt(hfi.read()))

hfo.close()
hfi.close()

md5_hash = hashlib.md5()
with open('FIRM.bin',"rb") as f:
    # Read and update hash in chunks of 4K
    for byte_block in iter(lambda: f.read(4096),b""):
        md5_hash.update(byte_block)
    print(md5_hash.hexdigest())

md5_hash_enc = hashlib.md5()
with open('FIRM.bin.enc',"rb") as f:
    # Read and update hash in chunks of 4K
    for byte_block in iter(lambda: f.read(4096),b""):
        md5_hash_enc.update(byte_block)
    print(md5_hash_enc.hexdigest())

version = 'zip_output\DRV138'
filename = version + '-' + str(int(time.time())) + '.zip'
zip_file = zipfile.ZipFile(filename, 'a', zipfile.ZIP_DEFLATED, False)

zip_file.write('FIRM.bin')

zip_file.write('FIRM.bin.enc')


info_txt = 'dev: M365;\nnam: {};\nenc: B;\ntyp: DRV;\nmd5: {};\nmd5e: {};\n'.format(
    version, md5_hash.hexdigest(), md5_hash_enc.hexdigest())

zip_file.writestr('info.txt', info_txt.encode())
zip_file.close()
os.remove('FIRM.bin')
os.remove('FIRM.bin.enc')
