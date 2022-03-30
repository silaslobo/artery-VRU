import subprocess

directory = 'compiled'
#subprocess.call('cd compiled && asn1c -fcompound-names -fincludes-quoted -no-gen-example ../cpm.asn && cd ..')

subprocess.call('rm compiled/asn_* compiled/ber_* compiled/der_* compiled/per_* compiled/xer_* compiled/constr_*', shell=True)

fileToWrite = 'results.txt'
subprocess.call('rm ' + fileToWrite, shell=True)
subprocess.call('diff -r compiled ../../../extern/vanetza/vanetza/asn1/its | grep \'diff -r compiled\' > '
                       + fileToWrite, shell=True)

files = open(fileToWrite, "r").read().split('\n')

for i in range(len(files) - 1):
    fileToDelete = files[i].split()[2]
    print('rm ' + fileToDelete)
    subprocess.call('rm ' + fileToDelete, shell=True)

filesToAdd = 'filesToAdd.txt'
subprocess.call('ls ../cpm/compiled/*.c > ' + filesToAdd, shell=True)

with open(filesToAdd, 'r') as file :
  filedata = file.read()

# Replace the target string
filedata = filedata.replace('../cpm', '\tcpm')

# Write the file out again
with open(filesToAdd, 'w') as file:
  file.write(filedata)

