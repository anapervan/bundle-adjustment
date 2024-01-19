import os
import bz2
import urllib.request

# Download data
project_url = "http://grail.cs.washington.edu/projects/bal/data/ladybug/"
# dataset_name = "problem-73-11032-pre.txt.bz2"  # "problem-49-7776-pre.txt.bz2"
dataset_name = "problem-49-7776-pre.txt.bz2"
decompressed_dataset_name = dataset_name.replace('.bz2', '')
full_url = project_url + dataset_name

# If the file isn't already saved, download it
if not os.path.isfile(decompressed_dataset_name):
    urllib.request.urlretrieve(full_url, dataset_name)

    # Decompress the .bz2 file
    with bz2.BZ2File(dataset_name, 'rb') as file:
        with open(decompressed_dataset_name, 'wb') as new_file:
            for data in iter(lambda: file.read(100 * 1024), b''):
                new_file.write(data)

print(f"Decompressed file saved as {decompressed_dataset_name}")

# Delete the compressed file
if os.path.isfile(dataset_name):
    os.remove(dataset_name)
