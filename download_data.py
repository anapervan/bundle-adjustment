import os
import urllib.request

# Download data
project_url = "http://grail.cs.washington.edu/projects/bal/data/ladybug/"
dataset_name = "problem-73-11032-pre.txt.bz2"  # "problem-49-7776-pre.txt.bz2"
full_url = project_url + dataset_name

# If the file isn't already saved, download it
if not os.path.isfile(dataset_name):
    urllib.request.urlretrieve(full_url, dataset_name)