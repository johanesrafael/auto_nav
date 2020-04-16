import csv
import picamera
import time

#sudo apt-get install python-picamera
def read_csv(path):
  rows = []
  with open(path) as csvfile:
    file_reader = csv.reader(csvfile)
    for row in file_reader:
      rows.append(list(row))
  return rows

camera = picamera.Picamera()
image = read_csv(camera.capture())
count = 0
seen = 0
for row in image:
    for column in image:
        if column[0] > 0 and column[1] > 0 and column[2] > 0: 
            seen += 1
        elif column[0] > 1:
            count += 1
print(seen)
print(count)
        
