import time

print('salam')
for i in range(10):
    print(i, end="")
    time.sleep(.1)
    print("", end="\r")
print('\nend')