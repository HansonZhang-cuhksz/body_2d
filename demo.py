with open("test.bvh", 'r') as file:
    linelist = file.readlines()[-1].split()
    print(len(linelist))