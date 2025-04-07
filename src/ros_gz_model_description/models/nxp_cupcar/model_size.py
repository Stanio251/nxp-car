import trimesh

if __name__ == '__main__':
    print('Please put the input file to the same folder as this script and type in the full name of your file.')
    file_name = input()
    ms=trimesh.load(file_name)
    print(ms.bounding_box.extents) #length, width , hight


