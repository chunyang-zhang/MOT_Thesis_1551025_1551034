import os
import numpy as np 
import csv 
from os import listdir

def convert_label(file_name):
    label_list = []
    index = [0,1,2,6,7,8,9,13,14,15]
    with open(file_name,'r') as file:
        lines = file.readlines()
        #split the title for each attributte
        for line in lines:
            label = []
            values = line.split(' ')
            if(values[2] == "DontCare"):
                continue
            for i in range(len(index)):
                if(index[i] == 8):
                    label.append(str(float(values[index[i]])-float(values[index[3]])))
                elif(index[i]==9):
                    label.append(str(float(values[index[i]])-float(values[index[4]])))
                else:
                    label.append(values[index[i]])
            label_list.append(label)

    return label_list
def reorder_label(label_list):
    reorder_label_dict = dict()
    i = 0
    for label in label_list:
        object_label = []
        if(label[0] =='0'):
            object_label.append(label)
            reorder_label_dict[label[1]]=object_label
            i+=1
        else:
            break
    for j in range(i,len(label_list)):
        #acess to the index
        if(label_list[j][1]in reorder_label_dict.keys()):
            reorder_label_dict[label_list[j][1]].append(label_list[j])
    return reorder_label_dict
    


def write_file(file_name,reorder_label_dict):
    with open(file_name,'w') as file:
        for value in reorder_label_dict.values():

            for line in value:
                line = ' '.join(line)
                line+='\n'
                file.write(line)
        
label_list = convert_label("label.txt")
print(label_list)
reorder_label_dict = reorder_label(label_list)
print(reorder_label_dict)
write_file("label1.txt",reorder_label_dict)