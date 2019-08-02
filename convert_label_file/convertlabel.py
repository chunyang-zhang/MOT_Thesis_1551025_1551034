import os
import numpy as np 
import csv 
from os import listdir
import operator
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
                elif(index[i]==0):
                    label.append(int(values[index[i]]))
                elif(index[i] ==1):
                    label.append(int(values[index[i]]))
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
    
#order based on object id -> 
def special_reorder_label(label_list):
    i = 0
    s = sorted(label_list, key = operator.itemgetter(1, 0))
    return s
    
def write_file(file_name,reorder_label_dict):
    countValueDict = 0
    with open(file_name,'w') as file:
        n = len(reorder_label_dict.values())
        for value in reorder_label_dict.values():
            countValueDict+=1
            if(countValueDict == n):
                valueLine = len(value)
                countLine = 0
                for line in value:
                    countLine+=1
                    if(countLine ==valueLine):
                        line = ' '.join(line)
                        file.write(line)
                    else:
                        line = ' '.join(line)
                        line+='\n'
                        file.write(line)
            else:
                for line in value:
                    line = ' '.join(line)
                    line+='\n'
                    file.write(line)
def write_special_file(file_name,reorder_label_list):
    countValueDict = 0
    with open(file_name,'w') as file:
        n = len(reorder_label_list)
        for value in reorder_label_list:
            countValueDict+=1
            value[0] =str(value[0])
            value[1] =str(value[1])
            if(countValueDict == n):
                line = ' '.join(value)
                file.write(line)
            else:
                line = ' '.join(value)
                line+='\n'
                file.write(line)

label_list = convert_label("label.txt")
a = [1,2,"asdf",'phy']
print(a)
#print(label_list)
reorder_label_dict = reorder_label(label_list)
#print(reorder_label_dict)
special_reorder_list = special_reorder_label(label_list)
#print(special_reorder_list)

write_special_file("label2.txt",special_reorder_list)