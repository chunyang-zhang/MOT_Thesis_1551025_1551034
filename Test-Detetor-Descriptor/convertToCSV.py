import os
import numpy as np 
import csv 
from os import listdir
#return the 
def read_from_folder(file_names_path ):
    method_result = []
    for file_path  in file_names_path:
        method_dict = load_from_file(file_path)
        method_result.append(method_dict)
    return method_result
def load_from_file(file_name):
    method_dict = dict()
    with open(file_name,'r') as file:
        lines = file.readlines()
        names = lines[0].split(', ')
        names[len(names)-1] = names[len(names)-1][:-1]
        values = lines[1].split(', ')
        for i in range(len(names)):
            method_dict[names[i]] =float(values[i])
    return method_dict 
def write_to_csv(file_name,method_names,method_result):
    total_mse = []
    with open(file_name, 'w',newline='') as csvfile:
        #method result keys are field names
        if(len(method_result)!=0):
            field_names =list(method_result[0].keys())
        else:
            return
        field_names.insert(0,"names")
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(method_result)):
            row = [method_names[i].replace(".txt","")]
            for value in method_result[i].values():
                row.append(value)
            #get mse value
            list_method_result = list(method_result[i].values())
            total_mse.append(list_method_result[-3:len(list_method_result)])
            spamwriter.writerow(row)
    return total_mse
def write_avg_result_to_csv(output_name,file_names,total_avg_matrix):
     with open(output_name, 'w',newline='') as csvfile:
        #method result keys are field names
        field_names =["MSE x","MSE y","MSE z"]
        field_names.insert(0,"names")
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(total_avg_matrix)):
            row = [file_names[i].replace(".txt","")]
            for value in total_avg_matrix[i]:
                row.append(value)
            spamwriter.writerow(row)

path = 'data'
folder_names = os.listdir(path)
path_folder_names =  [os.path.join(path,i) for i in folder_names]
total_mse = []
if (len(path_folder_names)!=0):
    file_names = os.listdir(path_folder_names[0])

total_mse_matrix = np.zeros((len(file_names),3),dtype = float)
total_avg_matrix = np.full((len(file_names),3),1./len(path_folder_names),dtype = float)
for i in range(len(path_folder_names)):
    #file_names inside folder
    #path of file name
    path_file_names =  [os.path.join(path_folder_names[i],name) for name in file_names]
    method_results = read_from_folder(path_file_names)
    #name of output file is name of the folder
    outputfile_name = ''.join((folder_names[i],".csv"))
    print(outputfile_name)
    total_mse = write_to_csv(outputfile_name,file_names,method_results)
    total_mse_matrix += np.matrix(total_mse)
#print(total_avg_matrix)
#print(total_mse_matrix)
total_avg_result = total_mse_matrix*total_avg_matrix
#print(total_avg_result)
write_avg_result_to_csv("final_avg_result.csv",file_names,total_avg_result)
#avg result to final output