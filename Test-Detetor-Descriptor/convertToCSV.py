import os
import numpy as np 
import csv 
from os import listdir
import matplotlib.pyplot as plt
from matplotlib.legend import Legend

#import matplotlib.colors as colors
#return the 
def read_from_folder(file_names_path ):
    method_result = []
    #read each file and append to method_result
    for file_path  in file_names_path:
        method_dict = load_from_file(file_path)
        method_result.append(method_dict)
    return method_result
def load_from_file(file_name):
    method_dict = dict()
    with open(file_name,'r') as file:
        lines = file.readlines()
        #split the title for each attributte
        names = lines[0].split(', ')
        #remove the enter at the last position
        names[len(names)-1] = names[len(names)-1][:-1]
        values = lines[1].split(', ')
        for i in range(len(names)):
            method_dict[names[i]] =float(values[i])
    return method_dict 
def write_to_csv(file_name,method_names,method_result):
    total_mse = []
    total_process_time = []
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
            #first col is the file name
            row = [method_names[i].replace(".txt","")]
            for value in method_result[i].values():
                row.append(value)
            #get mse value
            list_method_result = list(method_result[i].values())
            #get the 3xn error matrix 
            total_mse.append(list_method_result[-4:len(list_method_result)-1])
            total_process_time.append(list_method_result[-1])
            spamwriter.writerow(row)
    return total_mse, total_process_time
def write_avg_result_to_csv(output_name,file_names,total_avg_matrix,process_time_result):
    avg_mse = []
    with open(output_name, 'w',newline='') as csvfile:
        #method result keys are field names
        field_names =["MSE x","MSE y","MSE z","Avg MSE","Avg Process Time (s)"]
        field_names.insert(0,"names")
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(total_avg_matrix)):
            row = [file_names[i].replace(".txt","")]
            mse = 0
            for value in total_avg_matrix[i]:
                row.append(value)
                mse +=value 
            row.append(mse/3)
            avg_mse.append(mse/3)
            row.append(process_time_result[i])
            spamwriter.writerow(row)
    return avg_mse
def getDetectorDescriptor(file_names,total_avg_result,process_time_result,markers,colors):
    featureDict = []

    detectors = []
    descriptors = []
    detector_markers = dict()
    descriptor_colors = dict()
    for i in range( len(file_names)):
        name = file_names[i].replace(".txt","")
        values = name.split("_")
        feature = dict()
        dect = values[-2]
        desc = values[-1]
        if(dect not in detectors):
            detectors.append(dect)
        if(desc not in descriptors):
            descriptors.append(desc)
        feature["detector"] =  dect
        feature["descriptor"]= desc
        feature["mse"] = total_avg_result[i]
        feature["time"] = process_time_result[i]
        featureDict.append(feature)

    for i in range(len(markers)):
        detector_markers[detectors[i]] = markers[i]
    for i in range(len(colors)):
        descriptor_colors[descriptors[i]] = colors[i]

    for featureValue in featureDict:
        featureValue["marker"] = detector_markers[featureValue["detector"]]
        featureValue["color"] = descriptor_colors[featureValue["descriptor"]]

    return featureDict, detector_markers, descriptor_colors
path = 'data'
#whole folder name inside data
folder_names = os.listdir(path)
#whole folder name
path_folder_names =  [os.path.join(path,i) for i in folder_names]
total_mse = []
#the same file name inside each folder
if (len(path_folder_names)!=0):
    file_names = os.listdir(path_folder_names[0])
#a nx3 matrix for mse
total_mse_matrix = np.zeros((len(file_names),3),dtype = float)
#a 1/n file names
total_avg_matrix = np.full((len(file_names),3),1./len(path_folder_names),dtype = float)
#a nx1 matrix for process time
process_time_matrix = np.zeros(len(file_names),dtype = float)
#a 1/n file names
avg_process_time_matrix = np.full(len(file_names),1./len(path_folder_names),dtype = float)
for i in range(len(path_folder_names)):
    #file_names inside folder
    #path of file name
    path_file_names =  [os.path.join(path_folder_names[i],name) for name in file_names]
    method_results = read_from_folder(path_file_names)
    #name of output file is name of the folder
    outputfile_name = ''.join((folder_names[i],".csv"))
    print(outputfile_name)
    total_mse,process_time = write_to_csv(outputfile_name,file_names,method_results)
    total_mse_matrix += np.array(total_mse)
    process_time_matrix += np.array(process_time)
#print(total_avg_matrix)
#print(total_mse_matrix)
total_avg_result = total_mse_matrix*total_avg_matrix
process_time_result = process_time_matrix*avg_process_time_matrix
#print(total_avg_result)
avg_mse = write_avg_result_to_csv("final_avg_result.csv",file_names,total_avg_result,process_time_result)
#avg result to final output

#Draw Graph
#detector
#Assume number of markers = number of Detectors
markers = ["o","s","D","^","P","*"] #circle, square, diamond, pentagon, plus, hexagon
#descriptor
#Assume nubmer of colors = number of Descriptors
colors = ["gold","orangered","blue","aqua","green","deeppink"]
featureDict, detectors_marker, descriptors_color = getDetectorDescriptor(file_names,avg_mse,process_time_result,markers,colors)
#print(featureDict)
fig, ax = plt.subplots()
ax.set_facecolor('whitesmoke')
#plot features
for feature in featureDict:
    ax.scatter(feature["time"],feature["mse"],s = 50, color = feature["color"],marker= feature["marker"],edgecolors='black')
plt.xlabel('CPU Time (s)')
plt.ylabel('Overall MSE')
plt.title('Comparison of Accuracy vs Speed of Feature Detectors-Descriptors', fontsize = 10)
#plot empty list with desized color and label

for key,value in detectors_marker.items():
    ax.scatter([],[], c = 'k', marker = value , label = key,edgecolors='black')
legend1 =  ax.legend(frameon=True,loc ="upper left", title='Detectors',bbox_to_anchor=(1.04,1))
ax.add_artist(legend1)
h, l =  ax.get_legend_handles_labels()
#print(h, l)
#plot empty list with desized shape and label
for key, value in descriptors_color.items():
    ax.scatter([],[], c = value, marker = 'o', label = key)
h, l =  ax.get_legend_handles_labels()
#print(h, l)

legend2 = ax.legend(h[-6:12],l[-6:12], bbox_to_anchor=(1.04, 0), frameon=True,loc ="lower left", title='Descriptors')
plt.subplots_adjust(right=0.7)

plt.show()
fig.savefig('feature.png')