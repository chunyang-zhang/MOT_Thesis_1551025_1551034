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
    total_aed = []
    total_process_time = []
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
            #first col is the file name
            row = [method_names[i].replace(".txt","")]
            for value in method_result[i].values():
                if(type(value) == float):
                    row.append("{0:.4f}".format(value))
                else:
                    row.append(value)

            #get mse value
            list_method_result = list(method_result[i].values())
            #get the 3xn error matrix 
            total_mse.append(list_method_result[-5:-2])
            total_aed.append(list_method_result[-2])
            total_process_time.append(list_method_result[-1])
            spamwriter.writerow(row)
    return total_mse, total_aed, total_process_time
def write_onemethod_csv(file_name, folder_names, one_method_result):
    if(len(one_method_result)!=0):
        field_names =list(one_method_result[0].keys())
        field_names.insert(0,"DATA")
    avg_aed = 0
    avg_time = 0
    avg_x = 0
    avg_y = 0
    avg_z = 0
    with open(file_name, 'w',newline='') as csvfile:
        #method result keys are field names
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(0,len(one_method_result)):
            #first col is the file name
            row = []
            row.append(folder_names[i])
            avg_x += one_method_result[i]["MSE x"]
            avg_y += one_method_result[i]["MSE y"]
            avg_z += one_method_result[i]["MSE z"]
            avg_aed += one_method_result[i]["AED"]
            avg_time += one_method_result[i]["Avg Feature Time (s)"]
            for value in one_method_result[i].values():
                if(type(value) == float):
                    row.append("{0:.4f}".format(value))
                else:
                    row.append(value)
            
            spamwriter.writerow(row)
        row = []
        row.append("Avg Result")
        row.append("")
        row.append("")
        row.append("")
        row.append("")
        row.append("{0:.4f}".format(avg_x/len(one_method_result)))
        row.append("{0:.4f}".format(avg_y/len(one_method_result)))
        row.append("{0:.4f}".format(avg_z/len(one_method_result)))
        row.append("{0:.4f}".format(avg_aed/len(one_method_result)))
        row.append("{0:.4f}".format(avg_time/len(one_method_result)))
        spamwriter.writerow(row)

def write_avg_result_to_csv(output_name,file_names,total_avg_matrix,total_avg_aed,process_time_result):
    with open(output_name, 'w',newline='') as csvfile:
        #method result keys are field names
        field_names =["MSE x","MSE y","MSE z","AED","Avg Process Time (s)"]
        field_names.insert(0,"names")
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(total_avg_aed)):
            row = [file_names[i].replace(".txt","")]
            for value in total_avg_matrix[i]:
                if(type(value) == float):
                    row.append("{0:.4f}".format(value))
                else:
                    row.append(value)
            row.append("{0:.4f}".format(total_avg_aed[i]))
            row.append("{0:.4f}".format(process_time_result[i]))
            spamwriter.writerow(row)

def getDetectorDescriptor(file_names,aed_result,process_time_result,markers,colors):
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
        feature["aed"] = aed_result[i]
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
#a nx3 1/n matrix for mse
avg_mse_matrix = np.full((len(file_names),3),1./len(path_folder_names),dtype = float)

#a nx1 matrix for aed
total_aed_matrix = np.zeros(len(file_names),dtype = float)
#a nx1 matrix for process time
process_time_matrix = np.zeros(len(file_names),dtype = float)
#a 1/n proces time matrix
avg_process_time_matrix = np.full(len(file_names),1./len(path_folder_names),dtype = float)
#a 1/n aed matrix
avg_aed_matrix = np.full(len(file_names),1./len(path_folder_names),dtype = float)

method_id = 6
one_method_result = []
for i in range(len(path_folder_names)):
    #file_names inside folder
    #path of file name
    path_file_names =  [os.path.join(path_folder_names[i],name) for name in file_names]
    method_results = read_from_folder(path_file_names)
    #print only 1 method dector-descriptor of all dataset
    one_method_result.append(method_results[method_id])
    #name of output file is name of the folder
    outputfile_name = ''.join((folder_names[i],".csv"))
    print(outputfile_name)
    total_mse, total_aed ,process_time = write_to_csv(outputfile_name,file_names,method_results)
    total_mse_matrix += np.array(total_mse)
    process_time_matrix += np.array(process_time)
    total_aed_matrix += np.array(total_aed)
file_name = file_names[method_id].replace('.txt','.csv')
write_onemethod_csv(file_name,folder_names,one_method_result)
#print(total_avg_matrix)
#print(total_mse_matrix)
total_avg_result = total_mse_matrix*avg_mse_matrix
process_time_result = process_time_matrix*avg_process_time_matrix
aed_result = total_aed_matrix*avg_aed_matrix
#print(total_avg_result)
write_avg_result_to_csv("final_avg_result.csv",file_names,total_avg_result,aed_result,process_time_result)
#avg result to final output

#Draw Graph
#detector
#Assume number of markers = number of Detectors
markers = ["o","s","D","^","P","*"] #circle, square, diamond, pentagon, plus, hexagon
#descriptor
#Assume nubmer of colors = number of Descriptors
colors = ["gold","orangered","blue","aqua","green","deeppink"]

featureDict, detectors_marker, descriptors_color = getDetectorDescriptor(file_names,aed_result,process_time_result,markers,colors)
#print(featureDict)
fig, ax = plt.subplots()
ax.set_facecolor('whitesmoke')
#plot features
for feature in featureDict:
    ax.scatter(feature["time"],feature["aed"],s = 50, color = feature["color"],marker= feature["marker"],edgecolors='black')
plt.xlabel('CPU Time (s)')
plt.ylabel('Overall AED')
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
#plt.yticks(np.arange(1, 2.5, 0.5))
#plt.xticks(np.arange(0, 3, 0.4))
plt.show()
fig.savefig('feature.png')