import os
import numpy as np 
import csv 
from os import listdir
import matplotlib.pyplot as plt
from matplotlib.legend import Legend
#return the 
def read_from_folder(file_names_path ):
    method_result = []
    for file_path  in file_names_path:
        method_dict = load_from_file(file_path)
        method_result.append(method_dict)
    return method_result
def result_read_from_folder(error_pose_file_names):
    method_result = []
    for file_path  in error_pose_file_names:
        method_dict = dict()
        error_type = file_path.split("/")
        error_type = error_type[-1].replace(".txt","")
        error_type = error_type.split("_")
        error_type = error_type[-1]
        method_dict["name"] = error_type
        method_dict.update(load_from_file(file_path))

        method_result.append(method_dict)
    return method_result

def load_from_file(file_name):
    method_dict = dict()
    with open(file_name,'r') as file:
        lines = file.readlines()
        #read the title
        names = lines[0].split(', ')
        #remove enter at last item
        names[len(names)-1] = names[len(names)-1][:-1]
        #only get Value
        names = names[2:len(names)]
        values = lines[-1].split()
        values = values[2:len(values)]
        for i in range(len(names)):
            method_dict[names[i]] =float(values[i])
    return method_dict 
def load_from_file2(tracking_result_file_names,method_name):
    tracking_result = []
    j = 0
    for file_name in tracking_result_file_names:
        with open(file_name, 'r') as file:
            lines = file.readlines()
            #read the title
            names = lines[0].split(', ')
            #remove enter at last item
            names[len(names)-1] = names[len(names)-1][:-1]
            #only get Value
            n_lines = lines[1:-1]
            names.insert(0,method_name[j])
            for line in n_lines:
                method_dict = dict()
                line =line[:-1]
                line = line.split(' ')
                method_dict["name"] = names[0]
                for i in range(len(line)):
                    if(i>=2):
                        method_dict[names[i+1]] = float(line[i])
                    else:
                        method_dict[names[i+1]] =line[i]
                tracking_result.append(method_dict)
        j+=1
    return tracking_result 
def write_to_csv(file_name, method_result):
    with open(file_name, 'w',newline='') as csvfile:
        #method result keys are field names
        if(len(method_result)!=0):
            field_names =list(method_result[0].keys())
        else:
            return
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(method_result)):
            row = []
            for value in method_result[i].values():
                row.append(value)
            spamwriter.writerow(row)
    return 
def write_to_csv_numpy(output_name,method_results,label_name,field_names):
     with open(output_name, 'w',newline='') as csvfile:
        #method result keys are field names
        spamwriter = csv.writer(csvfile)
        spamwriter.writerow(field_names)
        for i in range(len(method_results)):
            row = []
            row.append(label_name[i])
            for value in method_results[i]:
                row.append(value)
            spamwriter.writerow(row)
def convert2NPMatrix (method_results):
    matrix = []
    method_name = []
    for method in method_results:
        #string error?
        line = list(method.values())
        method_name.append(line[0])
        matrix.append(np.array(line[1:]))
    return method_name, matrix

def getTrackingResult(method_name,tracking_final_result,markers,colors):
    trackingDict = []
    trackingMarker = dict()
    for i in range(len(method_name)):
        feature = dict()
        feature["name"] = method_name[i]
        feature["iou50"] = tracking_final_result[i][0]
        feature["iou75"] = tracking_final_result[i][1]
        feature["time"] = tracking_final_result[i][2]
        feature["marker"] = markers[i]
        trackingDict.append(feature)
        trackingMarker[method_name[i]] = markers[i]
    return trackingDict, trackingMarker
path = 'data'
#list all folder name
folder_names = os.listdir(path)
# path of folder name
path_folder_names =  [os.path.join(path,i) for i in folder_names]
total_mse = []
if (len(path_folder_names)!=0):
    file_names = os.listdir(path_folder_names[0])
tracking_result = []
error_pose_result = []
#file_name processing
for file_name in file_names:
    fnTmp = file_name.replace(".txt","")
    fnChar = fnTmp.split('_')
    if fnChar[0] == "error":
        error_pose_result.append(file_name)
    else:
        tracking_result.append(file_name)

full_error_matrix = np.zeros((len(error_pose_result),4),dtype = float)
avg_error_matrix = np.full((len(error_pose_result),4),1./len(path_folder_names),dtype = float)
full_tracking_matrix = np.zeros((len(tracking_result),3),dtype = float)
avg_tracking_matrix = np.full((len(tracking_result),3),1./len(path_folder_names),dtype = float)
method_name = []
detail_result = []
for i in range(len(path_folder_names)):
    #file_names inside folder
    #path of file name
    error_pose_file_names =  [os.path.join(path_folder_names[i],name) for name in error_pose_result]
    tracking_result_file_names =  [os.path.join(path_folder_names[i],name) for name in tracking_result]

    error_pose_results = result_read_from_folder(error_pose_file_names)
    tracking_results = result_read_from_folder(tracking_result_file_names)
    #name of output file is name of the folder
    error_outputfile = ''.join(("errorpose_",folder_names[i],".csv"))
    tracking_outputfile = ''.join(("tracking_",folder_names[i],".csv"))

    write_to_csv(error_outputfile, error_pose_results)
    write_to_csv(tracking_outputfile,tracking_results)

    method_name, error_matrix = convert2NPMatrix(error_pose_results)
    method_name, tracking_matrix = convert2NPMatrix(tracking_results)

    detail_result = load_from_file2(tracking_result_file_names,method_name)
    detail_tracking_outfile = ''.join(("detail_tracking_",folder_names[i],".csv"))
    write_to_csv(detail_tracking_outfile,detail_result)
    full_error_matrix +=np.array(error_matrix)
    full_tracking_matrix +=np.array(tracking_matrix)
error_pose_final_result = full_error_matrix*avg_error_matrix
tracking_final_result = full_tracking_matrix*avg_tracking_matrix
error_field_names = ["Method Name","MSE x","MSE y","MSE z","AED"]
tracking_field_names =  ["Method Name","IoU50","IoU75","Process Time (s)"]
#write avg result
write_to_csv_numpy("error_pose_avg_result.csv",error_pose_final_result, method_name, error_field_names)
write_to_csv_numpy("tracking_result.csv",tracking_final_result,method_name, tracking_field_names)

#Draw Graph of IoU Tracking

#Draw Graph
#detector
#Assume number of markers = number of Detectors
#Tracking 
#IoU IoU Matching, Image Matching
markers = ["o","s","D"] #circle, square, diamond
#descriptor
#Assume nubmer of colors = number of Descriptors
#iou 50 #iou 75
colors = ["orangered","blue"]

trackingDict,trackingMarker = getTrackingResult(method_name,tracking_final_result,markers,colors)
#print(featureDict)
fig, ax = plt.subplots()
ax.set_facecolor('whitesmoke')
#plot features
for feature in trackingDict:
    ax.scatter(feature["time"],feature["iou50"],s = 50, color = colors[0],marker= feature["marker"],edgecolors='black')
    ax.scatter(feature["time"],feature["iou75"],s = 50, color = colors[1],marker= feature["marker"],edgecolors='black')

plt.xlabel('CPU Time (s)')
plt.ylabel('Overall IoU')
plt.title('Comparison of Error vs Speed of Different Tracking Techniques', fontsize = 10)
#plot empty list with desized color and label

for key,value in trackingMarker.items():
    ax.scatter([],[], c = 'k', marker = value , label = key,edgecolors='black')
legend1 =  ax.legend(frameon=True,loc ="upper left", title='Detectors',bbox_to_anchor=(1.04,1))
ax.add_artist(legend1)
h, l =  ax.get_legend_handles_labels()
#print(h, l)
#plot empty list with desized shape and label
label_color = {"iou50":"orangered","iou75":"blue"}
for key, value in label_color.items():
    ax.scatter([],[], c = value, marker = 'o', label = key)
h, l =  ax.get_legend_handles_labels()
#print(h, l)

legend2 = ax.legend(h[3:5],l[3:5], bbox_to_anchor=(1.04, 0), frameon=True,loc ="lower left", title='Descriptors')
plt.subplots_adjust(right=0.7)
#plt.yticks(np.arange(1, 2.5, 0.5))
#plt.xticks(np.arange(0, 3, 0.4))
plt.show()
fig.savefig('feature.png')