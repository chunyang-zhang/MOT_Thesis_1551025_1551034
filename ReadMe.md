# Moving Object Tracking 

## OpenCV 4.1
### Requirement:
#### OpenCV 4.1
Download OpenCV 4.0
+ Install OpenCV 4.0 From https://opencv.org/releases/

+ Include OpenCV to system path  
    Go to Advanced System Settings > Environment Variables >System Variables > Path.  
    Choose Edit, click 'New' to add environment variable.   
    Copy Paste the path of bin folder. The path: C:\opencv\build\x64\vc14\bin.

**Inside Properties of the project,** 
+ Inside C/C++ > General  
    Copy the path to include the folder of OpenCV,  inside Additional Include Directories: C:\opencv\build\include
+ Inside Linker>General  
    Copy the path to include the folder containing lib folder inside Additional Include Directories:
    C:\opencv\build\x64\vc14\lib
+ Input  
    Edit Input> Additional Dependencies and paste the .lib file' name eg. opencv_world410d.lib
#### OpenCV 4.1 Contrib
+ Download OpenCV Contrib  
    https://github.com/opencv/opencv_contrib/releases
+ Build OpenCV Contrib  
    ```
    cd <opencv_build_directory>
    cmake -DOPENCV_EXTRA_MODULES_PATH=C:/opencv_contrib-4.1.0/modules C:/opencv/sources  -G
    ```
+ Open OpenCV Solution inside Opencv/build/ to build project  
    ```
    Build ALL_BUILD
    Build INSTALL
    ```
+ Add environment variable for OpenCV_Contrib    
  C:\opencv\build\install\x64\vc16\bin  
**Problem**  
    Python3.7_d.lib linked error.  
**Fix**  
    Install Python 3.7 x64 version with debug mode.    
    Choose Building platform correctly => Debug x64 in OpenCV\sources\modules\python\python3, edit CMakeLists.txt:  
    ```
    link_directories(C:/python37/libs)
    ```
**Inside Properties of the project in Visual Studio**
+ Inside C/C++ > General    
    Copy the path to include the folder of OpenCV,  inside Additional Include Directories: C:\opencv\build\install\include
+ Inside linker>General  
    Copy the path to include the folder containing lib folder inside Additional Include Directories: C:\opencv\build\install\x64\vc16\lib
+ Input  
    Edit Input > Additional Dependencies and paste the .lib file' name eg. opencv_xfeatures2d410d.lib, opencv_features2d410d.lib
    
#### Boost Library
+ Download: https://dl.bintray.com/boostorg/release/1.70.0/source/
+ Project properties -> C/C++ -> General -> Additional Include Directories add boost library root: C:\Program Files (x86)\Boost_1_70
+ Project properties -> Linker -> General -> Additional Libraries Directories:  
Path of Boost library: C:\Program Files (x86)\Boost_1_70

**OpenCV includes eigen error:**
+ Put <Eigen/Dense> before #include <opencv2/core/eigen.hpp>
#### Eigen library
+ C++ template library for linear algebra (high level)
+ Download: http://bitbucket.org/eigen/eigen/get/3.3.7.zip
+ Project properties -> C/C++ -> General -> Additional Include Directories add Eigen library root: C:\Program Files (x86)\Eigen
+ For example: 
```
    #include <Eigen/Dense>
    #include <eigen/src/lu/InverseImpl.h>
    #include <Eigen/Eigenvalues>
```
+ Reference:
    https://www.deciphertechnic.com/install-opencv-with-visual-studio/

**DataFormat**
+ Two Folder contains images from 2 camera
+ TimeStamps: time of image captured from each camera
+ GPS pose (roll, pitch, yaw )
+ all_IMU: the time using IMU sensors + values (30)
+ Output: Position

**OpenCV Window and Window conflict**
+ Cause by putting using name space cv in header;
+ If need to define cv in header: use cv:: in font of class name
+ Put using namespace cv; inside cpp file  
+ Reference:  
https://answers.opencv.org/question/62079/access-mask-ambiguous-symbol/

#### YOLO Object Detection
**Download model**  
+ Yolov3.weights:  
https://pjreddie.com/media/files/yolov3.weights
+ Network Configuration:  
https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true
+ 80 different class names dataset:  
https://github.com/pjreddie/darknet/blob/master/data/coco.names?raw=true 
