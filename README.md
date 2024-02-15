# Camera-Calibration
<font size=5 color=BLACK>**Introduction**</font>  
  
This is a camera calibration program designed for the optical navigation system of spinal surgery robots. Spinal surgery robotic systems commonly employ endoscopic cameras to provide surgeons with a visual guide during procedures.  
  
However, these endoscopic cameras are susceptible to camera distortion, as Figure 1 illustrates, which can impact the accuracy and reliability of the visual information relayed to surgeons. Camera distortion in spinal surgery robotic systems often arises due to lens imperfections, non-uniformities in the optical system, or the bending and reflection of light within the endoscope. These distortions may manifest as radial or tangential distortions, leading to image artifacts such as stretching, compression, or skewing of visual elements.   
<div align=center>
<img src="https://github.com/ShawnHYSun/Images/blob/main/Camera%20Distortion.png" width="400" height="220">
</div>
<p align="center">Figure 1. Camera Distortion</p>  
  
The presence of camera distortion is a critical consideration in spinal surgery robotics, as it can introduce inaccuracies in depth perception and spatial relationships, potentially compromising the surgeon's ability to precisely navigate and manipulate surgical instruments. Efforts to mitigate camera distortion involve advanced calibration techniques, distortion correction algorithms, and the utilization of high-quality optical components. Addressing camera distortion in spinal surgery robotic systems is essential to ensure the fidelity of the visual feedback provided to surgeons, ultimately enhancing the overall safety and efficacy of these advanced medical interventions.   
  
Additionally, the position of the surgical tools relative to the camera also needs to be determined. In the context of spinal surgery robot navigation systems, hand-eye calibration is a crucial technique with paramount importance in determining the accurate position and orientation of surgical tools relative to the robotic system. By modeling and calibrating the geometric relationship between the camera and the surgical tool, hand-eye calibration provides precise information about the tool's location, ensuring the robot system's ability to meet the high accuracy requirements for tool positioning in spinal surgeries. This process enables the robotic system to precisely control the position of the surgical tools in three-dimensional space, providing accurate camera-tool coordinate transformation for the navigation system. During actual surgical procedures, hand-eye calibration serves as the foundation for accurate robot system navigation and positioning. By ensuring that the robotic system comprehends the surgical scene accurately, hand-eye calibration facilitates precise navigation and positioning, enhancing the operational accuracy and stability of complex spinal surgeries conducted within a patient's body. This not only reduces surgical risks but also enhances the safety and controllability of the surgery, providing reliable support for surgeons.  
  
The stucture of the optical navigation system of spinal surgery robot is shown in Figure 2.  
<div align=center>
<img src="https://github.com/ShawnHYSun/Images/blob/main/System%20Stucture.png" width="400" height="220">
</div>
<p align="center">Figure 2. System Stucture Diagram</p>  

Consequently, this project proposes a method to calibrate the camera, remove distortion of the image, obtain the camera posture and tool posture, and finally perform hand-eye calibration by using **OpenCV**, typically **cv2.calibrateHandEye** and **cv2.calibrateCamera** functions.

**Code Description**  
  
The provided code focuses on the calibration and navigation aspects within a spinal surgery robot system. Initially, the script undergoes a calibration process, utilizing images of a chessboard pattern to determine intrinsic camera parameters like distortion coefficients, camera matrix, and rotation and translation vectors. These parameters are crucial for rectifying and undistorting subsequent images, enhancing the accuracy of image processing.  
  
The code then proceeds to undistort a set of input images using the obtained calibration parameters, creating a folder of undistorted images for further analysis. Subsequently, the undistorted images are recalibrated to refine the camera matrix and distortion coefficients. Additionally, the script extracts 3D pose information from corresponding CSV files, representing the transformation from the robot tool to the base. The core functionality lies in the calibration of hand-eye coordination, which involves finding the relationship between the tool's position and orientation concerning the camera's coordinate system. The script employs the OpenCV library to perform this hand-eye calibration (cv2.calibrateHandEye).  
  
The final output includes the transformation matrix representing the camera-to-tool relationship, enabling precise navigation and positioning of surgical tools during spinal surgeries. The code also demonstrates the projection of control points, showcasing the accuracy achieved in mapping points between different coordinate systems.  
  
In summary, this script automates the calibration of a spinal surgery robot system, ensuring accurate navigation and coordination between the camera and surgical tools, ultimately enhancing the precision and safety of spinal surgeries.  
  
**Result**  
  
In Figure 3, the upper image is the raw image of endoscope camera and the lower image is the image after calibration and undistortion.  

<div align=center>
<img src="https://github.com/ShawnHYSun/Images/blob/main/Camera%20Results.png" width="400" height="850">
</div>
<p align="center">Figure 3. Calibration and Undistortion Resuklt</p>  
