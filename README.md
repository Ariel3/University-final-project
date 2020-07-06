# University project

Development orientation obstacle avoidance algorithm for small drone.
using BoofCV library version 0.36.

In recent years there has been a rapid development in the Unmanned Aerial Vehicle (UAV) field. The most popular and useful tools for UAV are spatial sensors and GPS. These techniques can yield a high level of performance and accuracy, but they also have some drawbacks including their heavy weight and dependency on external devices. We suggest a new orientation algorithm to assist UAV navigate around obstacles using only one additional camera.

Our goal is to give the UAV the ability to move safely in space and avoid obstacles. A stereoscopic picture that uses two cameras will enable us to assess spatial depth. Based on the stereoscopic image the algorithm can assess its speed, depth and distance faster and more accurately between the UAV and the surrounding objects. 

The algorithm relies on parameters that will be set throughout the flight, such as the distance between the two cameras, camera aperture and dimensions of the image. 

Load set of sequential images.  
Compute associated pair images from BoofCV.
For each point: 
Find its spatial 3D coordinates.
Calculate the distance to every point in the image.
Build a 2D & 3D points map of distances. 
Using the KLT-pyramid algorithm for tracking points in the next image.
Estimate the best path to move forward.
Send the instruction to the car\UAV.


