# Troubleshooting

## Error in requesting the calibration metadata - Calibration node not initialized

The guidance node as well as the UI get metadata on the configuration (e.g. sensor and topic names) from the calibration node by requesting them via a service call.
If the calibration node is not initialized, the service request will return with an error  which in turn will be printed out by the requesting components. 
The failure in initialization of the calibration node typically arises if there is an error in the workspace initialization or if the sensor data is not received. 
An error in reading the workspaces often comes from a misconfiguration or wrong file permissions.
Failure in receiving the sensor data could arise from an error in the connection to the platform or a synchronization error in the data from the sensor.
When using approximated synchronization when subscribing to the sensor data, try to increase the queue size.
If the clocks of the sensors are not synchronized (e.g. time stamps that differ by seconds), no message pairs are found with exact or approximate synchronization.
In this case, set the synchronization policy to *Latest Message* (`sync_policy: latest`), which pairs each source message with the latest reference message regardless of the time stamps.

## Is there a possibility to visually debug the result of the GICP

The are a number of debug visualizations within the source code that are currently deactivated using the preprocessor defines `#if 0`.
For example, these debug visualizations are located after optimizing the coefficients the target detection in the LiDAR data, as well as, after the alignment of two 3D clouds by means of GICP.
To use these visualizations to debug, include them in your compilation by setting the preprocessor define to `#if 1`.
When activated, a PCL Visualization window will pop up after the computation revealing the results of the corresponding algorithm.
In this, the white cloud typically represents the target against which the source cloud is aligned, while the red and green cloud represents the not-aligned and aligned source cloud, respectively.