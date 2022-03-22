# SDVN simulation platform
Installation instructions：
1. Start it with the python file test**. Attention: you need to modify the data name to your own data file and related parameters.
2. Related test files: testTSD, testCT, testDijkstra, testIM-DOS, testLEAF,testGAME,testSWORDFISH.
3. The file "node" is used to process vehicle information while the vehicle is driving on the road.
4. File "Packet" is used to process data packet information.
5. File "SDVN_main" and "SDVN_Controller" are used to formulate the controller in SDVN.
6. File "Get_Move" is used to process the trajectory data of the vehicle.
7. The simulation platform is based on a real map of Tiexi District, Shenyang City, China. The map information is achieved from the OpenStreetMap. Then, we apply SUMO to generate the trajectory data of vehicle fitting on these maps. At last, we use our simulator to perform the data packet transmission in the vehicular networks. The transmission range is set as 250m. Even if the distance between the two vehicles is within range, high load and channel interference will still cause delay increase and even packet loss. These mechanisms have been integrated in the platform. Twenty routing requests are generated from random vehicles once per second. Also, the destination of the request is random. 
