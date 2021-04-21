/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <cmath>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

#define PI 3.14159265;

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

//--------------------------------------START OF MY FUNCTIONS-----------------------------------//

void clean_avg_data(float dist[], float ang[], float qual[], int i) { // function to make data clean :)
    int zDistCount = 0;
    int zQualCount = 0;
    int zero_parameter = 5;
    float set_dist_to = 0;
    if (qual[i] == 0) { // check for the 0 quality points (reflective strips on doors, weird screens etc.
        for (int j = 0; j < zero_parameter + 1; j++) { 
            if (qual[i + j] == 0) { //check for how many 0 qual points
                //printf("zqcount: %d \n", zQualCount);
                zQualCount++;
            }
        }
        if (zQualCount <= zero_parameter) { // if there are 4 or less consecutive 0 quality points we want to cleanem up. 
            for (int n = 0; n < zQualCount; n++) {
                if (i != 0) {
                    dist[i + n] = dist[i - 1];
                    qual[i + n] = qual[i - 1];
                }
                if (i == 0) {
                    qual[i + n] = qual[i + zQualCount];
                    dist[i + n] = dist[i + zQualCount];
                }
            }
        }
    } // end checking for quality == 0
    
    // If good quality but bad distance (anomaly points - unsure why they exist)
    if (dist[i] == 0) {
        //printf("Dist at %f = 0 ", ang[i]);
        for (int j = 0; j < 6; j++) {
            if (dist[i + j] == 0) {
                zDistCount++;
            }
            else {
                set_dist_to = dist[i + j];
            }
        }
        //printf("zDistCount: %d \n", zDistCount);
        if (zDistCount < 5) {
            //printf("Bad dist %f | Distance to be set to = %f \n", ang[i], dist[i-1]);
            for (int n = 0; n < zDistCount; n++) {
                dist[i + n] = set_dist_to;
                /*if (i != 0) { // branch to ensure we are not operating on the first index of the array
                    dist[i + n] = dist[i - 1]; // set a bad point equal to our last good point
                }
                if (i == 0) { // branch for if the first element of our data array is bad 
                    dist[i + n] = dist[i + zDistCount]; // set it to the first known good point (avoids indexing array[-1] causing the planet to explode)
                }*/
            }
        }
    }
    
}

void clean_raw_data(float dist[][20], float qual[][20], int row) { // function to throw out bad quality reads that will hurt the averaged values
    float modelDist = 0.0; // initialize variable to use as the benchmark for setting bad quality points' distance to. 
    float modelQual = 0.0; // same as above, but for the points' quality
    int numBadQual = 0; // initailize count variable
   
    for (int i = 0; i < 20; i++) {
        if (qual[row][i] == 0) {
            numBadQual++; // wanting to check for how many runs of a specific point return a quality of 0
        }
    }
    if ((numBadQual < 5) & (numBadQual > 1)) { // if < 5 of the scans return 0 quality, we can assume those 5 points are bad. (if > 5, could be out of range for return, so don't assume bad)
        for (int i = 0; i < 20; i++) {
            if ((qual[row][i] != 0) & (dist[row][i] != 0)) {
                modelDist = dist[row][i]; // find a good point, set model values to the good points' values
                modelQual = qual[row][i]; // ^
            }
        }
        for (int i = 0; i < 20; i++) { // go through and find the bad points again
            if (qual[row][i] == 0) {
                dist[row][i] = modelDist; // set bad points' distance to a good scan for that point
                qual[row][i] = modelQual; // ^ quality
                //printf("clean raw Dist: %f clean raw Qual: %f \n", dist[row][i], qual[row][i]);
            }
        }

    }
    //printf("numBadqual: %d \n", numBadQual);
    
}

bool is_path(float dist[], float threshold, int i) { // determines if a point is along a good path or not. 
    if ((dist[i] < threshold) && (dist[i] != 0)) { //currently set to 3.5m threshold - to be changed later
        return false;
    }
    else {
        return true;
    }
}

bool is_no_return(float dist[], float qual[], int i) {

    // shouldnt be needed anymore
    if ((dist[i] == 0) /*& (qual[i] == 0)*/) {
        return true;
    }
    else {
        return false;
    }
}

//------------POTENTIALLY SKETCH----------------//

//----------END POTENTIALLY SKETCH--------------//
float find_gap_width(float dist[], float ang[], int nStart, int nEnd) { // find size of gap (mainly for use indoors - may utilize outside)
    float dTheta = 0.0;
    if (nStart > nEnd) {
        dTheta = abs((ang[nEnd] + 360) - ang[nStart]);
    }
    else {
        dTheta = abs(ang[nStart] - ang[nEnd]);
    }
    //printf("nStart: %d nEnd: %d \n", nStart, nEnd);
    //printf("dTheta: %f dist[nStart]: %f dist[nEnd]: %f \n", dTheta, dist[nStart], dist[nEnd]);
    float gapSize = sqrt(((dist[nStart] * dist[nStart]) + (dist[nEnd] * dist[nEnd]) - (2.0 * dist[nStart] * dist[nEnd] * cos( dTheta * 3.14159265 / 180.0))));
    //printf("Gap Width: %f \n", gapSize);
  
    return gapSize;
}

float find_gap_depth(float dist[], float ang[], int nStart, int nEnd, int numPoints) { // find depth of a gap - will read slightly too big
    float minDist = 20000; // sufficiently large number
    float gapDepth = 0;

    if (nStart > nEnd) { // checking for a gap starting at more than 180 deg and ending at less than 180 deg
        int numBackend = (numPoints - nStart); // check for number of data points on the back end of our list to bridge the gap
        for (int i = 0; i < numBackend; i++) { //for each of these points:
            if (dist[nStart + i] < minDist) { // check for worst case depth on back end of gap
                minDist = dist[nStart + i];
            }
        }
        for (int i = 0; i < nEnd; i++) { // check for worst case depth on front end of gap
            if (dist[i] < minDist) {
                minDist = dist[i];
            }
            else {
                //printf("minDist not here: %f \n", ang[i]);
            }
            gapDepth = minDist - dist[nEnd + 1]; 
        }
    }
    else {
        int numel = nEnd - nStart; // total num of points in this gap
        for (int i = 0; i < numel; i++) {
            if (dist[nStart + i] < minDist) { //find the probable worst-case gap depth - cannot find exact, too many unknnowns, but this should work
                minDist = dist[nStart + i];
            }
            gapDepth = minDist - dist[nEnd + 1]; 
        }
    }

    //printf("Mindist: %f dist[nEnd + 1]: %f \n", minDist, dist[nEnd + 1]);
    return gapDepth;
    //printf("gapDepth: %f \n", gapDepth);
}

int find_bridge_end(int nStart, int nEnd) {
    int endBridge = 0;
    if (nStart <= 1) {
        //printf("nStart = %d nEnd = %d \n ", nStart, nEnd);
        endBridge = nEnd;
        return endBridge;
    }
    else {
        return nEnd;
    }
}

int find_bridge_start(int nStart, int nEnd, int count) {
    int startBridge = 0;
    if (nEnd == (count - 1)) {
        //printf("nStart = %d nEnd = %d \n ", nStart, nEnd);
        startBridge = nStart;
        return startBridge;
    }
    else {
        return nStart;
    }
}



//------------------------------------END OF FUNCTIONS-------------------------------//


int main(int argc, const char* argv[]) {


    const int arbSize = 1000; //arbitrary array size
    const int sampleAvg = 20;//IF YOU CHANGE THIS RESIZE THE 2d ARRAY FUNCTIONS' PARAMETERS!!

    float readDistArr[arbSize][sampleAvg];
    float thetaArr[arbSize] = {};
    float readQualArr[arbSize][sampleAvg];

    float distArr[arbSize] = {};
    float qualArr[arbSize] = {};

    float sumDist[arbSize] = {};
    float sumQual[arbSize] = {};

    int numPoints = 0;

    bool pathValid = false;

    float ten = 10.0; // beacause like i have to or something idk Im not a programer

    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;

    int all_clear_count = 0;
    int num_runs = 3;
    
    const char* opt_com_path = NULL;
    _u32         baudrateArray[2] = { 115200, 256000 };
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    std::ofstream wpFile;
    wpFile.open("waypoint_data.csv");

    std::ofstream badScanData;
    badScanData.open("bad_scan_data.csv");

    std::ofstream goodScanData;
    goodScanData.open("good_scan_data.csv");

    std::ofstream widthFile;
    widthFile.open("valid_gap_widths.csv");

    std::ofstream depthFile;
    depthFile.open("valid_gap_depths.csv");

    std::ofstream gapFile;
    gapFile.open("all_gaps_file.csv");

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
        "Version: " RPLIDAR_SDK_VERSION "\n");

    // read serial port from the command line...
    if (argc > 1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc > 2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com7";
#elif __APPLE__
        opt_com_path = "/dev/tty.SLAB_USBtoUART";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver* drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if (useArgcBaudrate)
    {
        if (!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
        for (size_t i = 0; i < baudRateArraySize; ++i)
        {
            if (!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result))
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
        "Firmware Ver: %d.%02d\n"
        "Hardware Rev: %d\n"
        , devinfo.firmware_version >> 8
        , devinfo.firmware_version & 0xFF
        , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    //signal(SIGINT, ctrlr);

    drv->startMotor();
    // start scan...
    drv->startScan(0, 1);

    
    
    // fetech result and print it out...
    while (d < num_runs) {
        if (d == 0) {
            all_clear_count = 0;
        }
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        
        if (IS_OK(op_result)) {
        }
        else {
            printf("Bad Health Code - Rerun...\n");
        }// Check if reading is OK 
            drv->ascendScanData(nodes, count); // Iterate the driver
            for (int col = 0; col < sampleAvg; ++col) { // Run two for loops - Columns for each data point to be aggregated column-wise
                for (int pos = 0; pos < (int)count; ++pos) { // Row-wise iteration to store the data for a single revolution 
                    thetaArr[pos] = (nodes[pos].angle_z_q14 * 90.f / (1 << 14)); // Store angle of a data point in thetaARR (Note: we do not care to average the angle - hence the 1-d array)
                    readDistArr[pos][col] = (nodes[pos].dist_mm_q2 / 4.0f); // Store distance of a data point in readDistArr
                    readQualArr[pos][col] = nodes[pos].quality; // Store quality of a data point in readQualArr
                    
                        /* printf("%s Pos: %d Col: %d theta: %f Dist: %f Q: %f \n",
                            (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ", pos, col,
                            thetaArr[pos], readDistArr[pos][col], readQualArr[pos][col]); */
                        
                    
                }
                //printf("Col: %d Theta: %f Dist: %f Qual: %f \n", col, thetaArr[4], readDistArr[4][col], readQualArr[4][col]);
            }            
            
            //-------------AVG VALS------------//
            for (int row = 0; row < int(count); ++row) { //get the avg values
                sumDist[row] = { 0 }; //initialize an array to sum the distance points
                sumQual[row] = { 0 }; //initialize an array to sum the quality points
                for (int col = 0; col < sampleAvg - 1; ++col) { // iterate through however many samples we are averging
                    sumDist[row] = sumDist[row] + readDistArr[row][col]; //Math...
                    sumQual[row] = sumQual[row] + readQualArr[row][col];
                }
                distArr[row] = (sumDist[row] / (sampleAvg));// More math...
                qualArr[row] = (sumQual[row] / (sampleAvg));

                //-----------VALUES ARE AVERAGED-----------//
            } 
            // put bad then good scan data into csv file for demo //
            badScanData << "Theta, Dist(mm)(Unclean), \n";

            for (int i = 0; i < count; i++) {
                badScanData << thetaArr[i] << "," << distArr[i] << "," << std::endl;
            }
            //
            for (int i = 0; i < int(count); i++) { // clean data
                //printf("Point: %d TH: %f Dist: %f Q: %d \n", i, thetaArr[i], distArr[i], qualArr[i]);
                clean_avg_data(distArr, thetaArr, qualArr, i);
                //printf("Point: %d TH: %f Dist: %f Q: %d \n", i, thetaArr[i], distArr[i], qualArr[i]);
            }
            //
            
            badScanData << "Theta, Dist(mm)(Unclean), \n";
            for (int i = 0; i < count; i++) {
                goodScanData << thetaArr[i] << "," << distArr[i] << "," << std::endl;
            }

            // csv file writing should be done // 
            
            int startEdge = 0;
            int endEdge = 0;
            float tempGapDepth = 0;
            float tempGapWidth = 0;
            int edgeArr[arbSize][2] = {};
            int openAreaArr[arbSize][2] = {};
            int edgeRow = 0;
            int openAreaRow = 0;
            int tempEnd = 0;
            int wayPoints[1000] = {};
            int wpIndex = 0;
            float delta_theta = 0.0;
            int block_obj_count = 0;
            

            //JULIA LOOK HERE: adjustable parameters for ya
            int widthThreshold = 500; // 500 as in 0.5m (~1.5ft)
            int depthThreshold = 500; // not as important. Can omit depth check for increased accuracy but slower performance 
            int minObjDist = 3500 ; // distance for detecting obstacles ('x' mm or less is an object to avoid, beyond 'x' we ignore)
            
            /*
            for (int i = 0; i < count; i++) {
                if (distArr[i] <= 2000) {
                    block_obj_count++;
                }
            }*/
            //Begin pathfinding:
            for (int i = 0; i < int(count); i++) {
                //printf("th: %f \n", thetaArr[i]);
                if (is_path(distArr, minObjDist, i)) {
                    startEdge = i;
                    //printf("startEdge: %f \n", thetaArr[i]);
                    while (is_path(distArr, minObjDist, i)) {
                        // printf("is path at theta: %f \n", thetaArr[i]);
                        i++;
                    }
                    endEdge = i;

                    if (startEdge <= 1) { // case for gap at 0-360 mark
                        tempEnd = endEdge;
                    }
                    if (endEdge >= count - 1) {
                        endEdge = tempEnd;
                        edgeArr[0][0] = startEdge;
                    }


                    edgeArr[edgeRow][0] = startEdge;
                    edgeArr[edgeRow][1] = endEdge;
                    //printf("Endedge %f \n", thetaArr[endEdge]);
                    //printf("%d %f %d %f \n", startEdge, thetaArr[startEdge], endEdge, thetaArr[endEdge]);
                    edgeRow++;

                } // if point detected is not a valid path - do next
            }
	    //adding this comment to remake

            for (int i = 0; i < int(count); i++) {
                
                 //branch for identifing a gap with no return readings
                if (is_no_return(distArr, qualArr, i)) {
                    startEdge = i;
                    //printf("startEdge %f ", thetaArr[i]);
                    while (is_no_return(distArr, qualArr, i)) {
                        i++;
                    }
                    endEdge = i-1;
		    //printf("count: %d \n", count);
		    //printf("e: %d, s: %d \n", endEdge, startEdge);
		    //printf("HERE: %d \n", (endEdge - startEdge));
		    if ((endEdge - startEdge ) == 3000 ){
			    openAreaRow++;
		    }
                    //printf("endEdge %f \n", thetaArr[i]);

                    if (startEdge <= 1) { // account for bridge across 360-0 mark
                        tempEnd = endEdge; // set a temporary variable to the front end
                    }
                    if (endEdge >= count - 1) {
                        endEdge = tempEnd; // set the back end to the front end (Gap is bridged ! :)
                        openAreaArr[0][0] = startEdge;
                        //openAreaArr[openAreaRow][0] = 0;
                        //openAreaArr[openAreaRow][1] = 0;

                    }
                    openAreaArr[openAreaRow][0] = startEdge;
                    openAreaArr[openAreaRow][1] = endEdge;
                    //printf("%d %f %d %f \n", startEdge, thetaArr[startEdge], endEdge, thetaArr[endEdge]);
                   // openAreaRow++;
                }

                    // By this point all open area gaps should be found and edges stored in the open area array                

            } // end for each point in count nodes
            
            /*
            for (int i = 0; i < edgeRow; i++) {
                if (edgeRow >= 1) {
                    //printf("edgeRowHERE: %d \n", edgeRow);
                    if (edgeArr[edgeRow - 1][1] == edgeArr[0][1]) {
                        for (int i = 0; i < edgeRow; i++) {
                            edgeArr[i][0] = edgeArr[i + 1][0];
                            edgeArr[i][1] = edgeArr[i + 1][1];
                        }
                    }
                }
            }
            */
            /*if ((edgeArr[0][0] != edgeArr[edgeRow][0]) && (edgeArr[0][1] == edgeArr[edgeRow][1])) {
                edgeArr[0][0] = edgeArr[edgeRow][0];
                edgeArr[0][1] = edgeArr[edgeRow][1];
            }*/

            for (int i = 0; i < edgeRow; i++) {

                tempGapWidth = find_gap_width(distArr, thetaArr, edgeArr[i][0], edgeArr[i][1]); // asign temp var width
                tempGapDepth = find_gap_depth(distArr, thetaArr, edgeArr[i][0], edgeArr[i][1], count); // asign temp var depth 
                if (c < edgeRow) {
                    gapFile << thetaArr[edgeArr[i][0]] << "," << thetaArr[edgeArr[i][1]] << "," << std::endl;
                    c++;
                }
                delta_theta = abs(thetaArr[edgeArr[i][0]] - thetaArr[edgeArr[i][1]]);
                if (thetaArr[edgeArr[i][0]] > thetaArr[edgeArr[i][1]]) { 
                    delta_theta = 360 - delta_theta;
                }
                if (delta_theta > 40) {
                    wayPoints[wpIndex] = ((edgeArr[i][0] + edgeArr[i][1]) / 2); // set one waypoint to the midpoint of a valid gap
                    wpIndex++;
		    if (thetaArr[edgeArr[i][1]] > 400){
			   printf("0 359 \n");
		    }
		    else { 
                    	printf("%f %f \n", thetaArr[edgeArr[i][0]], thetaArr[edgeArr[i][1]]);
		    }
                }
            }

            //printf("open arr row: %d \n", openAreaRow);
            
            if (openAreaRow == 1) {
	//	printf("openAreaRow: %d \n", openAreaRow);
                all_clear_count++;
                if (all_clear_count >= num_runs) {
                    printf("0 359 \n");
                }
            }
            else {
                for (int i = 0; i < openAreaRow; i++) {
                    tempGapWidth = find_gap_width(distArr, thetaArr, openAreaArr[i][0], openAreaArr[i][1]); // asign temp var width
                    tempGapDepth = find_gap_depth(distArr, thetaArr, openAreaArr[i][0], openAreaArr[i][1], count); // asign temp var depth
                    //printf("width: %f \n", tempGapWidth);
                    delta_theta = abs(thetaArr[openAreaArr[i][0]] - thetaArr[openAreaArr[i][1]]);
                    if (thetaArr[openAreaArr[i][0]] > thetaArr[openAreaArr[i][1]]) {
                        delta_theta = 360 - delta_theta;
                    }
                    if (delta_theta >= 40) {
                        //printf("width: %f \n", tempGapWidth);
                        //printf("%f %f \n", thetaArr[openAreaArr[i][0]], thetaArr[openAreaArr[i][1]]);
                        wayPoints[wpIndex] = ((openAreaArr[i][0] + openAreaArr[i][1]) / 2);
                        wpIndex++;
                        //printf("WP Index: %d \n", wpIndex);
                    }
                    else {
                        //printf("gap too small: %f %f \n", thetaArr[openAreaArr[i][0]], thetaArr[openAreaArr[i][1]]);
                        /*if (tempGapWidth >= widthThreshold) {
                            //printf("openarrow: %d \n", openAreaRow);
                            printf("%f %f %f %f \n", thetaArr[openAreaArr[i][0]], distArr[openAreaArr[i][0]], thetaArr[openAreaArr[i][1]], distArr[openAreaArr[i][1]]);
                            wayPoints[wpIndex] = (count / 2);
                            wpIndex++;
                        }*/
                    }

                }
            }
            //all waypoints written to wayPoints[index]

            //remove zeroes for navigation
            /*
            for (int i = 0; i++; i < wpIndex) {
                if (distArr[wayPoints[i]] == 0) {
                    distArr[wayPoints[i]] = minObjDist + 250;
                }
            }
            */

            
            //write to waypoint file
            if (a < int(count)) { // write to csv file
                if (a < 1) { // title rows once
                    wpFile << "Waypoint, Waypoint Angle, \n";
                    widthFile << "Waypoint, Width(mm), \n";
                    depthFile << "Waypoint, depth(mm), \n";
                }
                for (int j = 0; j < wpIndex ; j++) {
                    if (b < wpIndex) {
                        wpFile << j << "," << thetaArr[wayPoints[j]] << "," << std::endl;
                        
                        widthFile << j << "," << find_gap_width(distArr, thetaArr, edgeArr[j][0], edgeArr[j][1]) << "," << std::endl;
                        depthFile << j << "," << find_gap_depth(distArr, thetaArr, edgeArr[j][0], edgeArr[j][1], count) << "," << std::endl;
                        b++;
                        //printf("b: %d, wpIndex: %d \n", b, wpIndex);

                        if (distArr[wayPoints[j]] == 0) {
                            distArr[wayPoints[j]] = minObjDist + 150;
                        }
                        //printf("%f %f \n", thetaArr[edgeArr[j][0]], thetaArr[edgeArr[j][1]]);
                        //printf("wpIndex %d \n", wpIndex);
                        //printf(" Valid waypoint at Theta: %f , Distance: %f , Waypoint %d \n", thetaArr[wayPoints[j]], distArr[wayPoints[j]], j); 
                    }
                }
                a++; //a++
            }
            
        //} //end if(IS_OK)



        //else { printf("Is_ok false \n"); }


        if (ctrl_c_pressed) {
            break;
        }
        printf("d: %d \n", d);
        d++;
    } // end while(1)
    printf("Done with 'while' loop - code should cease running \n");



    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}
