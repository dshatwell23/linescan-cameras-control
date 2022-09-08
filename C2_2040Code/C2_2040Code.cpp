/*
C2 AUTO IMAGE ACQUISITION
-------------------------

This program is used to store images taken from the C2-2040 camera. The following images are stored in memory:
- Full image
- Cropped image of every rock, located in the top left corner
- Cropped image of every rock, located in the center
- Cropped mask of every rock, located in the top left corner
- Cropped mask of every rock, located in the center

Created by: David Shatwell
Date: 14/07/2022

Modified by: David Shatwell & Hugo Negreyros
Date: 15/07/2022
*/

#pragma warning(disable: 4996)

/* Include libraries */
#include <iostream>
#include <algorithm>
#include <string>
#include "AT/cx/CVUtils.h"
#include "cx_cam_common.h"
#include <ctime>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace AT;
using namespace cv;
using namespace std;
using namespace cx;

/* Constants */
static const char* PROJECTDIR = "C:\\Users\\dshatwell.HOCPLC\\source\\repos\\C2_2040Code";
static const int MAX_PATH = 260;
static const int IMAGE_HEIGHT = 512;
static const int IMAGE_WIDTH = 1344;
static const int ROCK_MAX_HEIGHT = 110;
static const int ROCK_MAX_WIDTH = 251;
static const int SEGMENTATION_THRESHOLD = 15;
static const int AREA_THRESHOLD = 200;
static const int NUM_DATA_CHANNELS = 3;

/* Global variables */
bool waitforcap = 0;
Mat lastimg, lookUpTable;
vector<Mat> RockImagesBuffer;
vector<Mat> RockMasksBuffer;
vector<string> Imagenames;

/* Function prototypes */
Mat createLUT(const int threshold);
void getChannels(ImagePtr image, Mat* channels);
Mat getFrameFromChannels(Mat* channels);
bool compareArea(const vector<Point>& cont1, const vector<Point>& cont2);
void GetTimeBasedName(char* filename);
bool RockDetected(Mat curimg, Mat bw);
void RockDivision(char* imgname, Mat img, Mat mask);

/* Main function */
int main()
{
    /* Connect camera */
    //string uri = "gev://192.168.87.60/?mac=00-50-c2-8e-d7-67&nif=38-F3-AB-15-92-40";  
    string uri = "gev://192.168.87.60/?mac=00-50-c2-8e-d7-67&nif=DC-4A-3E-75-1D-E5";
    auto cam = DeviceFactory::openDevice(uri);
    cout << "Open Device: " << uri.c_str() << endl;

    /* Set transport layer parameters */
    cam->setParam("GevSCPSPacketSize", 8192);
    cam->setParam("GevSCPD", 0);

    //Variant val;
    //cam->getParam("Width", val);
    //
    //cout << (int)val << endl;
    
    // Allocate and queue internal acquisition buffers 
    cam->allocAndQueueBuffers(3);
    cout << "Buffers allocated" << endl;

    // Start image acquisition
    cam->startAcquisition();
    cout << "Acquisition started" << endl;

    // Creare LUT for efficient segmentation
    lookUpTable = createLUT(SEGMENTATION_THRESHOLD);
    cout << "LUT created" << endl;

    cout << "Press any key in the OpenCV window to exit ... \n";

    cout << endl;

    // Acquisition loop
    while (true) {
        auto t0 = chrono::high_resolution_clock::now();
        // Grab acquisition buffer, wait for valid buffer with optional timeout. Timeout is given in ms.
        DeviceBuffer buffer = cam->waitForBuffer(5000);

        // Get image data from buffer and do some processing on the image data (or get a copy for later use)
        // \note img holds a reference to the image data in the DeviceBuffer, if you need the image data after cx_queueBuffer you need to clone the image!
        auto image = buffer.getImage();
    
        /* Convert image to OpenCV format and get individual channels */
        Mat channels[3];
        getChannels(image, channels);

        // Create initial mask using height image
        Mat bw(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        LUT(channels[2], lookUpTable, bw);

        /* Get frame from individual channels */
        auto frame = getFrameFromChannels(channels);

        RockDetected(frame, bw);

        auto elapsed = chrono::high_resolution_clock::now() - t0;
        long long microseconds = chrono::duration_cast<chrono::microseconds>(elapsed).count();
        cout << 1000000.0 / microseconds << endl;

        // Display resulting frame
        imshow("Mask", bw);
        imshow("Frame", frame);

        // Press  ESC on keyboard to  exit
        if (waitKey(1) != -1)
            break;

        // 6. Queue back the buffer to the devices acquisition engine.
        // \note From now on any references to the buffer images are not valid anymore and might be overwritten with new image data at any time!
        buffer.queueBuffer();
    }

    /* Save captures and crops */
    for (int k = 0; k < Imagenames.size(); k++)
    {
        char framepath[MAX_PATH];
        char imgnametosave[MAX_PATH];
        strcpy(imgnametosave, Imagenames[k].c_str());
        sprintf(framepath, "%s%s%s", PROJECTDIR, "\\Captures\\", imgnametosave);
        imwrite(framepath, RockImagesBuffer[k]);
        RockDivision(imgnametosave, RockImagesBuffer[k], RockMasksBuffer[k]);
    }

    /* Stop acquisition */
    cam->stopAcquisition();

    /* Cleanup */
    cam->freeBuffers();
    cam->close();

}

/* FUNCTIONS */
Mat createLUT(const int threshold) {
    uchar maskTable[256];
    for (int i = 0; i < 256; i++)
        maskTable[i] = (i >= threshold) ? 255 : 0;
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; i++)
        p[i] = maskTable[i];
    return lookUpTable;
}

void getChannels(const ImagePtr image, Mat* channels) {
    
    // Create 3-channel image
    Mat imgmat(image->height() / NUM_DATA_CHANNELS, image->width() * NUM_DATA_CHANNELS, cvUtils::pf2cv(image->pixelFormat()), image->data());

    for (int ch = 0; ch < 3; ch++) 
    {
        Mat DC(image->height(), image->width(), CV_16UC1);
        Rect chcrop(image->width() * ch, 0, image->width(), image->height() / NUM_DATA_CHANNELS);
        DC = Mat(imgmat, chcrop);

        if (ch == 0) 
        {
            DC.convertTo(DC, CV_8U, 255.0 / 1023.0);
        }
        else
        {
            DC.convertTo(DC, CV_8U);
        }
        channels[ch] = DC;
    }
}

Mat getFrameFromChannels(Mat* channels) {
    Mat frame;
    merge(channels, 3, frame);
    return frame;
}

bool compareArea(const vector<Point>& cont1, const vector<Point>& cont2) {
    /* Function used to sort contours by area, from bigger to smallest */
    double i = fabs(contourArea(Mat(cont1)));
    double j = fabs(contourArea(Mat(cont2)));
    return i > j;
}

void GetTimeBasedName(char* filename)
{
    time_t now;
    tm* ltm;

    now = time(0);
    ltm = localtime(&now);
    sprintf(filename, "img%d%d%d_%d%d%d.tiff", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour - 5, ltm->tm_min, ltm->tm_sec);
}

bool RockDetected(Mat frame, Mat bw)
{
    /* end detection - no rock */
    if (frame.empty())
        return false;

    char framename[MAX_PATH];
    GetTimeBasedName(framename);

    if (!waitforcap)
    {
        /* Find contours */
        vector<vector<Point>> contours;
        findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        /* Create mask from largest contour */
        sort(contours.begin(), contours.end(), compareArea);
        Mat mask = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        drawContours(mask, contours, 0, 1, FILLED);

        /* Save frame if rock present in the frame */
        if (sum(mask)[0] >= AREA_THRESHOLD) 
        {
            Rect bbxdata;
            bbxdata = boundingRect(contours[0]);
            if (bbxdata.y + bbxdata.height == bw.size().height)
            {
                cout << "rock in the limit, wait for the next capture" << endl;
                lastimg = frame;
                waitforcap = true;
            }
            else 
            {
                cout << "Image saved ==> " << framename << endl;
                RockImagesBuffer.push_back(frame);
                RockMasksBuffer.push_back(bw);
                Imagenames.push_back(framename);
            }
        }
    }
    else 
    {
        Mat mergeimg, newimg, splitimg[3];
        vconcat(lastimg, frame, mergeimg);
        newimg = mergeimg(Rect(0, 255, frame.size().width, frame.size().height));
        
        split(newimg, splitimg);

        /* Create initial mask using height image */
        Mat bw(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        LUT(splitimg[2], lookUpTable, bw);

        cout << "Merge completed " << endl;
        cout << "New image saved ==> " << framename << endl;
        cout << "Flag turned off" << endl;

        RockImagesBuffer.push_back(newimg);
        RockMasksBuffer.push_back(bw);
        Imagenames.push_back(framename);
        waitforcap = false;
    }

    return true;
}

void RockDivision(char* imgname, Mat img, Mat mask)
{
    /* Find contours in the binary image */
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int nrocks = 0;

    for (int i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) > 90)
        {
            Rect bbxdata;
            Mat paddimg, cropbinimg, centercropbinimg, centercropimg, croprgbimg;

            /* get boundingbox of the contour */
            bbxdata = boundingRect(contours[i]);
            int x = bbxdata.x;
            int y = bbxdata.y;

            /* crop limited rock */
            cropbinimg = mask(bbxdata).clone();

            /* most rigthbottom point */
            int rb_x = bbxdata.x + bbxdata.width;
            int rb_y = bbxdata.y + bbxdata.height;

            /* dimension differences */
            int widthdif = ROCK_MAX_WIDTH - bbxdata.width;
            int heightdif = ROCK_MAX_HEIGHT - bbxdata.height;
            int halfwdif = widthdif / 2;
            int halfhdif = heightdif / 2;

            /* set max dimensions */
            bbxdata.width = ROCK_MAX_WIDTH;
            bbxdata.height = ROCK_MAX_HEIGHT;

            /* TOP LEFT crop control */
            int rightpad = 0, leftpad = 0, bottompad = 0, toppad = 0;

            if (rb_x + widthdif <= img.size().width)
            {
                rightpad = widthdif;
            }
            else
            {
                rightpad = img.size().width - rb_x;
                leftpad = widthdif - rightpad;
                bbxdata.x -= leftpad;
            }

            if (rb_y + heightdif <= img.size().height)
            {
                bottompad = heightdif;
            }
            else
            {
                bottompad = img.size().height - rb_y;
                toppad = heightdif - bottompad;
                bbxdata.y -= toppad;
            }

            /* create mask and crop RGB-image - rock at the most left top zone */
            copyMakeBorder(cropbinimg, paddimg, toppad, bottompad, leftpad, rightpad, BORDER_CONSTANT, Scalar::all(0));
            croprgbimg = img(bbxdata).clone();

            /* CENTER crop control */
            if (widthdif < 32)
            {
                rightpad = halfwdif;
            }
            else
            {
                rightpad = (halfwdif < 32) ? 32 : (halfwdif / 32) * 32;
            }

            if (rb_x + rightpad > img.size().width)
            {
                rightpad = img.size().width - rb_x;
            }

            leftpad = widthdif - rightpad;

            if ((x - leftpad) < 0)
            {
                rightpad += leftpad - x;
                leftpad = x;
                bbxdata.x = 0;
            }
            else
            {
                bbxdata.x = x - leftpad;
            }

            if (heightdif < 32)
            {
                bottompad = halfhdif;
            }
            else
            {
                bottompad = (halfhdif < 32) ? 32 : (halfhdif / 32) * 32;
            }

            if (rb_y + bottompad > img.size().height)
            {
                bottompad = img.size().height - rb_y;
            }

            toppad = heightdif - bottompad;

            if ((y - toppad) < 0)
            {
                bottompad += toppad - y;
                toppad = y;
                bbxdata.y = 0;
            }
            else
            {
                bbxdata.y = y - toppad;
            }

            /* create mask and crop RGB-image - rock at center zone */
            copyMakeBorder(cropbinimg, centercropbinimg, toppad, bottompad, leftpad, rightpad, BORDER_CONSTANT, Scalar::all(0));
            centercropimg = img(bbxdata);

            /* save */
            char maskpath[MAX_PATH];
            char cropspath[MAX_PATH];

            char cmaskpath[MAX_PATH];
            char ccropspath[MAX_PATH];

            sprintf(maskpath, "%s%s%s_%d%s", PROJECTDIR, "\\Masks\\", strtok(imgname, "."), nrocks, ".tiff");
            sprintf(cropspath, "%s%s%s_%d%s", PROJECTDIR, "\\Crops\\", strtok(imgname, "."), nrocks, ".tiff");

            sprintf(cmaskpath, "%s%s%s_%d%s", PROJECTDIR, "\\CMasks\\", strtok(imgname, "."), nrocks, ".tiff");
            sprintf(ccropspath, "%s%s%s_%d%s", PROJECTDIR, "\\CCrops\\", strtok(imgname, "."), nrocks, ".tiff");

            imwrite(maskpath, paddimg);
            imwrite(cropspath, croprgbimg);

            imwrite(cmaskpath, centercropbinimg);
            imwrite(ccropspath, centercropimg);

            nrocks++;
        }
    }
}