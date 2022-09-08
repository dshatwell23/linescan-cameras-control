/*
C2 IMAGE ACQUISITION THREAD
---------------------------

This program is a test used to acquire images with the C2-2040 using multithreading

Created by: David Shatwell & Hugo Negreyros
Date: 19/08/2022

Modified by: Hugo Negreyros
Date: 22/08/2022
*/

#pragma warning(disable: 4996)

#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <opencv2/opencv.hpp> 
#include "stdio.h"
#include "conio.h"
#include "math.h"
#include "sapclassbasic.h"
#include "ExampleUtils.h"
#include "AT/cx/CVUtils.h"
#include "cx_cam_common.h"

using namespace AT;
using namespace cv;
using namespace std;
using namespace cx;

/* Constants */
static const char* PROJECTDIR = "C:\\Users\\dshatwell.HOCPLC\\source\\repos\\MultiThreadCamControl";
static const char* SPYDERCONFIGFILE = "C:\\Program Files\\Teledyne DALSA\\Sapera\\CamFiles\\User\\T_Spyder_GigE_Colour_Camera_Last_ColorAdjust.ccf";
//static const int MAX_PATH = 260;  // already defined in minwindef.h
//static const int IMAGE_HEIGHT = 512;
//static const int IMAGE_WIDTH = 1344;

static const int SEGMENTATION_THRESHOLD = 15;
static const int AREA_THRESHOLD = 200;
static const int NUM_DATA_CHANNELS = 3;
static const long long TIME_BETWEEN_CAMERAS = 98148; // us
static const long long TOTAL_PERIOD = 370688; // us
static const long long STABILIZATION_PERIOD = 5000000; // us
/* CAM IDs */
static const int C2ID = 0;
static const int SPYDERID = 1;


/* Flags */
static bool s_Finished = false;

/* Global variables */
bool waitforcap = 0;
Mat lastimg, lookUpTable;
vector<Mat> RockImagesBuffer;
vector<Mat> RockMasksBuffer;
vector<string> Imagenames;
vector<int> SeqCamID;
int ROCK_MAX_HEIGHT;
int ROCK_MAX_WIDTH;
std::chrono::steady_clock::time_point t0;
std::chrono::steady_clock::time_point t1;
long long microseconds = 0;
DevicePtr cam;

bool flagshow = 1;
SapAcquisition Acq;
SapAcqDevice AcqDevice;
SapBufferWithTrash Buffers;
SapTransfer AcqToBuf = SapAcqToBuf(&Acq, &Buffers);
SapTransfer AcqDeviceToBuf = SapAcqDeviceToBuf(&AcqDevice, &Buffers);
SapTransfer* Xfer = NULL;
SapView View;

/* Function prototypes */
void C2ImageAcquisition();
Mat createLUT(const int threshold);
void getChannels(const ImagePtr image, Mat* channels);
Mat getFrameFromChannels(Mat* channels);
bool compareArea(const vector<Point>& cont1, const vector<Point>& cont2);
void GetTimeBasedName(char* filename, int camID);
bool RockDetected(Mat curimg, Mat bw, int camID);
void RockDivision(char* imgname, Mat img, Mat mask, int camID);

void Spyder3ImageAcquisition();
void ExportToOpenCV_Direct(SapBuffer* pSapBuf);

/* Static class functions */
static void XferCallback(SapXferCallbackInfo* pInfo);
//
//bool OpenSpyderCam() 
//{
//       
//}


int main()
{
    /* Connect C2 camera  */
    cout << "Open and configure C2 cam" << endl;
    //string uri = "gev://192.168.87.60/?mac=00-50-c2-8e-d7-67&nif=38-F3-AB-15-92-40";
    string uri = "gev://169.254.169.169/?mac=00-50-c2-8e-d7-67&nif=DC-4A-3E-75-1D-E4";
    cam = DeviceFactory::openDevice(uri);
    cout << "Open Device: " << uri.c_str() << endl;

    /* Set transport layer parameters */
    cam->setParam("GevSCPSPacketSize", 8192);
    cam->setParam("GevSCPD", 0);

    /* Allocate and queue internal acquisition buffers */
    cam->allocAndQueueBuffers(3);

    /* Start acquisition */
    cam->startAcquisition();

    /* Creare LUT for efficient segmentation */
    lookUpTable = createLUT(SEGMENTATION_THRESHOLD);
    cout << "LUT created" << endl;

    cout << "C2 cam ready ..." << endl << endl;

    /* Connect and configure Spyder camera */
    cout << "Open and configure Spyder cam" << endl;
    UINT32 acqDeviceNumber = 0;
    char* acqServerName = new char[CORSERVER_MAX_STRLEN];
    char findPath[MAX_PATH];

    /* Establish server name */
    for (int serverIndex = 0; serverIndex < SapManager::GetServerCount(); serverIndex++) 
    {
        char serverName[CORSERVER_MAX_STRLEN];
        SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));       
        if (strncmp(serverName, "Spyder_GigE_Colour_Camera_x",25) == 0)
        {
            sprintf(acqServerName, "%s", serverName);
        }
    }
    
    cout << "Open Server: " << acqServerName << endl;

    /* Allocate server and device */
    SapLocation loc(acqServerName, acqDeviceNumber);

    /* Load configuration file */
    AcqDevice = SapAcqDevice(loc, SPYDERCONFIGFILE);

    /* Allocate buffers and callback */
    Buffers = SapBufferWithTrash(2, &AcqDevice);
    View = SapView(&Buffers, SapHwndAutomatic);
    AcqDeviceToBuf = SapAcqDeviceToBuf(&AcqDevice, &Buffers, XferCallback, &View);
    Xfer = &AcqDeviceToBuf;

    /* Create acquisition object */
    if (!AcqDevice.Create())
        return -1;

    /* Create buffer object */
    if (!Buffers.Create())
        return -1;

    /* Create transfer object */
    if (Xfer && !Xfer->Create())
        return -1;

    cout << "Spyder cam ready ..." << endl << endl;

    /* Create thread for image acquisition */
    std::thread C2Worker(C2ImageAcquisition);
    std::thread Spyder3Worker(Spyder3ImageAcquisition);

    cin.get();
    s_Finished = true;

    C2Worker.join();
    Spyder3Worker.join();
    cout << "Finished" << std::endl;

    /* Stop acquisition */
    cam->stopAcquisition();

    /* Stop grab */
    Xfer->Freeze();
    if (!Xfer->Wait(5000))
        printf("Grab could not stop properly.\n");

    /* Unregister the acquisition callback */
    Acq.UnregisterCallback();

    /* Save captures and crops */
    for (int k = 0; k < Imagenames.size(); k++)
    {
        char framepath[MAX_PATH];
        char imgnametosave[MAX_PATH];
        strcpy(imgnametosave, Imagenames[k].c_str());
        sprintf(framepath, "%s%s%s", PROJECTDIR, "\\Captures\\", imgnametosave);
        imwrite(framepath, RockImagesBuffer[k]);
        //RockDivision(imgnametosave, RockImagesBuffer[k], RockMasksBuffer[k], SeqCamID[k]);
    }

    /* Cleanup */
    cam->freeBuffers();
    cam->close();

    /* Destroy transfer object */
    if (Xfer && *Xfer && !Xfer->Destroy()) return FALSE;

    /* Destroy buffer object */
    if (!Buffers.Destroy()) return FALSE;

    /* Destroy acquisition object */
    if (!Acq.Destroy()) return FALSE;

    /* Destroy acquisition object */
    if (!AcqDevice.Destroy()) return FALSE;
}


/* FUNCTIONS */
void C2ImageAcquisition()
{
    /* Print thread info */
    cout << "C2-2040 thread: " << std::this_thread::get_id() << endl;

    /* Start timer */
    t0 = std::chrono::high_resolution_clock::now();

    /* Wait for framerate to stabilize */
    microseconds = 0;
    while (microseconds < STABILIZATION_PERIOD) {
        auto elapsed = std::chrono::high_resolution_clock::now() - t0;
        microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    }

    cout << "C2 Cam is Stable..." << endl;

    /* Acquisition loop */
    while (!s_Finished)
    {
        // Grab acquisition buffer, wait for valid buffer with optional timeout. Timeout is given in ms.
        DeviceBuffer buffer = cam->waitForBuffer(5000);

        // Get image data from buffer and do some processing on the image data (or get a copy for later use)
        // \note img holds a reference to the image data in the DeviceBuffer, if you need the image data after cx_queueBuffer you need to clone the image!
        auto image = buffer.getImage();

        /* Convert image to OpenCV format and get individual channels */
        Mat channels[3];
        getChannels(image, channels);

        // Create initial mask using height image
        Mat bw(image->height(), image->width(), CV_8UC1);
        LUT(channels[2], lookUpTable, bw);

        /* Get frame from individual channels */
        auto frame = getFrameFromChannels(channels);

        RockDetected(frame, bw, C2ID);

        //imshow("Frame", bw);

        //if (waitKey(1) != -1)
        //    break;

        // Queue back the buffer to the devices acquisition engine.
        // \note From now on any references to the buffer images are not valid anymore and might be overwritten with new image data at any time!
        buffer.queueBuffer();
    }
}

void Spyder3ImageAcquisition()
{
    /* Print thread info */
    cout << "Spyder3 thread: " << std::this_thread::get_id() << endl;

    /* Grab activate a callback to get the current frame */
    Xfer->Grab();

    while (!s_Finished) {}
}


static void XferCallback(SapXferCallbackInfo* pInfo)
{

    SapView* pView = (SapView*)pInfo->GetContext();
    SapBuffer* pBuffer = pView->GetBuffer();

    /* procesing image and detect rock */
    ExportToOpenCV_Direct(pBuffer);

    /* refresh framerate */
    static float lastframerate = 0.0f;

    SapTransfer* pXfer = pInfo->GetTransfer();
    float framerate = 0.0f;
    if (pXfer->UpdateFrameRateStatistics())
    {
        SapXferFrameRateInfo* pFrameRateInfo = pXfer->GetFrameRateStatistics();

        if (pFrameRateInfo->IsLiveFrameRateAvailable())
        {
            framerate = pFrameRateInfo->GetLiveFrameRate();
        }

        /* check if frame rate is stalled */
        if (pFrameRateInfo->IsLiveFrameRateStalled())
        {
            printf("Live frame rate is stalled.\n");
        }
        /* update FPS only if the value changed by +/- 0.1 */
        else if ((framerate > 0.0f) && (abs(lastframerate - framerate) > 0.1f))
        {
            printf("Grabbing at %.1f frames/sec\n", framerate);
            lastframerate = framerate;
        }
    }

    if (framerate >= 2.65 && framerate <= 2.75 && flagshow)
    {
        t1 = chrono::high_resolution_clock::now();

        cout << "Stable framerate, wainting 98ms" << endl;
        pXfer->Freeze();
        pXfer->Abort();

        long long microseconds2 = 0;

        while (microseconds2 < 98000)
        {
            auto Ecount = chrono::high_resolution_clock::now() - t1;
            microseconds2 = chrono::duration_cast<chrono::microseconds>(Ecount).count();
        }
        pXfer->Grab();
        flagshow = 0;
    }
}

void ExportToOpenCV_Direct(SapBuffer* pSapBuf)
{
    if (pSapBuf == NULL)
        return;

    SapFormat sapFormat = pSapBuf->GetFormat();
    int OpenCV_Type = CV_8UC3;

    if (sapFormat != SapFormatUnknown)
    {
        /* Export to OpenCV Mat object using SapBuffer data directly */
        void* pBuf = NULL;
        pSapBuf->GetAddress(&pBuf);
        Mat exportImg(pSapBuf->GetHeight(), pSapBuf->GetWidth(), OpenCV_Type, pBuf);
        pSapBuf->ReleaseAddress(&pBuf);

        /* Convert from BRG to RGB Opencv image */
        Mat splitimg[3], grayimg, binimg, rgbimg;
        cvtColor(exportImg, rgbimg, COLOR_BGR2RGB);

        /* Green channel extraction */
        split(rgbimg, splitimg);
        grayimg = splitimg[1];

        /* Binarization */
        threshold(grayimg, binimg, SEGMENTATION_THRESHOLD, 255, THRESH_BINARY);

        RockDetected(rgbimg, binimg, SPYDERID);
    }
}


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

void GetTimeBasedName(char* filename, int camID)
{
    time_t now;
    tm* ltm;

    now = time(0);
    ltm = localtime(&now);

    char camName[CORSERVER_MAX_STRLEN];

    switch (camID) 
    {
        case 0:
            sprintf(camName, "C2Cam");
            break;
        case 1:
            sprintf(camName, "SpyderCam");
            break;
        default:
            sprintf(camName, "Unkwnown");
    }

    sprintf(filename, "%s_img%d%d%d_%d%d%d.tiff", camName, 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour - 5, ltm->tm_min, ltm->tm_sec);
}

bool RockDetected(Mat frame, Mat bw, int camID)
{
    /* end detection - no rock */
    if (frame.empty())
    {
        cout << "ERROR: Empty frame" << endl;
        return false;
    }
        
    char framename[MAX_PATH];
    GetTimeBasedName(framename, camID);

    if (!waitforcap)
    {
        /* Find contours */
        vector<vector<Point>> contours;
        findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        /* Create mask from largest contour */
        sort(contours.begin(), contours.end(), compareArea);
        Mat mask = Mat::zeros(frame.size().height, frame.size().width, CV_8UC1);
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
                SeqCamID.push_back(camID);
                Imagenames.push_back(framename);
            }
        }
    }
    else
    {
        Mat mergeimg, newimg, splitimg[3];
        vconcat(lastimg, frame, mergeimg);
        newimg = mergeimg(Rect(0, frame.size().height/2, frame.size().width, frame.size().height));

        split(newimg, splitimg);

        /* Create initial mask using height image */
        Mat bw(frame.size().height, frame.size().width, CV_8UC1);
        LUT(splitimg[2], lookUpTable, bw);

        cout << "Merge completed " << endl;
        cout << "New image saved ==> " << framename << endl;
        cout << "Flag turned off" << endl;

        RockImagesBuffer.push_back(newimg);
        RockMasksBuffer.push_back(bw);
        SeqCamID.push_back(camID);
        Imagenames.push_back(framename);
        waitforcap = false;
    }

    return true;
}

void RockDivision(char* imgname, Mat img, Mat mask, int camID)
{
    switch (camID)
    {
    case 0:
        ROCK_MAX_HEIGHT = 110;
        ROCK_MAX_WIDTH = 251;
        break;
    case 1:
        ROCK_MAX_HEIGHT = 160;
        ROCK_MAX_WIDTH = 160;
        break;
    default:
        ROCK_MAX_HEIGHT = 110;
        ROCK_MAX_WIDTH = 251;
    }

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

            sprintf(maskpath, "%s%s%s%s_%d%s", PROJECTDIR, strtok(imgname, "_"),"\\Masks\\", strtok(imgname, "."), nrocks, ".tiff");
            sprintf(cropspath, "%s%s%s%s_%d%s", PROJECTDIR, strtok(imgname, "_"),"\\Crops\\", strtok(imgname, "."), nrocks, ".tiff");

            sprintf(cmaskpath, "%s%s%s%s_%d%s", PROJECTDIR, strtok(imgname, "_"),"\\CMasks\\", strtok(imgname, "."), nrocks, ".tiff");
            sprintf(ccropspath, "%s%s%s%s_%d%s", PROJECTDIR, strtok(imgname, "_"),"\\CCrops\\", strtok(imgname, "."), nrocks, ".tiff");

            imwrite(maskpath, paddimg);
            imwrite(cropspath, croprgbimg);

            imwrite(cmaskpath, centercropbinimg);
            imwrite(ccropspath, centercropimg);

            nrocks++;
        }
    }
}
