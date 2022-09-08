// SpyderGigECode.cpp : Este archivo contiene la función "main". La ejecución del programa comienza y termina ahí.
//

#pragma warning(disable: 4996)  /* unable sprintf warnings */

#include <iostream>
#include "stdio.h"
#include "conio.h"
#include "math.h"
#include "sapclassbasic.h"
#include "ExampleUtils.h"
#include <chrono>
#include <ctime>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/* Constants */
#define PROJECTDIR "C:\\Users\\dshatwell.HOCPLC\\source\\repos\\SpyderGigECode"
#define ROCKMAXWIDTH 160
#define ROCKMAXHEIGHT 160
#define SEGMENTATION_THRESHOLD 15
#define IMAGE_HEIGHT 1172

/* Static functions */
void ExportToOpenCV_Direct(SapBuffer* pSapBuf);
void ShowMatImage(Mat img);
bool SaveVideoRecorded();
bool RockDetected(Mat curimg, Mat bw);
void GetTimeBasedName(char* filename);
void writecsv(int idx, char* savepath);
void RockDivision(char* imgname, Mat img, Mat binimg);

/* Class functions */
static void XferCallback(SapXferCallbackInfo* pInfo);
static BOOL GetOptions(int argc, char* argv[], char* acqServerName, UINT32* pAcqDeviceIndex, char* configFileName);
static BOOL GetOptionsFromCommandLine(int argc, char* argv[], char* acqServerName, UINT32* pAcqDeviceIndex, char* configFileName);

/* Global Variables */
SapBuffer cmpBuffer;
vector<Mat> RockImagesBuffer;
vector<Mat> RockMasksBuffer;
vector<string> Imagenames;
int idxTotal = 0;
bool waitforcap = 0;
Mat lastimg;

int main(int argc, char* argv[])
{
    UINT32 acqDeviceNumber;
    char* acqServerName = new char[CORSERVER_MAX_STRLEN];
    char* configFilename = new char[MAX_PATH];

    printf("Spyder Console Grab Code (C++ version)\n");

    if (!GetOptions(argc, argv, acqServerName, &acqDeviceNumber, configFilename))
    {
        printf("\nPress any key to terminate\n");
        CorGetch();
        return 0;
    }

    SapAcquisition Acq;
    SapAcqDevice AcqDevice;
    SapBufferWithTrash Buffers;
    SapTransfer AcqToBuf = SapAcqToBuf(&Acq, &Buffers);
    SapTransfer AcqDeviceToBuf = SapAcqDeviceToBuf(&AcqDevice, &Buffers);
    SapTransfer* Xfer = NULL;
    SapView View;
    SapLocation loc(acqServerName, acqDeviceNumber);

    if (SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcq) > 0)
    {
        Acq = SapAcquisition(loc, configFilename);
        Buffers = SapBufferWithTrash(2, &Acq);
        View = SapView(&Buffers, SapHwndAutomatic);
        AcqToBuf = SapAcqToBuf(&Acq, &Buffers, XferCallback, &View);
        Xfer = &AcqToBuf;

        /* Create acquisition object */
        if (!Acq.Create())
            goto FreeHandles;
    }

    else if (SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcqDevice) > 0)
    {
        if (strcmp(configFilename, "NoFile") == 0)
            AcqDevice = SapAcqDevice(loc, FALSE);
        else
            AcqDevice = SapAcqDevice(loc, configFilename);

        Buffers = SapBufferWithTrash(2, &AcqDevice);
        cmpBuffer = SapBuffer(500, &AcqDevice);
        View = SapView(&Buffers, SapHwndAutomatic);
        AcqDeviceToBuf = SapAcqDeviceToBuf(&AcqDevice, &Buffers, XferCallback, &View);
        Xfer = &AcqDeviceToBuf;

        // Create acquisition object
        if (!AcqDevice.Create())
            goto FreeHandles;
    }

    /* Create buffer object */
    if (!Buffers.Create())
        goto FreeHandles;

    /* Create transfer object */
    if (Xfer && !Xfer->Create())
        goto FreeHandles;

    /* Grab activate a callback to get the current frame */
    Xfer->Grab();

    printf("Press any key to stop grab\n\n");
    CorGetch();

    /* Stop grab */
    Xfer->Freeze();
    if (!Xfer->Wait(5000))
        printf("Grab could not stop properly.\n");

    /* unregister the acquisition callback */
    Acq.UnregisterCallback();
  
    /* Save captures and crops */
    for (int k=0; k< Imagenames.size(); k++)
    {
        char savepath[MAX_PATH];
        char imgnametosave[MAX_PATH];
        strcpy(imgnametosave, Imagenames[k].c_str());
        sprintf(savepath, "%s%s%s", PROJECTDIR, "\\Videos\\", imgnametosave);
        imwrite(savepath, RockImagesBuffer[k]);
        RockDivision(imgnametosave, RockImagesBuffer[k], RockMasksBuffer[k]);
        writecsv(k, imgnametosave);
    }

FreeHandles:
    printf("Press any key to terminate\n");
    CorGetch();

    /* Destroy transfer object */
    if (Xfer && *Xfer && !Xfer->Destroy()) return FALSE;

    /* Destroy buffer object */
    if (!Buffers.Destroy()) return FALSE;

    /* Destroy acquisition object */
    if (!Acq.Destroy()) return FALSE;

    /* Destroy acquisition object */
    if (!AcqDevice.Destroy()) return FALSE;

    return 0;
}

static void XferCallback(SapXferCallbackInfo* pInfo)
{
    auto t0 = chrono::high_resolution_clock::now();
    SapView* pView = (SapView*)pInfo->GetContext();
    SapBuffer* pBuffer = pView->GetBuffer();

    /* procesing image and detect rock */
    ExportToOpenCV_Direct(pBuffer);

    auto elapsed = chrono::high_resolution_clock::now() - t0;
    long long microseconds = chrono::duration_cast<chrono::microseconds>(elapsed).count();

    //cout << "Processing time: " << microseconds << " us" << endl;

    /* refresh framerate */
    static float lastframerate = 0.0f;

    SapTransfer* pXfer = pInfo->GetTransfer();
    if (pXfer->UpdateFrameRateStatistics())
    {
        SapXferFrameRateInfo* pFrameRateInfo = pXfer->GetFrameRateStatistics();
        float framerate = 0.0f;

        if (pFrameRateInfo->IsLiveFrameRateAvailable())
        {
            framerate = pFrameRateInfo->GetLiveFrameRate();
            cmpBuffer.SetFrameRate(framerate);
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
}

static BOOL GetOptions(int argc, char* argv[], char* acqServerName, UINT32* pAcqDeviceIndex, char* configFileName)
{
    // Check if arguments were passed
    if (argc > 1)
        return GetOptionsFromCommandLine(argc, argv, acqServerName, pAcqDeviceIndex, configFileName);
    else
        return GetOptionsFromQuestions(acqServerName, pAcqDeviceIndex, configFileName);
}

static BOOL GetOptionsFromCommandLine(int argc, char* argv[], char* acqServerName, UINT32* pAcqDeviceIndex, char* configFileName)
{
    // Check the command line for user commands
    if ((strcmp(argv[1], "/?") == 0) || (strcmp(argv[1], "-?") == 0))
    {
        // print help
        printf("Usage:\n");
        printf("GrabCPP [<acquisition server name> <acquisition device index> <config filename>]\n");
        return FALSE;
    }

    // Check if enough arguments were passed
    if (argc < 4)
    {
        printf("Invalid command line!\n");
        return FALSE;
    }

    // Validate server name
    if (SapManager::GetServerIndex(argv[1]) < 0)
    {
        printf("Invalid acquisition server name!\n");
        return FALSE;
    }

    // Does the server support acquisition?
    int deviceCount = SapManager::GetResourceCount(argv[1], SapManager::ResourceAcq);
    int cameraCount = SapManager::GetResourceCount(argv[1], SapManager::ResourceAcqDevice);

    if (deviceCount + cameraCount == 0)
    {
        printf("This server does not support acquisition!\n");
        return FALSE;
    }

    // Validate device index
    if (atoi(argv[2]) < 0 || atoi(argv[2]) >= deviceCount + cameraCount)
    {
        printf("Invalid acquisition device index!\n");
        return FALSE;
    }

    // Verify that the specified config file exist
    OFSTRUCT of = { 0 };
    if (OpenFile(argv[3], &of, OF_EXIST) == HFILE_ERROR)
    {
        printf("The specified config file (%s) is invalid!\n", argv[3]);
        return FALSE;
    }

    // Fill-in output variables
    CorStrncpy(acqServerName, argv[1], CORSERVER_MAX_STRLEN);
    *pAcqDeviceIndex = atoi(argv[2]);
    CorStrncpy(configFileName, argv[3], MAX_PATH);

    return TRUE;
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

        RockDetected(rgbimg, binimg);
    }
}

void ShowMatImage(Mat img) 
{
    imshow("image", img);
    waitKey(1);
}

bool SaveVideoRecorded() 
{
    // Save video
    char m_Options[20];
    char filename[100];
    char savepath[MAX_PATH];
    time_t now;
    tm* ltm;

    printf("Complete Buff, start: %d, end: %d, fps/s: %f \n", 0, cmpBuffer.GetIndex(), cmpBuffer.GetFrameRate());
    sprintf(m_Options, "-format avi");
    sprintf(savepath, PROJECTDIR);

    now = time(0);
    ltm = localtime(&now);
    sprintf(filename, "\\Videos\\vid%d%d%d_%d%d%d.avi", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour - 5, ltm->tm_min, ltm->tm_sec);

    CorStrncat(savepath, filename, sizeof(savepath));
    printf("Saving video at: %s \n", savepath);
    return cmpBuffer.Save(savepath, m_Options, 0, cmpBuffer.GetIndex() + 1);
}

bool RockDetected(Mat curimg, Mat bw)
{
    /* end detection - no rock */
    if (curimg.empty())
        return FALSE;

    char name[100];
    GetTimeBasedName(name);

    if (!waitforcap) 
    {
        /* Find contours in binary image */
        vector<vector<Point>> contours;
        findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        int idx_maxcnt = 0;

        /* Get contours and the max area of the objects */
        for (int i = 0; i < contours.size(); i++) 
        {
            if (contourArea(contours[i]) > max_area)
            {
                max_area = contourArea(contours[i]);
                idx_maxcnt = i;
            }
        }

        /* Detect if rock is present, store the image and timestamp */
        if (max_area >= 90)
        {
            Rect bbxdata;
            bbxdata = boundingRect(contours[idx_maxcnt]);

            if (bbxdata.y + bbxdata.height == bw.size().height)
            {
                cout << "rock in the limit, wait for the next capture" << endl;
                lastimg = curimg;
                waitforcap = TRUE;
            }
            else 
            {
                RockImagesBuffer.push_back(curimg); 
                RockMasksBuffer.push_back(bw);
                cout << idxTotal << " ==> " << name << "    maxArea: " << max_area << endl;
                Imagenames.push_back(name);                
                idxTotal++;
            }
        }       

        /* show image  */
        ShowMatImage(bw);
    }
    else /* case control: rock divided in two images*/
    {
        Mat mergeimg, newimg, newbw, splitimg[3], grayimg;

        /* Merge two frames vertically */
        vconcat(lastimg, curimg, mergeimg);
        newimg = mergeimg(Rect(0, IMAGE_HEIGHT/2, curimg.size().width, curimg.size().height));

        split(newimg, splitimg);
        grayimg = splitimg[1];

        threshold(grayimg, newbw, SEGMENTATION_THRESHOLD, 255, THRESH_BINARY);
        
        cout << "Merge completed " << endl;
        cout << "New image saved: " << idxTotal << " ==> " << name << endl;
        cout << "Flag turned off" << endl;

        RockImagesBuffer.push_back(newimg);
        RockMasksBuffer.push_back(newbw);
        Imagenames.push_back(name);
        
        waitforcap = FALSE;
        idxTotal++;
    }

    return TRUE;
}

void GetTimeBasedName(char* filename) 
{
    /* Get current date stamp */
    time_t now;
    tm* ltm;

    now = time(0);
    ltm = localtime(&now);
    sprintf(filename, "img%d%d%d_%d%d%d.tiff", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour - 5, ltm->tm_min, ltm->tm_sec);
}

void writecsv(int idx, char* savepath) 
{
    fstream fout;
    char configPath[MAX_PATH];
    sprintf(configPath, PROJECTDIR);
    CorStrncat(configPath, "\\imgreport.csv", sizeof(configPath));
    
    /* Open or Create csv file */
    fout.open(configPath, ios::out | ios::app);

    /* Write the idx image and the image name */
    fout<< idx << "," <<savepath << "\n";
}

void RockDivision(char* imgname, Mat img, Mat binimg) 
{
    /* Find contours in the binary image */
    vector<vector<Point>> contours;
    findContours(binimg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
            cropbinimg = binimg(bbxdata).clone();

            /* most rigthbottom point */
            int rb_x = bbxdata.x + bbxdata.width;
            int rb_y = bbxdata.y + bbxdata.height;

            /* dimension differences */
            int widthdif = ROCKMAXWIDTH - bbxdata.width;
            int heightdif = ROCKMAXHEIGHT - bbxdata.height;
            int halfwdif = widthdif / 2;
            int halfhdif = heightdif / 2;

            /* set max dimensions */
            bbxdata.width = ROCKMAXWIDTH;
            bbxdata.height = ROCKMAXHEIGHT;

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

            /* create mask and crop RGB-image - rock in the most left top zone */
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
            
            sprintf(maskpath, "%s%s%s_%d%s",PROJECTDIR,"\\Masks\\",strtok(imgname,"."),nrocks,".tiff");
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