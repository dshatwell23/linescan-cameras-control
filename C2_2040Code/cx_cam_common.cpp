/*************************************************************************************
THIS SOFTWARE IS PROVIDED BY AUTOMATION TECHNOLOGY GMBH "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUTOMATION TECHNOLOGY GMBH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*************************************************************************************/

#include <string>
#include <regex>
#include <iostream>
using namespace std;

#include "cx_cam_common.h"

namespace AT {
    namespace cx {

        DeviceInfoPtr discoverAndChooseDevice(bool useFilterDriver)
        {
            cx::DeviceInfoList devList = cx::DeviceFactory::findDevices();
            if (devList.size() > 0)
            {
                // list found devices
                std::cout << "Devices found: " << endl;
                for (unsigned int i = 0; i < devList.size(); i++)
                    std::cout << "  # " << i << ", Model: " << devList[i]->deviceModel << ", Name: " << devList[i]->deviceUserID << ", URI: " << devList[i]->deviceURI << endl;
                std::cout << endl;

                while (1)
                {
                    // query user for device index
                    std::string userInput;
                    std::cout << "Choose device index or 'c' for cancel: ";
                    cin >> userInput;
                    if (std::regex_search(userInput, std::regex("[cCqQ]")))
                        exit(-1);

                    if (std::regex_search(userInput, std::regex("[0-9]")))
                    {
                        unsigned devIdx = std::stoi(userInput);
                        if (devIdx < devList.size())
                        {
                            if (useFilterDriver == false)
                                devList[devIdx]->deviceURI += "&fd=false";
                            return devList[devIdx];
                        }
                    }
                }
            }
            std::cerr << "No Camera Devices found." << endl;
            exit(-2);
        }

        void printParamInfo(cx::DevicePtr dev, const std::string& name, std::ostream& outstr)
        {
            cx::Variant val, type, range, descr, tooltip, access, visibility, sym_entries, int_value, cat_children;
            val = "None";

            descr = std::string("");
            tooltip = std::string("");

            dev->getParamInfo(CX_PARAM_INFO_TYPE, name.c_str(), type);
            dev->getParamInfo(CX_PARAM_INFO_ACCESSS_MODE, name.c_str(), access);
            if ((int)access == CX_PARAM_ACCESS_RO || (int)access == CX_PARAM_ACCESS_RW)
            {
                dev->getParamInfo(CX_PARAM_INFO_DESCRIPTION, name.c_str(), descr);
                dev->getParamInfo(CX_PARAM_INFO_TOOLTIP, name.c_str(), tooltip);
            }
            if ((int)access > CX_PARAM_ACCESS_NOT_AVAILABLE)
            {
                dev->getParamInfo(CX_PARAM_INFO_RANGE, name.c_str(), range);
                dev->getParamInfo(CX_PARAM_INFO_VISIBILITY, name.c_str(), visibility);
                if ((int)type == CX_PARAM_ENUM)
                {
                    dev->getParamInfo(CX_PARAM_INFO_ENUM_SYMBOLS, name.c_str(), sym_entries);
                    dev->getParamInfo(CX_PARAM_INFO_ENUM_INT_VALUE, name.c_str(), int_value);
                }
            }

            std::string visibility_str = "Undefined";
            if ((int)access > CX_PARAM_ACCESS_NOT_AVAILABLE)
            {
                if ((int)visibility == CX_PARAM_VISIBILITY_BEGINNER)
                    visibility_str = "Beginner";
                if ((int)visibility == CX_PARAM_VISIBILITY_EXPERT)
                    visibility_str = "Expert";
                if ((int)visibility == CX_PARAM_VISIBILITY_GURU)
                    visibility_str = "Guru";
            }

            std::string access_str = "";
            if ((int)access == CX_PARAM_ACCESS_RO)
                access_str = "RO";
            else if ((int)access == CX_PARAM_ACCESS_WO)
                access_str = "WO";
            else if ((int)access == CX_PARAM_ACCESS_RW)
                access_str = "RW";
            else if ((int)access == CX_PARAM_ACCESS_NOT_AVAILABLE)
                access_str = "Not Available";
            else if ((int)access == CX_PARAM_ACCESS_NOT_IMPLEMENTED)
                access_str = "Not Implemented";


            std::string type_str = "Unknown";
            if ((int)type == CX_PARAM_INTEGER)
                type_str = "Integer";
            else if ((int)type == CX_PARAM_FLOAT)
                type_str = "Float";
            else if ((int)type == CX_PARAM_STRING)
                type_str = "String";
            else if ((int)type == CX_PARAM_ENUM)
                type_str = "Enum";
            else if ((int)type == CX_PARAM_BOOLEAN)
                type_str = "Boolean";
            else if ((int)type == CX_PARAM_COMMAND)
                type_str = "Command";
            else if ((int)type == CX_PARAM_CATEGORY)
                type_str = "Category";

            outstr << "(Type: " << type_str;
            outstr << ", Access: " << access_str;
            outstr << ", Description: " << (std::string)descr;
            outstr << ", Tooltip: " << (std::string)tooltip;
            if ((int)access > CX_PARAM_ACCESS_NOT_AVAILABLE)        // check if node is not implemented and not available
            {
                outstr << ", Visibility: " << visibility_str;

                if ((int)type == CX_PARAM_ENUM)
                {
                    outstr << ", Entries: ";
                    for (int i = 0; i < sym_entries.data.a.len; i++)
                    {
                        char c;
                        if ((c = ((char*)sym_entries.data.a.buf)[i]) == '\0')
                            outstr << ", ";
                        else
                            outstr << c;
                    }

                    outstr << ", IntValue: " << (int)int_value;

                    outstr << ", Range: ";
                    for (int i = 0; i < range.data.a.len; i++)
                    {
                        outstr << ((int64_t*)range.data.a.buf)[i] << ", ";
                    }
                }
                else if ((int)type == CX_PARAM_STRING)
                {
                    outstr << ", Range: " << (int)range;

                }
                else if ((int)type == CX_PARAM_COMMAND)
                {
                    outstr << ", Range: None";
                }
                else if ((int)type == CX_PARAM_INTEGER)
                {
                    std::vector<int64_t> rangeVec;
                    range.get(rangeVec);
                    outstr << ", Range: " << rangeVec[0] << ".." << rangeVec[1];
                }
            }
            outstr << ")";
        }

        std::string partTypeIDToStr(cx_buffer_part_type typeID)
        {
            std::string strTypeID = "";

            if (typeID == CX_BUFFER_PART_TYPE_ID_UNDEFINED) strTypeID = "CX_BUFFER_PART_TYPE_ID_UNDEFINED";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_IMAGE2D) strTypeID = "CX_BUFFER_PART_TYPE_ID_IMAGE2D";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_TOKEN) strTypeID = "CX_BUFFER_PART_TYPE_ID_TOKEN";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_CHUNK) strTypeID = "CX_BUFFER_PART_TYPE_ID_CHUNK";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_CALIBIR) strTypeID = "CX_BUFFER_PART_TYPE_ID_CALIBIR";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_CALIB3D) strTypeID = "CX_BUFFER_PART_TYPE_ID_CALIB3D";
            else if (typeID == CX_BUFFER_PART_TYPE_ID_CONFIG) strTypeID = "CX_BUFFER_PART_TYPE_ID_CONFIG";
            else strTypeID = "Unknown";

            return strTypeID;
        }

        std::string partPurposeIDToStr(cx_buffer_part_purpose purposeID)
        {
            std::string strPurposeID = "";

            if (purposeID == CX_BUFFER_PART_PURPOSE_ID_UNDEFINED) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_UNDEFINED";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_INTENSITY) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_INTENSITY";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_INFRARED) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_INFRARED";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_RANGE) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_RANGE";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_REFLECTANCE) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_REFLECTANCE";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_CONFIDENCE) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_CONFIDENCE";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_SCATTER) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_SCATTER";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_RANGE_AT_TOKEN) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_RANGE_AT_TOKEN";
            else if (purposeID == CX_BUFFER_PART_PURPOSE_ID_METADATA) strPurposeID = "CX_BUFFER_PART_PURPOSE_ID_METADATA";
            else  strPurposeID = "Unknown";

            return strPurposeID;
        }

        template<typename _Tp>
        void imageMinMax(double& minVal, double& maxVal, const cx::ImagePtr& image)
        {
            _Tp* pData = (_Tp*)image->data();

            minVal = std::numeric_limits<double>::max();
            maxVal = std::numeric_limits<double>::min();

            for (size_t i = 0; i < image->dataSz() / sizeof(_Tp); i++, ++pData)
            {
                if ((double)*pData < minVal) minVal = (double)*pData;
                if ((double)*pData > maxVal) maxVal = (double)*pData;
            }
        }

        void imageMinMax(double& minVal, double& maxVal, const cx::ImagePtr& image)
        {
            switch (image->pixelFormat())
            {
            case CX_PF_MONO_8:
                imageMinMax<uint8_t>(minVal, maxVal, image);
                break;
            case CX_PF_MONO_10:
            case CX_PF_MONO_12:
            case CX_PF_MONO_14:
            case CX_PF_MONO_16:
                imageMinMax<uint16_t>(minVal, maxVal, image);
                break;
            case CX_PF_MONO_32:
                imageMinMax<uint32_t>(minVal, maxVal, image);
                break;
            case CX_PF_MONO_64:
                imageMinMax<uint64_t>(minVal, maxVal, image);
                break;
            case CX_PF_COORD3D_C8:
                imageMinMax<uint8_t>(minVal, maxVal, image);
                break;
            case CX_PF_COORD3D_C16:
                imageMinMax<uint16_t>(minVal, maxVal, image);
                break;
            case CX_PF_COORD3D_C32f:
                imageMinMax<float>(minVal, maxVal, image);
                break;
            }
        }

    }
}