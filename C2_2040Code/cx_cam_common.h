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
#pragma once
#ifndef CX_CAM_COMMON_H_INCLUDED
#define CX_CAM_COMMON_H_INCLUDED

#include "cx_cam.h"
#include "cx_cam_param.h"
#include "AT/cx/base.h"
#include "AT/cx/Device.h"
#include "AT/cx/DeviceFactory.h"
#include "AT/cx/DeviceBuffer.h"

namespace AT {
    namespace cx {


        DeviceInfoPtr discoverAndChooseDevice(bool useFilterDriver = false);

        void printParamInfo(cx::DevicePtr dev, const std::string& param, std::ostream& os);

        std::string partTypeIDToStr(cx_buffer_part_type typeID);

        std::string partPurposeIDToStr(cx_buffer_part_purpose purposeID);

        void imageMinMax(double& minVal, double& maxVal, const cx::ImagePtr& image);

    }   // namespace cx
}   // namespace AT
#endif // CX_CAM_COMMON_H_INCLUDED
