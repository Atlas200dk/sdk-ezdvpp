/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#include <unistd.h>
#include <memory>
#include <sstream>

#include "dvpp/idvppapi.h"
#include "ascenddk/ascend_ezdvpp/dvpp_data_type.h"
#include "ascenddk/ascend_ezdvpp/dvpp_utils.h"
#include "ascenddk/ascend_ezdvpp/dvpp_vdec.h"

using hiai::ImageData;
using hiai::IMAGEFORMAT;
namespace ascend {
namespace utils {

const static int BIT_DEPTH8 = 8;

DvppVideoDecoder::DvppVideoDecoder(int id,
                                   ResolutionRatio size) {
	channel_id = id;
	is_ready = false;
	resolution = size;
	Init();
}

DvppVideoDecoder::~DvppVideoDecoder() {
	DestroyDvppApi(vdec_api_handle);
}

int DvppVideoDecoder::Init() {
	if (vdec_api_handle == NULL) {
		if (kDvppOperationOk != CreateVdecApi(vdec_api_handle, kVdecSingleton)) {
			ASC_LOG_ERROR("Failed to create dvpp vdec api for channel %d", channel_id);
			return kDvppErrorCreateDvppFail;
		}
	}
    
	if (dvpp_api_handle == NULL) {
		if (kDvppOperationOk != CreateDvppApi(dvpp_api_handle)){ // check create dvpp api result
			ASC_LOG_ERROR("Fail to create vpc api handle");
			return kDvppErrorCreateDvppFail;
		}
	}

	is_ready = true;

	return kDvppOperationOk;
}

int DvppVideoDecoder::Decode(VideoFrameData* video_frame) {
	if (!is_ready && (kDvppOperationOk != Init())) {
		ASC_LOG_ERROR("Dvpp vdec init failed");
		return kDvppErrorVdecInit;
	}

	vdec_msg.channelId = channel_id;
	vdec_msg.call_back = DvppVideoDecoder::FrameDecodeCallBack;	
	vdec_msg.hiai_data_sp = make_shared<HiaiDataSpSon>(
		(void *)this, video_frame->frame_id, video_frame->is_finished);

	if (video_frame->is_finished) {
		vdec_msg.isEOS = 1;
	} else {
		if (video_frame->video_type == kVideoH264)
			strncpy_s(vdec_msg.video_format, sizeof(vdec_msg.video_format), "h264", 4);
		else if (video_frame->video_type == kVideoH265)
			strncpy_s(vdec_msg.video_format, sizeof(vdec_msg.video_format), "h265", 4);
		else
		{
			ASC_LOG_ERROR("Decode video(channel id %d) failed for invalid video type %d", video_frame->video_type);
			return kDvppErrorInvalidParameter;
		}
			
		vdec_msg.in_buffer_size = video_frame->image.size;
		vdec_msg.in_buffer = reinterpret_cast<char*>(video_frame->image.data.get());
		vdec_msg.isEOS = 0;
	}

	dvppapi_ctl_msg dvppApiCtlMsg;
	dvppApiCtlMsg.in = (void*)(&vdec_msg);
	dvppApiCtlMsg.in_size = sizeof(vdec_in_msg);
	// call vdec and check result
	int ret = VdecCtl(vdec_api_handle, DVPP_CTL_VDEC_PROC, &dvppApiCtlMsg, 0);
	if (ret != kDvppOperationOk) {
		ASC_LOG_ERROR("Call dvppctl process return %d, channel id:%d", 
		              ret, channel_id);
		return kDvppErrorVdecCtl;
	}

	return kDvppOperationOk;
}


int DvppVideoDecoder::ConvertHfbcToYuvImage(FRAME* frame, uint8_t* vpc_out_buffer) {
	std::shared_ptr<VpcUserImageConfigure> user_image(new VpcUserImageConfigure);
	user_image->bareDataAddr = nullptr;
	user_image->bareDataBufferSize = 0;
	user_image->widthStride = frame->width;
	user_image->heightStride = frame->height;
	string imageFormat(frame->image_format);
	if (frame->bitdepth == BIT_DEPTH8) {
		if (imageFormat == "nv12") {
			user_image->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
		}
		else {
			user_image->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU;
		}
	}
	else {
		if (imageFormat == "nv12") {
			user_image->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV_10BIT;
		}
		else {
			user_image->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU_10BIT;
		}
	}
	user_image->outputFormat = OUTPUT_YUV420SP_UV;
	user_image->isCompressData = true;

	VpcCompressDataConfigure* compress_data_conf = &user_image->compressDataConfigure;
	uintptr_t baseAddr = (uintptr_t)frame->buffer;
	compress_data_conf->lumaHeadAddr = baseAddr + frame->offset_head_y;
	compress_data_conf->chromaHeadAddr = baseAddr + frame->offset_head_c;
	compress_data_conf->lumaPayloadAddr = baseAddr + frame->offset_payload_y;
	compress_data_conf->chromaPayloadAddr = baseAddr + frame->offset_payload_c;
	compress_data_conf->lumaHeadStride = frame->stride_head;
	compress_data_conf->chromaHeadStride = frame->stride_head;
	compress_data_conf->lumaPayloadStride = frame->stride_payload;
	compress_data_conf->chromaPayloadStride = frame->stride_payload;

	user_image->yuvSumEnable = false;
	user_image->cmdListBufferAddr = nullptr;
	user_image->cmdListBufferSize = 0;

	std::shared_ptr<VpcUserRoiConfigure> roi_conf(new VpcUserRoiConfigure);
	roi_conf->next = nullptr;
	user_image->roiConfigure = roi_conf.get();

	VpcUserRoiInputConfigure* roi_input = &roi_conf->inputConfigure;
	roi_input->cropArea.leftOffset = 0;
	roi_input->cropArea.rightOffset = (frame->width % DVPP_YUV420SP_SIZE_DENOMINATOR) ? frame->width : (frame->width - 1);
	roi_input->cropArea.upOffset = 0;
	roi_input->cropArea.downOffset = (frame->height % DVPP_YUV420SP_SIZE_DENOMINATOR) ? frame->height : (frame->height - 1);

	VpcUserRoiOutputConfigure* roi_output = &roi_conf->outputConfigure;
	roi_output->outputArea.leftOffset = 0;
	roi_output->outputArea.rightOffset = (frame->width % DVPP_YUV420SP_SIZE_DENOMINATOR) ? frame->width : (frame->width - 1);
	roi_output->outputArea.upOffset = 0;
	roi_output->outputArea.downOffset = (frame->height % DVPP_YUV420SP_SIZE_DENOMINATOR) ? frame->height : (frame->height - 1);
	roi_output->widthStride = ALIGN_UP(frame->width, kVpcWidthAlign);
	roi_output->heightStride = ALIGN_UP(frame->height, kVpcHeightAlign);
	roi_output->bufferSize = DVPP_YUV420SP_SIZE(roi_output->widthStride, roi_output->heightStride);
	roi_output->addr = vpc_out_buffer;
	roi_input->cropArea = roi_conf->inputConfigure.cropArea;

	dvppapi_ctl_msg dvpp_api_ctl_msg;
	dvpp_api_ctl_msg.in = static_cast<void*>(user_image.get());
	dvpp_api_ctl_msg.in_size = sizeof(VpcUserImageConfigure);
	int ret = DvppCtl(dvpp_api_handle, DVPP_CTL_VPC_PROC, &dvpp_api_ctl_msg);
	if (ret != kDvppOperationOk) {
		ASC_LOG_ERROR("Call dvppctl fail");
		DestroyDvppApi(dvpp_api_handle);
		return kDvppErrorDvppCtlFail;
	}
	return kDvppOperationOk;

}

void DvppVideoDecoder::FrameDecodeCallBack(FRAME* frame, void* hiai_data) {
	if (frame == nullptr || hiai_data == nullptr) { // check input parameters
		ASC_LOG_ERROR("The input data for function:CallVpcGetYuvImage is nullptr!");
		return;
	}
    
    HiaiDataSpSon* hiai_sp = (HiaiDataSpSon*)hiai_data;  
	DvppVideoDecoder* decoder = (DvppVideoDecoder *)(hiai_sp->decoder);
	int aligned_output_width = ALIGN_UP(frame->width, kVpcWidthAlign);
	int aligned_output_height = ALIGN_UP(frame->height, kVpcHeightAlign);
	int vpc_output_size = DVPP_YUV420SP_SIZE(aligned_output_width, 
	                                         aligned_output_height);

	// check vpc output size is valid
	if (vpc_output_size <= 0 || vpc_output_size > kAllowedMaxImageMemory) {
		ASC_LOG_ERROR("The vpc_output_size:%d is invalid! value range: 1~67108864", vpc_output_size);			
		return;
	}
	// construct vpc out data buffer
	uint8_t* vpc_out_buffer = DvppUtils::DvppDMalloc(vpc_output_size);
	if (vpc_out_buffer == NULL) {
        ASC_LOG_ERROR("Failed to malloc memory in dvpp(decode video)");
        return;    
    }

    if (kDvppOperationOk != decoder->ConvertHfbcToYuvImage(frame, vpc_out_buffer)) {
		DvppUtils::DvppDFree(vpc_out_buffer);
		return;
	}

	shared_ptr<VideoFrameData> video_frame = make_shared<VideoFrameData>();
	video_frame->is_finished = hiai_sp->is_finished;
	video_frame->channel_id = decoder->GetChannelId();
	video_frame->frame_id = hiai_sp->frame_id;	
	video_frame->frame_width = frame->realWidth;
	video_frame->frame_height = frame->realHeight;
	video_frame->image.width = aligned_output_width;
	video_frame->image.height = aligned_output_height;	
	video_frame->image.size = vpc_output_size;
	video_frame->image.data = shared_ptr<uint8_t>(vpc_out_buffer, 
	                                   [](uint8_t *p) { DvppUtils::DvppDFree(p); });
	decoder->AddImageToQueue(video_frame);

	return;
}

int DvppVideoDecoder::AddImageToQueue(shared_ptr<VideoFrameData> frame_image) {
	int ret = kDvppErrorQueueFull;

	for (int count = 0; count < kEnQueueRetryTimes; count++) {
		ret = output_frame_queue.Push(frame_image);
		if (ret == kDvppOperationOk)
			return ret;

		usleep(kEnQueueInterval); // sleep 10 ms		
	}

	ASC_LOG_ERROR("Fail to add video data to frame image queue,"
		          " channel id:%d", channel_id);
	return ret;
}

VideoFrameData* DvppVideoDecoder::Read() {
	for (int i = 0; i < kOutQueueRetryTimes; i++) {
		shared_ptr<VideoFrameData> frame_image = output_frame_queue.Pop();
        if (frame_image != nullptr) {
            return frame_image.get();
        }            
        
        usleep(kOutQueueInterval);        
    }

	return NULL;
}

}
}
