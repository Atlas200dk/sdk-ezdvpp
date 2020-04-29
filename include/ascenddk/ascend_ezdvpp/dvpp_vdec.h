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
#ifndef ASCENDDK_ASCEND_EZDVPP_VDEC_H_
#define ASCENDDK_ASCEND_EZDVPP_VDEC_H_
#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include "hiaiengine/data_type.h"
#include "dvpp/Vdec.h"
#include "dvpp_data_type.h"
#include "circle_queue.h"


using namespace std;

#define VIDEO_CHANNEL_MAX  16
#define INVALID_CHANNEL_ID -1

#define VDEC_QUEUE_SIZE 16

namespace ascend {
namespace utils {

class HiaiDataSpSon : public HIAI_DATA_SP {
public:
	HiaiDataSpSon(void* decoder_, uint32_t frame_id_, bool is_finished_) {
		decoder = decoder_;
		frame_id = frame_id_;
		is_finished = is_finished_;
	}

	~HiaiDataSpSon() {
	}
	
	bool is_finished;
	uint32_t frame_id;
    void* decoder;

};

class DvppVdecQueue {
public:
    DvppVdecQueue(){}
    ~DvppVdecQueue(){}
    
    int Push(shared_ptr<VideoFrameData> frame_image) {
		if (frame_image_queue.IsFull())
			return kDvppErrorQueueFull;
		frame_image_queue.Push(frame_image);
		return kDvppOperationOk;
	}

    shared_ptr<VideoFrameData> Pop(){
		if (frame_image_queue.IsEmpty())
			return nullptr;
		return frame_image_queue.Pop();
	}

private:
    CircleQueue<shared_ptr<VideoFrameData>, VDEC_QUEUE_SIZE> frame_image_queue;	            
};

class DvppVideoDecoder {
public:
	DvppVideoDecoder(int id, ResolutionRatio size);
	~DvppVideoDecoder();

	int Decode(VideoFrameData* video_frame);
	static void FrameDecodeCallBack(FRAME* frame, void* hiai_data);
	int AddImageToQueue(shared_ptr<VideoFrameData> frame_image);	
    VideoFrameData* Read();
	int GetChannelId() { return channel_id; };

private:
    int Init();
	int ConvertHfbcToYuvImage(FRAME* frame, uint8_t* vpc_out_buffer);
	
private:
	bool is_ready;
	int channel_id;
	ResolutionRatio resolution;
	IDVPPAPI* vdec_api_handle;
	IDVPPAPI* dvpp_api_handle;
	vdec_in_msg vdec_msg;
	DvppVdecQueue output_frame_queue;
};
}
}
#endif /* OBJECT_DETECTION_OBJECT_DETECTION_H_ */
