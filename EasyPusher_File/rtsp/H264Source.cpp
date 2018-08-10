// PHZ
// 2018-5-16

#if defined(WIN32) || defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "H264Source.h"
//#include "RtpConnection.h"
#include <cstdio>
#include <chrono>
#include "xop\Base64.h"
#if defined(__linux) || defined(__linux__)
#include <sys/time.h>
#endif

using namespace xop;
using namespace std;

H264Source::H264Source(uint32_t frameRate)
	: _frameRate(frameRate)
{
    _payload = 96; // rtp负载类型
    _mediaType = H264;
    _clockRate = 90000;
}

H264Source* H264Source::createNew(uint32_t frameRate)
{
    return new H264Source(frameRate);
}

H264Source::~H264Source()
{

}

string H264Source::getMediaDescription(uint16_t port)
{
    char buf[100] = {0};
    sprintf(buf, "m=video %hu RTP/AVP 96", port); // \r\nb=AS:2000

    return string(buf);
}

void H264Source::setSPS(uint8_t *sps, int spsLen) {
	if (sps != nullptr && spsLen != 0) {
		_sps.reset(new uint8_t[spsLen+1]);
		memcpy(_sps.get(), sps, spsLen);
		_sps.get()[spsLen] = 0;
		_spsLen = spsLen;
	}
}

void H264Source::setPPS(uint8_t *pps, int ppsLen) {
	if (pps != nullptr && ppsLen != 0) {
		_pps.reset(new uint8_t[ppsLen+1]);
		memcpy(_pps.get(), pps, ppsLen);
		_pps.get()[ppsLen] = 0;
		_ppsLen = ppsLen;
	}
}

/*
m=video 0 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1;sprop-parameter-sets=,0KAH5ZTAeAi/eAqEAAAPpAADqYJ2hYuaA==
a=control:streamid=0
*/
string H264Source::getAttribute()
{
    // by zhangjianping
    // 增加sps + pps 到 a=
	char buf[500] = { 0 };
	sprintf(buf, "a=rtpmap:96 H264/90000\r\n");
	
	char strTemp[100];
	memset(strTemp, 0, 100);
	av_base64_encode(strTemp, 100, _sps.get(), _spsLen);

	char strTemp2[100];
	memset(strTemp2, 0, 100);
	av_base64_encode(strTemp2, 100, _pps.get(), _ppsLen);
	sprintf(buf + strlen(buf), "a=fmtp:96 packetization-mode=1;sprop-parameter-sets=%s,%s", strTemp, strTemp2);

	return string(buf);
}

bool H264Source::handleFrame(MediaChannelId channelId, AVFrame& frame)
{
    char *frameBuf  = frame.buffer.get();
    uint32_t frameSize = frame.size;

    if(frame.timestamp == 0)
        frame.timestamp = getTimeStamp();

    if (frameSize <= MAX_RTP_PAYLOAD_SIZE)
    {
		//RtpPacketPtr rtpPkt((char*)xop::Alloc(1500), xop::Free);
        RtpPacketPtr rtpPkt(new char[1500]);
        memcpy(rtpPkt.get()+4+RTP_HEADER_SIZE, frameBuf, frameSize); // 预留12字节 rtp header

        if(_sendFrameCallback)
            _sendFrameCallback(channelId, frame.type, rtpPkt, frameSize+4+RTP_HEADER_SIZE, 1, frame.timestamp);
    }
    else
    {
        char FU_A[2] = {0};

        // 分包参考live555
        FU_A[0] = (frameBuf[0] & 0xE0) | 28;
        FU_A[1] = 0x80 | (frameBuf[0] & 0x1f);

        frameBuf  += 1;
        frameSize -= 1;

        while (frameSize + 2 > MAX_RTP_PAYLOAD_SIZE)
        {
			//RtpPacketPtr rtpPkt((char*)xop::Alloc(1500), xop::Free);
            RtpPacketPtr rtpPkt(new char[1500]);
            rtpPkt.get()[RTP_HEADER_SIZE+4] = FU_A[0];
            rtpPkt.get()[RTP_HEADER_SIZE+5] = FU_A[1];
            memcpy(rtpPkt.get()+4+RTP_HEADER_SIZE+2, frameBuf, MAX_RTP_PAYLOAD_SIZE-2);

            if(_sendFrameCallback)
                _sendFrameCallback(channelId, frame.type, rtpPkt, 4+RTP_HEADER_SIZE+MAX_RTP_PAYLOAD_SIZE, 0, frame.timestamp);

            frameBuf  += MAX_RTP_PAYLOAD_SIZE - 2;
            frameSize -= MAX_RTP_PAYLOAD_SIZE - 2;

            FU_A[1] &= ~0x80;
        }

        {
			//RtpPacketPtr rtpPkt((char*)xop::Alloc(1500), xop::Free);
            RtpPacketPtr rtpPkt(new char[1500]);
            FU_A[1] |= 0x40;
            rtpPkt.get()[RTP_HEADER_SIZE+4] = FU_A[0];
            rtpPkt.get()[RTP_HEADER_SIZE+5] = FU_A[1];
            memcpy(rtpPkt.get()+4+RTP_HEADER_SIZE+2, frameBuf, frameSize);

            if(_sendFrameCallback)
                _sendFrameCallback(channelId, frame.type, rtpPkt, 4+RTP_HEADER_SIZE+2+frameSize, 1, frame.timestamp);
        }
    }

    return true;
}

uint32_t H264Source::getTimeStamp()
{
/* #if defined(__linux) || defined(__linux__)
	struct timeval tv = {0};
	gettimeofday(&tv, NULL);
	uint32_t ts = ((tv.tv_sec*1000)+((tv.tv_usec+500)/1000))*90; // 90: _clockRate/1000;
	return ts;
#else  */
	//auto timePoint = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now());
	//auto timePoint = chrono::time_point_cast<chrono::milliseconds>(chrono::steady_clock::now());

    //auto timePoint = chrono::time_point_cast<chrono::milliseconds>(chrono::high_resolution_clock::now());
    //return (uint32_t)(timePoint.time_since_epoch().count()*90);

		auto timePoint = chrono::time_point_cast<chrono::microseconds>(chrono::high_resolution_clock::now());
		return (uint32_t)((timePoint.time_since_epoch().count() + 500) / 1000 * 90 );
//#endif
}
