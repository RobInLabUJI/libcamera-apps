/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_encoder.cpp - libcamera video encoding class.
 */

#include "core/libcamera_app.hpp"
#include "core/video_options.hpp"
#include "encoder/encoder.hpp"

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;

class LibcameraEncoder : public LibcameraApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	LibcameraEncoder() : LibcameraApp(std::make_unique<VideoOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&LibcameraEncoder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream)
	{
		assert(encoder_);
		unsigned int w, h, stride;
		StreamDimensions(stream, &w, &h, &stride);
		FrameBuffer *buffer = completed_request->buffers[stream];
		libcamera::Span span = Mmap(buffer)[0];
		void *mem = span.data();
		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");
		int64_t timestamp_ns = buffer->metadata().timestamp;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer(buffer->planes()[0].fd.fd(), span.size(), mem, w, h, stride, timestamp_ns / 1000);
	}
	VideoOptions *GetOptions() const { return static_cast<VideoOptions *>(options_.get()); }
	void StopEncoder() { encoder_.reset(); }

protected:
	virtual void createEncoder() { encoder_ = std::unique_ptr<Encoder>(Encoder::Create(GetOptions())); }
	std::unique_ptr<Encoder> encoder_;

private:
	void encodeBufferDone(void *mem)
	{
		// If non-NULL, mem would indicate which buffer has been completed, but
		// currently we're just assuming everything is done in order. (We could
		// handle this by replacing the queue with a vector of <mem, completed_request>
		// pairs.)
		assert(mem == nullptr);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			encode_buffer_queue_.pop(); // drop shared_ptr reference
		}
	}

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
};
