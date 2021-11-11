/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Enric Cervera.
 *
 * libcamera_ros_publisher.cpp - libcamera ROS publisher node.
 */

#include <chrono>

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"
#include "core/options.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

using Stream = libcamera::Stream;

using namespace std::placeholders;

// The main event loop for the application.

static void event_loop(LibcameraApp &app)
{
	Options const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();

	auto start_time = std::chrono::high_resolution_clock::now();

	Stream *stream = app.GetMainStream();
	if (!stream || stream->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("Error: only YUV420 format supported");
		
	unsigned int w, h, stride;
	Mat src;
	app.StreamDimensions(stream, &w, &h, &stride);

	for (unsigned int count = 0; ; count++)
	{
		LibcameraApp::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Quit)
			return;
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		if (options->verbose)
			std::cerr << "Viewfinder frame " << count << std::endl;
		auto now = std::chrono::high_resolution_clock::now();
		if (options->timeout && now - start_time > std::chrono::milliseconds(options->timeout)) {
			std::cout << "w = " << w << std::endl;
			std::cout << "h = " << h << std::endl;
			std::cout << "stride = " << stride << std::endl;			
			imwrite("/home/pi/opencv.jpg", src);
			return;
		}
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		
		libcamera::Span<uint8_t> buffer = app.Mmap(completed_request->buffers[stream])[0];
		uint8_t *ptr = (uint8_t *)buffer.data();
		src = Mat(h, w, CV_8U, ptr, stride);

		app.ShowPreview(completed_request, app.ViewfinderStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
	}
	return 0;
}
