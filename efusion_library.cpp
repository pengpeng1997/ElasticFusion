/*
 *
 *
 * This benchmark file can run any SLAM algorithm compatible with the SLAMBench API
 * We recommend you to generate a library which is compatible with this application.
 * 
 * The interface works that way :
 *   - First benchmark.cpp will call an initialisation function of the SLAM algorithm. (void sb_init())
 *     This function provides the compatible interface with the SLAM algorithm.
 *   - Second benchmark.cpp load an interface (the textual interface in our case)
 *   - Then for every frame, the benchmark.cpp will call sb_process()
 *
 *
 */
 
#include <ElasticFusion.h>

#include <timings.h>
#include "context_helper.h"
#include <SLAMBenchAPI.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/sensor/DepthSensor.h>
#include <io/SLAMFrame.h>




// ===========================================================
// Default Parameters
// ===========================================================

static const float
      default_confidence = 10.0f,
      default_depth = 3.0f,
      default_icp = 10.0f,
      default_icpErrThresh = 5e-05,
      default_covThresh = 1e-05,
      default_photoThresh = 115,
      default_fernThresh = 0.3095f;
static const int
    default_timeDelta = 200,
    default_icpCountThresh = 35000;

static const int
    default_textureDim = 3072,
    default_nodeTextureDim = 16384;

static const bool
     default_openLoop = false,
     default_reloc = false,
     default_fastOdom = false,
     default_so3 = true,
     default_frameToFrameRGB = false;



// ===========================================================
// Algorithm parameters
// ===========================================================

  float confidence,
          depth,
          icp,
          icpErrThresh,
          covThresh,
          photoThresh,
          fernThresh;

    int  icpCountThresh,
         timeDelta,
         textureDim,
         nodeTextureDim;

    bool openLoop,
         reloc,
         fastOdom,
         so3,
         frameToFrameRGB;

	std::string shader_dir;

#define STRINGIFY(x) _STRINGIFY(x)	
#define _STRINGIFY(x) #x
#ifndef efusion_SHADER_DIR
#define efusion_SHADER_DIR ""
#endif
	std::string default_shader_dir = STRINGIFY(efusion_SHADER_DIR);
#undef STRINGIFY
#undef _STRINGIFY
	



// ===========================================================
// ElasticFusion
// ===========================================================


static ElasticFusion * eFusion;

static sb_uchar3* inputRGB;
static sb_uchar4* imageTex;
static unsigned short int * inputDepth;

static sb_uint2 inputSize;


// ===========================================================
// SLAMBench Sensors
// ===========================================================

static slambench::io::DepthSensor *depth_sensor;
static slambench::io::CameraSensor *rgb_sensor;


// ===========================================================
// SLAMBench Outputs
// ===========================================================

slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;

slambench::outputs::Output *rgb_frame_output;
slambench::outputs::Output *render_frame_output;



 bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings)  {

     slam_settings->addParameter(TypedParameter<float>("c", "confidence",     "Confidence",      &confidence, &default_confidence));
     slam_settings->addParameter(TypedParameter<float>("d", "depth",          "Depth",           &depth,      &default_depth));
     slam_settings->addParameter(TypedParameter<float>("", "icp",            "ICP",             &icp, &default_icp));
     slam_settings->addParameter(TypedParameter<float>("ie", "icpErrThresh",   "ICPErrThresh",    &icpErrThresh, &default_icpErrThresh));
     slam_settings->addParameter(TypedParameter<float>("cv", "covThresh",      "CovThresh",       &covThresh, &default_covThresh));
     slam_settings->addParameter(TypedParameter<float>("pt", "photoThresh",    "PhotoThresh",     &photoThresh, &default_photoThresh));
     slam_settings->addParameter(TypedParameter<float>("ft", "fernThresh",     "FernThresh",      &fernThresh, &default_fernThresh));
     slam_settings->addParameter(TypedParameter<int>  ("ic", "icpCountThresh", "ICPCountThresh",  &icpCountThresh, &default_icpCountThresh));
     slam_settings->addParameter(TypedParameter<int>  ("t", "timeDelta",      "TimeDelta",       &timeDelta,      &default_timeDelta));
     slam_settings->addParameter(TypedParameter<int>  ("td", "textureDim",      "textureDim",       &textureDim,      &default_textureDim));
     slam_settings->addParameter(TypedParameter<int>  ("ntd", "nodeTextureDim",      "nodeTextureDim",       &nodeTextureDim,      &default_nodeTextureDim));
     slam_settings->addParameter(TypedParameter<bool>("ol", "openLoop",        "OpenLoop",        &openLoop       , &default_openLoop       ));
     slam_settings->addParameter(TypedParameter<bool>("rl", "reloc",           "Reloc",           &reloc          , &default_reloc          ));
     slam_settings->addParameter(TypedParameter<bool>("fod", "fastOdom",        "FastOdom",        &fastOdom       , &default_fastOdom       ));
     slam_settings->addParameter(TypedParameter<bool>("nso", "so3",             "So3",             &so3            , &default_so3            ));
     slam_settings->addParameter(TypedParameter<bool>("ftf", "frameToFrameRGB", "FrameToFrameRGB", &frameToFrameRGB, &default_frameToFrameRGB));
	 
	 slam_settings->addParameter(TypedParameter<std::string>("sh", "shader-dir", "Directory containing shaders", &shader_dir, &default_shader_dir));
     return true;
 }

 bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings) {


     /**
      * Retrieve RGB and Depth sensors,
      *  - check input_size are the same
      *  - check camera are the same
      *  - get input_file
      */

	setenv("SHADER_DIR", shader_dir.c_str(), 1);

	slambench::io::CameraSensorFinder sensor_finder;
	rgb_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "rgb"}});
	depth_sensor = (slambench::io::DepthSensor*)sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "depth"}});
	 
    if ((rgb_sensor == nullptr) || (depth_sensor == nullptr)) {
        std::cerr << "Invalid sensors found, RGB or Depth not found." << std::endl;
        return false;
    }

	if(rgb_sensor->FrameFormat != slambench::io::frameformat::Raster) {
		std::cerr << "RGB data is in wrong format" << std::endl;
		return false;
	}
	if(depth_sensor->FrameFormat != slambench::io::frameformat::Raster) {
		std::cerr << "Depth data is in wrong format" << std::endl;
		return false;
	}
	if(rgb_sensor->PixelFormat != slambench::io::pixelformat::RGB_III_888) {
		std::cerr << "RGB data is in wrong format pixel" << std::endl;
		return false;
	}
	if(depth_sensor->PixelFormat != slambench::io::pixelformat::D_I_16) {
		std::cerr << "Depth data is in wrong pixel format" << std::endl;
		return false;
	}

	assert(depth_sensor->Width == rgb_sensor->Width);
	assert(depth_sensor->Height == rgb_sensor->Height);
	//assert(depth_sensor->Intrinsics == rgb_sensor->Intrinsics);

     inputSize = make_sb_uint2(rgb_sensor->Width, rgb_sensor->Height);

     Resolution::getInstance(rgb_sensor->Width, rgb_sensor->Height);
    //  std::cout<<"rgb_sensor->Width, rgb_sensor->Height:"<<rgb_sensor->Width<<","<<rgb_sensor->Height<<std::endl;

    float4 camera =  make_float4(
			rgb_sensor->Intrinsics[0],
			rgb_sensor->Intrinsics[1],
			rgb_sensor->Intrinsics[2],
			rgb_sensor->Intrinsics[3]);

     camera.x = camera.x * rgb_sensor->Width;
     camera.y = camera.y * rgb_sensor->Height;
     camera.z = camera.z * rgb_sensor->Width;
     camera.w = camera.w * rgb_sensor->Height;
	 
     // fx, fy, cx, cy
     std::cerr << "Intrisics are fx:" << camera.x << " fy:" << camera.y << " cx:" << camera.z << " cy:" << camera.w << std::endl;
     Intrinsics::getInstance(camera.x, camera.y, camera.z, camera.w);

     std::cerr << "OpenGL setup..." << std::endl;

     setup_opengl_context();

     std::cerr << "OpenGL setup is done." << std::endl;

     eFusion = new ElasticFusion(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
                icpCountThresh,
                icpErrThresh,
                covThresh,
                true,//!openLoop,
                false, // icl-nuim parameter activate a file writing process
                true,//reloc,
                photoThresh,
                confidence,
                depth,
                icp,
                fastOdom,
                fernThresh,
                so3,
                frameToFrameRGB,
                textureDim,
                nodeTextureDim,
				"");


     imageTex = (sb_uchar4*) malloc(
                   sizeof(sb_uchar4) * inputSize.x * inputSize.y);

     inputRGB = (sb_uchar3*) malloc(
                   sizeof(sb_uchar3) * inputSize.x * inputSize.y);
     inputDepth = (uint16_t*) malloc(
                    sizeof(uint16_t) * inputSize.x * inputSize.y);




     pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
     slam_settings->GetOutputManager().RegisterOutput(pose_output);

     pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_COLOUREDPOINTCLOUD, true);
     pointcloud_output->SetKeepOnlyMostRecent(true);
     slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

     rgb_frame_output = new slambench::outputs::Output("RGB Frame", slambench::values::VT_FRAME);
     rgb_frame_output->SetKeepOnlyMostRecent(true);
     slam_settings->GetOutputManager().RegisterOutput(rgb_frame_output);

     render_frame_output = new slambench::outputs::Output("Rendered frame", slambench::values::VT_FRAME);
     render_frame_output->SetKeepOnlyMostRecent(true);
     slam_settings->GetOutputManager().RegisterOutput(render_frame_output);


     return true;
 }


 bool depth_ready = false;
 bool rgb_ready   = false;

bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame* s) {

	if (depth_ready and rgb_ready) {
		depth_ready = false;
		rgb_ready   = false;
	}

	assert(s != nullptr);
	
	char *target = nullptr;
	
	if(s->FrameSensor == depth_sensor) {
		target = (char*)inputDepth;
		depth_ready = true;
	} else if(s->FrameSensor == rgb_sensor) {
		target = (char*)inputRGB;
		rgb_ready = true;
	}
	
	if(target != nullptr) {
		memcpy(target, s->GetData(), s->GetSize());
        // std::cout<<"s->GetSize():"<<s->GetSize()<<std::endl;
		s->FreeData();
	}
	
	return depth_ready and rgb_ready;
}

 bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {
     static int frame = 0;
     eFusion->processFrame((unsigned char*)inputRGB, inputDepth, frame);
     frame++;
     return true;
 }

 
 bool sb_get_tracked  (bool* tracking)  {
    *tracking = fabs(eFusion->getModelToModel().lastSO3Error  - eFusion->getModelToModel().lastSO3Count) >= 0.001;
    return true;
}

 bool sb_clean_slam_system() {
     delete eFusion;
     delete inputRGB;
     delete inputDepth;
     return true;
 }



 bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
		slambench::TimeStamp ts = *ts_p;

		if(pose_output->IsActive()) {
			// Get the current pose as an eigen matrix
			Eigen::Matrix4f mat = eFusion->getCurrPose();

			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
			pose_output->AddPoint(ts, new slambench::values::PoseValue(mat));
		}

		if(pointcloud_output->IsActive()) {
			slambench::values::ColoredPointCloudValue *point_cloud = new slambench::values::ColoredPointCloudValue();

			  Eigen::Vector4f * mapData = eFusion->getGlobalModel().downloadMap();


			     for(unsigned int i = 0; i < eFusion->getGlobalModel().lastCount(); i++)
			         {

			             Eigen::Vector4f pos = mapData[(i * 3) + 0];
			             Eigen::Vector4f col = mapData[(i * 3) + 1];


				    	 slambench::values::ColoredPoint3DF   new_vertex (pos[0] , pos[1] , pos[2]);
				    	 new_vertex.R = int(col[0]) >> 16 & 0xFF;
				    	 new_vertex.G = int(col[0]) >> 8 & 0xFF;
						 new_vertex.B = int(col[0]) & 0xFF;
			         	point_cloud->AddPoint(new_vertex);
			         }




			     // we're finished with the map data we got from efusion, so delete it
			     delete mapData;

			// Take lock only after generating the map
			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());

			pointcloud_output->AddPoint(ts, point_cloud);



		}

		if(rgb_frame_output->IsActive()) {
			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
			rgb_frame_output->AddPoint(ts, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGB_III_888, inputRGB));
		}


		if(render_frame_output->IsActive()) {
		    eFusion->getIndexMap().imageTex()->texture->Download(imageTex, GL_RGBA, GL_UNSIGNED_BYTE);
			std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
			render_frame_output->AddPoint(ts, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGBA_IIII_8888, imageTex));
		}

		return true;

     
     
     return true;
 }

