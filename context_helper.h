/*
 * context_helper.h
 *
 *  This code is partially imported from Pangolin 
 */

#ifndef BENCHMARKS_EFUSION_INCLUDE_CONTEXT_HELPER_H_
#define BENCHMARKS_EFUSION_INCLUDE_CONTEXT_HELPER_H_

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>


typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

static bool ctxErrorOccurred = false;
static int ctxErrorHandler( Display *, XErrorEvent * )
{
	ctxErrorOccurred = true;
	return 0;
}

// Helper to check for extension string presence.  Adapted from:
//   http://www.opengl.org/resources/features/OGLextensions/
static bool isExtensionSupported(const char *extList, const char *extension)
{
	const char *start;
	const char *where, *terminator;

	/* Extension names should not have spaces. */
	where = strchr(extension, ' ');
	if (where || *extension == '\0')
		return false;

	for (start=extList;;) {
		where = strstr(start, extension);

		if (!where)
			break;

		terminator = where + strlen(extension);

		if ( where == start || *(where - 1) == ' ' )
			if ( *terminator == ' ' || *terminator == '\0' )
				return true;

		start = terminator;
	}

	return false;
}

inline void setup_opengl_context() {

	std::cerr << "Startup OpenGL context..." << std::endl;
	Display *display = 0;
	std::cerr << "XOpenDisplay..." << std::endl;

	display = XOpenDisplay(NULL);

	if (!display) {
		throw std::runtime_error("X11: Failed to open X display");
	}

	std::cerr << "glXChooseFBConfig..." << std::endl;
	// Desired attributes
	static int visual_attribs[] =
	{
			GLX_X_RENDERABLE    , True,
			GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
			GLX_RENDER_TYPE     , GLX_RGBA_BIT,
			GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
			GLX_RED_SIZE        , 8,
			GLX_GREEN_SIZE      , 8,
			GLX_BLUE_SIZE       , 8,
			GLX_ALPHA_SIZE      , 8,
			GLX_DEPTH_SIZE      , 24,
			GLX_STENCIL_SIZE    , 8,
			GLX_DOUBLEBUFFER    , True,
			None
	};


	int glx_major, glx_minor;
	if ( !glXQueryVersion( display, &glx_major, &glx_minor ) ||
			( ( glx_major == 1 ) && ( glx_minor < 3 ) ) || ( glx_major < 1 ) )
	{
		// FBConfigs were added in GLX version 1.3.
		throw std::runtime_error("Invalid GLX version. Require GLX >= 1.3");
	}

	int fbcount;
	GLXFBConfig* fbc = glXChooseFBConfig(display, DefaultScreen(display), visual_attribs, &fbcount);
	if (!fbc) {
		throw std::runtime_error("Unable to retrieve framebuffer options");
	}

	int best_fbc = -1;
	int worst_fbc = -1;
	int best_num_samp = -1;
	int worst_num_samp = 999;
	std::cerr << "Frame Buffer options are " << fbcount << std::endl;

	// Enumerate framebuffer options, storing the best and worst that match our attribs
	for (int i=0; i<fbcount; ++i)
	{
		XVisualInfo *vi = glXGetVisualFromFBConfig( display, fbc[i] );
		if ( vi )
		{
			int samp_buf, samples;
			glXGetFBConfigAttrib( display, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
			glXGetFBConfigAttrib( display, fbc[i], GLX_SAMPLES       , &samples  );

			std::cerr << " -  samp_buf:" << samp_buf << " samples:" << samples << std::endl;
			if ( (best_fbc < 0) || (samp_buf>0 && samples>best_num_samp) )
				best_fbc = i, best_num_samp = samples;

			if ( (worst_fbc < 0) || (samp_buf>0 && samples<worst_num_samp) )
				worst_fbc = i, worst_num_samp = samples;
		}
		XFree( vi );
	}
	std::cerr << " -  best_fbc:" << best_fbc << " worst_fbc:" << worst_fbc << std::endl;
	std::cerr << " -  best_num_samp:" << best_num_samp << " worst_num_samp:" << worst_num_samp << std::endl;
	std::cerr << "glXGetVisualFromFBConfig XCreateColormap  XCreateWindow ... "  << std::endl;

	// Select the minimum suitable option. The 'best' is often too slow.
	GLXFBConfig bestFbc = fbc[ worst_fbc ];
	XFree( fbc );

	// Get a visual
	XVisualInfo *vi = glXGetVisualFromFBConfig( display, bestFbc );

	// Create colourmap
	XSetWindowAttributes swa;
	swa.colormap = XCreateColormap( display,
			RootWindow( display, vi->screen ),
			vi->visual, AllocNone );
	swa.background_pixmap = None ;
	swa.border_pixel      = 0;
	swa.event_mask        = StructureNotifyMask;

	// Create window
	Window win = XCreateWindow( display, RootWindow( display, vi->screen ),
			0, 0, 1024, 768, 0, vi->depth, InputOutput,
			vi->visual,
			CWBorderPixel|CWColormap|CWEventMask, &swa );

	XFree( vi );




	std::cerr << "glXQueryExtensionsString glXGetProcAddressARB ... "  << std::endl;
	// Get the default screen's GLX extension list
	const char *glxExts = glXQueryExtensionsString( display, DefaultScreen( display ) );
	std::cerr << "glxExts= " << glxExts  << std::endl;
	std::cout << "glXCreateContextAttribsARB " << (void*) glXGetProcAddress((const GLubyte*)"glXCreateContextAttribsARB") << std::endl;

	glXCreateContextAttribsARBProc glXCreateContextAttribsARB =
			(glXCreateContextAttribsARBProc) glXGetProcAddressARB(
					(const GLubyte *) "glXCreateContextAttribsARB"
			);


	// Install an X error handler so the application won't exit if GL 3.0
	// context allocation fails. Handler is global and shared across all threads.
	ctxErrorOccurred = false;
	GLXContext ctx = 0;
	int (*oldHandler)(Display*, XErrorEvent*) = XSetErrorHandler(&ctxErrorHandler);

	if ( isExtensionSupported( glxExts, "GLX_ARB_create_context" ) && glXCreateContextAttribsARB )
	{
		std::cerr << "GLX_ARB_create_context is available ... " << std::endl;
		int context_attribs[] = {
				GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
				GLX_CONTEXT_MINOR_VERSION_ARB, 0,
				//GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
				//GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
				None
		};

		ctx = glXCreateContextAttribsARB( display, bestFbc, 0, true, context_attribs );

		// Sync to ensure any errors generated are processed.
		XSync( display, False );
		if ( ctxErrorOccurred || !ctx ) {
			std::cerr << "glXCreateContextAttribsARB Fall back to old-style 2.x context... " << std::endl;
			ctxErrorOccurred = false;
			// Fall back to old-style 2.x context. Implementations will return the newest
			// context version compatible with OpenGL versions less than version 3.0.
			context_attribs[1] = 1;  // GLX_CONTEXT_MAJOR_VERSION_ARB = 1
			context_attribs[3] = 0;  // GLX_CONTEXT_MINOR_VERSION_ARB = 0
			ctx = glXCreateContextAttribsARB( display, bestFbc, 0, True, context_attribs );
		}
	} else {
		std::cerr << "glXCreateNewContext  Fallback to GLX 1.3 Context... " << std::endl;
		// Fallback to GLX 1.3 Context
		ctx = glXCreateNewContext( display, bestFbc, GLX_RGBA_TYPE, 0, True );
	}

	// Sync to ensure any errors generated are processed.
	XSync( display, False );

	// Restore the original error handler
	XSetErrorHandler( oldHandler );

	if ( ctxErrorOccurred || !ctx )
	{
		throw std::runtime_error("Failed to create an OpenGL context");
	}

	// Verifying that context is a direct context
	if ( ! glXIsDirect ( display, ctx ) ) {
		std::cerr << "Indirect GLX rendering context obtained\n";
	}

	glXMakeCurrent( display, win, ctx );


	glewInit();
}


#endif /* BENCHMARKS_EFUSION_INCLUDE_CONTEXT_HELPER_H_ */
