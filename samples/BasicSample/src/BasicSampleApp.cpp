#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "Triangle.h"

// We'll create a new Cinder Application by deriving from the BasicApp class
class BasicSampleApp : public ci::app::AppBasic 
{

public:

	void keyDown( ci::app::KeyEvent event );
	void mouseDrag( ci::app::MouseEvent event );

	void draw();

	void shutdown();

private:

	// This will maintain a list of points which we will draw line segments between
	std::vector<ci::Vec2f>		mPoints;
	std::vector<Triangle>		mTriangles;

};

// Import namespaces
using namespace ci;
using namespace ci::app;
using namespace std;

// Render
void BasicSampleApp::draw()
{

	// Clear screen
	gl::setMatricesWindow( getWindowSize() );
	gl::clear( Colorf::black() );

	// We'll set the color to an orange color
	glLineWidth( 2.0f );
	gl::color( 1.0f, 0.5f, 0.25f );
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
		glBegin( GL_LINE_STRIP );
		gl::vertex( triIt->a() );
		gl::vertex( triIt->b() );
		gl::vertex( triIt->c() );
		glEnd();
		gl::drawSolidCircle( triIt->getCentroid(), 1.0f, 12 );
	}

	// Draw outline
	glLineWidth( 0.5f );
	gl::color( Colorf::white() );
	glBegin( GL_LINE_STRIP );
	for ( vector<Vec2f>::const_iterator pointIt = mPoints.begin(); pointIt != mPoints.end(); ++pointIt ) {
		gl::vertex( * pointIt );
	}
	glEnd();

	// Write instructions
	gl::drawString( "Drag mouse to draw", Vec2f( 20.0f, getWindowHeight() - 68.0f ) );
	gl::drawString( "SPACE = Clear screen", Vec2f( 20.0f, getWindowHeight() - 52.0f ) );
	gl::drawString( "F = Toggle full screen", Vec2f( 20.0f, getWindowHeight() - 36.0f ) );
	gl::drawString( "ESC = quit", Vec2f( 20.0f, getWindowHeight() - 20.0f ) );

}

// Handles key press
void BasicSampleApp::keyDown( KeyEvent event )
{

	// Use keyboard to quit, toggle fullscreen, or clear data
	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_SPACE:
		mPoints.clear();
		mTriangles.clear();
		break;
	}

}

// Handles mouse drag
void BasicSampleApp::mouseDrag( MouseEvent event )
{

	// Add a point if we've more than ten pixels or don't have a point yet
	if ( mPoints.size() <= 0 ||  event.getPos().distance( * mPoints.rbegin() ) > 10 ) {

		// Add point to list
		mPoints.push_back( event.getPos() );
	
		// Start triangulating when we have at least three points
		if ( mPoints.size() >= 3 ) {
			mTriangles = Triangle::triangulate( mPoints );
		}

	}

}

// Called on exit
void BasicSampleApp::shutdown()
{

	// Clear lists
	mPoints.clear();
	mTriangles.clear();

}

// Launch application
CINDER_APP_BASIC( BasicSampleApp, RendererGl )
