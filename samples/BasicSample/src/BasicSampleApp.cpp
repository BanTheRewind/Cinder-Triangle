/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "Triangle.h"

/*
 * This application demonstrates how to triangulate a 
 * vector of points and hit test the generated triangles.
 * Draw with mouse to add points. Click on a triangle
 * to select it.
 */
class BasicSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void mouseDrag( ci::app::MouseEvent event );
	void mouseUp( ci::app::MouseEvent event );
	void shutdown();

private:

	// Path created by mouse
	std::vector<ci::Vec2f>		mPoints;

	// Triangles created from points
	std::vector<Triangle>		mTriangles;

	// ID of selected triangle
	int32_t						mSelectedId;

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

	// Draw triangles
	glLineWidth( 2.0f );
	gl::color( 1.0f, 0.25f, 0.5f );
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
		glBegin( mSelectedId == triIt->getId() ? GL_TRIANGLE_STRIP : GL_LINE_STRIP );
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
		mSelectedId = -1;
		mPoints.clear();
		mTriangles.clear();
		break;
	}

}

// Handles mouse drag
void BasicSampleApp::mouseDrag( MouseEvent event )
{

	// Clear selected triangle ID
	mSelectedId = -1;

	// Add a point if we've moved more than ten pixels or don't have a point yet
	if ( mPoints.size() <= 0 ||  event.getPos().distance( * mPoints.rbegin() ) > 10 ) {

		// Add point to list
		mPoints.push_back( event.getPos() );
	
		// Start triangulating when we have at least three points
		if ( mPoints.size() >= 3 ) {
			mTriangles = Triangle::triangulate( mPoints );
		}

	}

}

// Handles mouse up
void BasicSampleApp::mouseUp( MouseEvent event )
{

	// Get position
	Vec2f position = event.getPos();
	
	// Reset selected ID
	mSelectedId = -1;

	// Hit test triangles
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
		if ( triIt->contains( position ) ) {
			mSelectedId = triIt->getId();
			return;
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
