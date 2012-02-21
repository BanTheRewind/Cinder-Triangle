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
 * vector of points, then move the generated triangles
 * and track their velocity. Draw with the mouse to add 
 * points. Click on a triangle to select it and drag it.
 */
class AdvancedSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void mouseDown( ci::app::MouseEvent event );
	void mouseDrag( ci::app::MouseEvent event );
	void mouseUp( ci::app::MouseEvent event );
	void setup();
	void shutdown();

private:

	// Path created by mouse
	std::vector<ci::Vec2f>		mPoints;

	// Triangles created from points
	std::vector<Triangle>		mTriangles;

	// ID of selected triangle
	ci::Vec2f					mMouse;
	int32_t						mSelectedId;

};

// Import namespaces
using namespace ci;
using namespace ci::app;
using namespace std;

// Render
void AdvancedSampleApp::draw()
{

	// Clear screen
	gl::setMatricesWindow( getWindowSize() );
	gl::clear( Colorf::black() );

	// Set line width
	glLineWidth( 2.0f );
	
	// Iterate through triangles
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {

		// Draw triangle
		gl::color( 1.0f, 0.25f, 0.5f );
		glBegin( mSelectedId == triIt->getId() ? GL_TRIANGLE_STRIP : GL_LINE_STRIP );
		gl::vertex( triIt->a() );
		gl::vertex( triIt->b() );
		gl::vertex( triIt->c() );
		gl::vertex( triIt->a() );
		glEnd();

		// Draw centroid
		gl::drawSolidCircle( triIt->getCentroid(), 1.0f, 12 );

		// Draw velocity
		gl::color( Colorf::white() );
		glBegin( GL_LINE_STRIP );
		gl::vertex( triIt->getCentroid() );
		gl::vertex( triIt->getCentroid() - triIt->getVelocity() );
		glEnd();

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
	gl::drawString( "Drag mouse to draw", Vec2f( 20.0f, getWindowHeight() - 84.0f ) );
	gl::drawString( "Select triangle to drag", Vec2f( 20.0f, getWindowHeight() - 68.0f ) );
	gl::drawString( "SPACE = Clear screen", Vec2f( 20.0f, getWindowHeight() - 52.0f ) );
	gl::drawString( "F = Toggle full screen", Vec2f( 20.0f, getWindowHeight() - 36.0f ) );
	gl::drawString( "ESC = quit", Vec2f( 20.0f, getWindowHeight() - 20.0f ) );

}

// Handles key press
void AdvancedSampleApp::keyDown( KeyEvent event )
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

// Handles mouse press
void AdvancedSampleApp::mouseDown( MouseEvent event )
{

	// Update mouse position
	mMouse = event.getPos();

	// Hit test triangles
	for ( vector<Triangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
		if ( triIt->contains( mMouse ) ) {
			mSelectedId = triIt->getId();
			return;
		}
	}

}

// Handles mouse drag
void AdvancedSampleApp::mouseDrag( MouseEvent event )
{

	// Update mouse position
	mMouse = event.getPos();

	// Do not draw while dragging
	if ( mSelectedId >= 0 ) {
		mTriangles[ mSelectedId ].setPosition( mMouse );
		return;
	}

	// Add a point if we've moved more than ten pixels or don't have a point yet
	if ( mPoints.size() <= 0 ||  event.getPos().distance( * mPoints.rbegin() ) > 10 ) {

		// Add point to list
		mPoints.push_back( mMouse );
	
		// Start triangulating when we have at least three points
		if ( mPoints.size() >= 3 ) {
			mTriangles = Triangle::triangulate( mPoints );
		}

	}

}

// Handles mouse up
void AdvancedSampleApp::mouseUp( MouseEvent event )
{

	// Clear selected ID
	mSelectedId = -1;

}

// Set up
void AdvancedSampleApp::setup()
{

	// Initialize selected ID
	mSelectedId = -1;

}

// Called on exit
void AdvancedSampleApp::shutdown()
{

	// Clear lists
	mPoints.clear();
	mTriangles.clear();

}

// Launch application
CINDER_APP_BASIC( AdvancedSampleApp, RendererGl )
