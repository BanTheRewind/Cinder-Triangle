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
#include "cinder/params/Params.h"
#include "cinder/Triangulate.h"
#include "cinder/TriMesh.h"
#include "Triangle.h"

/*
 * This application demonstrates how to triangulate a 
 * Path2d, then move the generated triangles and track 
 * their velocity. Draw with the mouse to add points. 
 * Click on a triangle to select it and drag it.
 */
class AdvancedSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void							draw();
	void							mouseDown( ci::app::MouseEvent event );
	void							mouseDrag( ci::app::MouseEvent event );
	void							mouseMove( ci::app::MouseEvent event );
	void							mouseUp( ci::app::MouseEvent event );
	void							setup();
	void							shutdown();
	void							update();

private:

	// Path created by mouse
	ci::Path2d						mLine;

	struct VelocityTriangle
	{
		VelocityTriangle( uint32_t id, const Trianglef &triangle )
			: mId( id ), mTriangle( triangle ), mVelocity( ci::Vec2f::zero() )
		{
		}
		uint32_t					mId;
		Trianglef					mTriangle;
		ci::Vec2f					mVelocity;
	};

	// Triangles created from path
	void							clear();
	std::vector<VelocityTriangle>	mTriangles;
	void							triangulate();

	ci::Vec2f						mClosestPoint;
	float							mDistancePoint;
	float							mDistanceTriangle;

	// ID of selected triangle
	ci::Vec2f						mMouse;
	bool							mMouseDown;
	int32_t							mSelectedId;

	bool							mFullScreen;
	ci::params::InterfaceGl			mParams;

};

// Import namespaces
using namespace ci;
using namespace ci::app;
using namespace std;

// Reset
void AdvancedSampleApp::clear()
{
	mClosestPoint = Vec2f::zero();
	mDistancePoint = 0.0f;
	mDistanceTriangle = 0.0f;
	mLine = Path2d();
	mMouseDown = false;
	mSelectedId = -1;
	mTriangles.clear();
}

// Render
void AdvancedSampleApp::draw()
{

	// Clear screen
	gl::setMatricesWindow( getWindowSize() );
	gl::clear( Colorf::black() );

	// Iterate through triangles
	for ( vector<VelocityTriangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {

		// Draw triangle and centroid
		Vec2f centroid = triIt->mTriangle.calcCentroid();
		gl::color( 1.0f, 0.25f, 0.5f );
		glLineWidth( 1.0f );
		if ( mSelectedId == triIt->mId ) {
			gl::drawSolidTriangle( triIt->mTriangle );
		} else {
			gl::drawStrokedTriangle( triIt->mTriangle );
		}
		gl::drawSolidCircle( centroid, 1.0f, 12 );

		// Draw velocity
		gl::color( Colorf::white() );
		glBegin( GL_LINE_STRIP );
		{
			gl::vertex( centroid );
			gl::vertex( centroid - triIt->mVelocity );
		}
		glEnd();

		if ( !mMouseDown ) {
			if ( mClosestPoint.length() > 0.0f ) {
				glLineWidth( 0.25f );
				glBegin( GL_LINE_STRIP );
				{
					gl::vertex( mMouse );
					gl::vertex( mClosestPoint );
				}
				glEnd();
			}
		}

		// Draw bounding box of selected triangle
		if ( mSelectedId == triIt->mId ) {
			glLineWidth( 0.25f );
			gl::drawStrokedRect( triIt->mTriangle.calcBoundingBox() );
		}

	}

	// Draw outline
	glLineWidth( 0.5f );
	gl::color( Colorf::white() );
	gl::draw( mLine );

	// Draw params
	mParams.draw();

}

// Handles mouse press
void AdvancedSampleApp::mouseDown( MouseEvent event )
{

	// Update mouse position
	mMouseDown = true;
	mMouse = event.getPos();

	// Hit test triangles
	for ( vector<VelocityTriangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
		if ( triIt->mTriangle.contains( mMouse ) ) {
			mSelectedId = triIt->mId;
			return;
		}
	}

}

// Handles mouse drag
void AdvancedSampleApp::mouseDrag( MouseEvent event )
{

	// Update mouse position
	mMouse = event.getPos();

	// Calculate velocity of selected triangle
	if ( mSelectedId >= 0 ) {
		VelocityTriangle& triangle = mTriangles[ mSelectedId ];
		Vec2f position = triangle.mTriangle.calcCentroid();
		triangle.mTriangle.offsetCenterTo( mMouse );
		triangle.mVelocity = mMouse - position;
		return;
	}

	// Add a point if we've moved more than ten pixels or don't have a point yet
	uint32_t numPoints = mLine.getNumPoints();
	if ( numPoints == 0 ||  event.getPos().distance( mLine.getCurrentPoint() ) > 10 ) {

		// Add point to list
		if ( numPoints == 0 ) {
			mLine.moveTo( event.getPos() );
		}
		mLine.lineTo( event.getPos() );

		// Start triangulating when we have at least three points
		if ( numPoints >= 3 ) {
			triangulate();
		}

	}

}

// Handles mouse move
void AdvancedSampleApp::mouseMove( MouseEvent event )
{

	// Update mouse position
	mMouse = event.getPos();

}

// Handles mouse up
void AdvancedSampleApp::mouseUp( MouseEvent event )
{
	mMouseDown = false;
	mSelectedId = -1;
}

// Set up
void AdvancedSampleApp::setup()
{
	setWindowSize( 800, 600 );
	mFullScreen = false;
	clear();

	gl::enable( GL_LINE_SMOOTH );
	glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );

	mParams = params::InterfaceGl( "PARAMS ", Vec2i( 200, 200 ) );
	mParams.addText( "Drag mouse to draw" );
	mParams.addText( "Select triangle to drag" );
	mParams.addSeparator();
	mParams.addButton( "Clear", bind( &AdvancedSampleApp::clear, this ), "key=space" );
	mParams.addParam( "Toggle fullscreen", &mFullScreen, "key=f" );
	mParams.addButton( "Quit", bind( &AdvancedSampleApp::quit, this ), "key=esc" );
	mParams.addSeparator();
	mParams.addParam( "Nearest point", &mDistancePoint, "", true );
	mParams.addParam( "Nearest triangle", &mDistanceTriangle, "", true );
}

// Called on exit
void AdvancedSampleApp::shutdown()
{

	// Clear lists
	mTriangles.clear();

}

void AdvancedSampleApp::triangulate()
{

	// Triangulate line
	TriMesh2d mesh = Triangulator( mLine ).calcMesh();

	// Rebuild list of triangles from mesh
	mTriangles.clear();
	Vec2f a, b, c;
	for ( uint32_t i = 0; i < mesh.getNumTriangles(); i++ ) {
		mesh.getTriangleVertices( i, &a, &b, &c );
		Trianglef triangle( c, b, a );
		VelocityTriangle velTriangle( i, triangle );
		mTriangles.push_back( velTriangle );
	}

}

void AdvancedSampleApp::update()
{
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}

	if ( mMouseDown ) {
		mClosestPoint = Vec2f::zero();
		mDistancePoint = numeric_limits<float>::max();
		mDistanceTriangle = numeric_limits<float>::max();

		for ( vector<VelocityTriangle>::const_iterator triIt = mTriangles.begin(); triIt != mTriangles.end(); ++triIt ) {
			const Trianglef& triangle = triIt->mTriangle;
			float dist = triangle.distance( mMouse );
			console() << dist << endl;
			if ( dist < mDistanceTriangle ) {
				mDistanceTriangle = dist;
				mClosestPoint = triangle.closestPoint( mMouse );
				mDistancePoint = mClosestPoint.distance( mMouse );
			}
		}
	}
}

// Launch application
CINDER_APP_BASIC( AdvancedSampleApp, RendererGl )
