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

// Include header
#include "Triangle.h"

// Imports
using namespace ci;
using namespace tpp;
using namespace std;

// Convert point list into Delaunay triangles
vector<Triangle> Triangle::triangulate( const vector<Vec2f> & aPoints, uint32_t aResolution )
{

	// Initialize output list
	vector<Triangle> triangles;

	// Set resolution to default or specified value
	float resolution = math<float>::max( 3.0f, (float)( aResolution > 0 ? aResolution : aPoints.size() / 2 ) );

	// Convert Cinder points to Delaunay points
    float size = (float)aPoints.size();
    float count = math<float>::min( resolution, size );
    Delaunay::Point position;
    vector<Delaunay::Point> positions;
    for ( float i = 0.0f; i < count; i++ ) {
		Vec2f point = aPoints[ (uint32_t)( i / count * size ) ];
		position[ 0 ] = point.x;
        position[ 1 ] = point.y;
        positions.push_back( position );
    }

	// Triangulate points
	Delaunay delaunay( positions );
	delaunay.Triangulate();

	// Iterate through triangles
    for ( Delaunay::fIterator triIt = delaunay.fbegin(); triIt != delaunay.fend(); ++triIt ) {

		// Get point indexes
        float a = (float)delaunay.Org( triIt );
        float b = (float)delaunay.Dest( triIt );
        float c = (float)delaunay.Apex( triIt );
        uint32_t aId = (int32_t)( ( a / resolution ) * size );
        uint32_t bId = (int32_t)( ( b / resolution ) * size );
        uint32_t cId = (int32_t)( ( c / resolution ) * size );

		// Set positions in triangles
        Vec2f triangle[ 3 ];
        triangle[ 0 ] = aPoints[ aId ];
        triangle[ 1 ] = aPoints[ bId ];
        triangle[ 2 ] = aPoints[ cId ];

		// Find center of triangle
		Vec2f centroid = Vec2f(
			( triangle[0].x + triangle[1].x + triangle[2].x ) / 3.0f, 
			( triangle[0].y + triangle[1].y + triangle[2].y ) / 3.0f
		);

		// Initialize properties to test triangle position
		int32_t counter = 0;
		Vec2f point = aPoints[ 0 ];

		// Iterate through points
		int32_t id = 0;
		for ( vector<Vec2f>::const_iterator pointIt = aPoints.begin(); pointIt != aPoints.end(); ++pointIt ) {

			// Compare centroid of this triangle to the previous one
			if ( centroid.y >  math<float>::min( point.y, pointIt->y ) && 
				 centroid.y <= math<float>::max( point.y, pointIt->y ) && 
				 centroid.x <= math<float>::max( point.x, pointIt->x ) && 
				 point.y != pointIt->y && 
				 ( point.x == pointIt->x || centroid.x <= ( centroid.y - point.y ) * ( pointIt->x - point.x ) / ( pointIt->y - point.y ) + point.x ) ) {
				counter++;
			}

			// Assign this point to last
			point = * pointIt;

		}

		// Only include triangles which are inside shape
        if ( counter % 2 != 0 ) {

			// Add triangle to list
			Triangle triData( triangle[ 0 ], triangle[ 1 ], triangle[ 2 ], id, (float)delaunay.area( triIt ), centroid );
			triangles.push_back( triData );

			// Increase default ID
			id++;

        }

    }

	// Return triangles
	return triangles;

}

//! Constructor
Triangle::Triangle( const ci::Vec2f & origin, const ci::Vec2f & destination, const ci::Vec2f & apex, 
					int32_t id, float area, const ci::Vec2f & centroid ) 
	: mApex( apex ), mArea( 0.0f ), mCentroid( centroid ), mDestination( destination ), mId( id ), 
	  mOrigin( origin ), mPrevCentroid( centroid )
{
}

// Point getter shortcuts
const ci::Vec2f Triangle::a() 
{ 
	return mOrigin; 
}
const ci::Vec2f Triangle::a() const 
{ 
	return mOrigin; 
}
const ci::Vec2f Triangle::b() 
{ 
	return mDestination; 
}
const ci::Vec2f Triangle::b() const 
{ 
	return mDestination; 
}
const ci::Vec2f Triangle::c() 
{ 
	return mApex; 
}
const ci::Vec2f Triangle::c() const 
{ 
	return mApex; 
}

// Getters
const ci::Vec2f Triangle::getApex() 
{ 
	return mApex; 
}
const ci::Vec2f Triangle::getApex() const 
{ 
	return mApex; 
}
const float	Triangle::getArea() 
{ 
	return mArea; 
}
const float Triangle::getArea() const 
{ 
	return mArea; 
}
const ci::Vec2f Triangle::getCentroid() 
{ 
	return mCentroid; 
}
const ci::Vec2f Triangle::getCentroid() const 
{ 
	return mCentroid; 
}
const ci::Vec2f Triangle::getDestination() 
{ 
	return mDestination; 
}
const ci::Vec2f Triangle::getDestination() const 
{ 
	return mDestination; 
}
const int32_t Triangle::getId() 
{ 
	return mId; 
}
const int32_t Triangle::getId() const 
{ 
	return mId; 
}
const ci::Vec2f Triangle::getOrigin() 
{ 
	return mOrigin; 
}
const ci::Vec2f Triangle::getOrigin() const 
{ 
	return mOrigin; 
}
ci::Vec2f Triangle::getVelocity() 
{ 
	return mCentroid - mPrevCentroid; 
}
ci::Vec2f Triangle::getVelocity() const 
{ 
	return mCentroid - mPrevCentroid; 
}

// Point setter shortcuts
void Triangle::a( const ci::Vec2f & origin )
{
	mOrigin = origin;
}
void Triangle::b( const ci::Vec2f & destination )
{
	mDestination = destination;
}
void Triangle::c( const ci::Vec2f & apex )
{
	mApex = apex;
}

// Setters
void Triangle::setApex( const ci::Vec2f & apex )
{
	mApex = apex;
}
void Triangle::setArea( float area )
{
	mArea = area;
}
void Triangle::setCentroid( const ci::Vec2f & centroid ) 
{ 
	mPrevCentroid = mCentroid;
	mCentroid = centroid;
}
void Triangle::setDestination( const ci::Vec2f & destination )
{
	mDestination = destination;
}
void Triangle::setId( int32_t id )
{
	mId = id;
}
void Triangle::setOrigin( const ci::Vec2f & origin )
{
	mOrigin = origin;
}
