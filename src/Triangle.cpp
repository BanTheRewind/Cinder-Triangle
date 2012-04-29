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
using namespace ci::app;
using namespace std;

// Calculate area of a triangle
float Triangle::calcArea( const Vec2f &a, const Vec2f &b, const Vec2f &c )
{

	// Get length of each side
	float xod( a.x - b.x );
	float yod( a.y - b.y );
	float xda( b.x - c.x );
	float yda( b.y - c.y );

	// Area is half times base times height
	return ( xod * yda - yod * xda ) * 0.5f;

}

// Calculate centroid from points
Vec2f Triangle::calcCentroid( const Vec2f &a, const Vec2f &b, const Vec2f &c ) {

	// The centroid is the average of the points
	return ( a + b + c ) / 3.0f;

}

// Constructor
Triangle::Triangle( const Vec2f &origin, const Vec2f &destination, const Vec2f &apex, 
					int32_t id ) 
	: mApex( apex ), mDestination( destination ), mId( id ), mOrigin( origin )
{
	mCentroid = calcCentroid( mOrigin, mDestination, mApex );
	mPrevCentroid = mCentroid;
	calcArea();
}

// Hit test triangle
bool Triangle::contains( const Vec2f &position )
{
	
	// Get area values using input position and two points from triangle
	float area0 = calcArea( mDestination, mApex, position );
	float area1 = calcArea( mApex, mOrigin, position );
	float area2 = calcArea( mOrigin, mDestination, position );

	// Sum the absolute values of each area
	float areaSum = math<float>::abs( area0 ) + math<float>::abs( area1 ) + math<float>::abs( area2 );

	// If sum of the three areas is the same as the triangle's 
	// area, the point is inside
	return areaSum == mArea;

}

// Hit test triangle
bool Triangle::contains( const Vec2f &position ) const
{

	// Get area values using input position and two points from triangle
	float area0 = calcArea( mDestination, mApex, position );
	float area1 = calcArea( mApex, mOrigin, position );
	float area2 = calcArea( mOrigin, mDestination, position );

	// Sum the absolute values of each area
	float areaSum = math<float>::abs( area0 ) + math<float>::abs( area1 ) + math<float>::abs( area2 );

	// If sum of the three areas is the same as the triangle's 
	// area, the point is inside
	return areaSum == mArea;

}

// Point getter shortcuts
Vec2f& Triangle::a() 
{ 
	return mOrigin; 
}
const Vec2f& Triangle::a() const 
{ 
	return mOrigin; 
}
Vec2f& Triangle::b() 
{ 
	return mDestination; 
}
const Vec2f& Triangle::b() const 
{ 
	return mDestination; 
}
Vec2f& Triangle::c() 
{ 
	return mApex; 
}
const Vec2f& Triangle::c() const 
{ 
	return mApex; 
}

// Update area
float Triangle::calcArea()
{
	mArea = calcArea( mOrigin, mDestination, mApex );
	return mArea;
}

// Getters
Vec2f& Triangle::getApex() 
{ 
	return mApex; 
}
const Vec2f& Triangle::getApex() const 
{ 
	return mApex; 
}
float Triangle::getArea() 
{ 
	return mArea; 
}
float Triangle::getArea() const 
{ 
	return mArea; 
}
Vec2f& Triangle::getCentroid() 
{ 
	return mCentroid; 
}
const Vec2f& Triangle::getCentroid() const 
{ 
	return mCentroid; 
}
Vec2f& Triangle::getDestination() 
{ 
	return mDestination; 
}
const Vec2f& Triangle::getDestination() const 
{ 
	return mDestination; 
}
int32_t Triangle::getId() 
{ 
	return mId; 
}
int32_t Triangle::getId() const 
{ 
	return mId; 
}
Vec2f& Triangle::getOrigin() 
{ 
	return mOrigin; 
}
const Vec2f& Triangle::getOrigin() const 
{ 
	return mOrigin; 
}
Vec2f Triangle::getVelocity() 
{ 
	return mCentroid - mPrevCentroid; 
}
const Vec2f Triangle::getVelocity() const 
{ 
	return mCentroid - mPrevCentroid; 
}

// Move triangle
void Triangle::move( const Vec2f & offset )
{

	// Move all points
	mApex += offset;
	mCentroid += offset;
	mDestination += offset;

	// Move centroid to update velocity
	setCentroid( mCentroid + offset );

}

// Point setter shortcuts
void Triangle::a( const Vec2f &origin )
{
	mOrigin = origin;
}
void Triangle::b( const Vec2f &destination )
{
	mDestination = destination;
}
void Triangle::c( const Vec2f &apex )
{
	mApex = apex;
}

// Setters
void Triangle::setApex( const Vec2f &apex )
{
	mApex = apex;
}
void Triangle::setArea( float area )
{
	mArea = area;
}
void Triangle::setCentroid( const Vec2f &centroid ) 
{ 
	mPrevCentroid = mCentroid;
	mCentroid = centroid;
}
void Triangle::setDestination( const Vec2f &destination )
{
	mDestination = destination;
}
void Triangle::setId( int32_t id )
{
	mId = id;
}
void Triangle::setOrigin( const Vec2f &origin )
{
	mOrigin = origin;
}
void Triangle::setPosition( const ci::Vec2f &position )
{
	
	// Move each point
	mApex = mApex - mCentroid + position;
	mDestination = mDestination - mCentroid + position;
	mOrigin = mOrigin - mCentroid + position;

	// Update centroid
	setCentroid( position );

}

namespace cinder { namespace gl {

void drawSolidTriangle( const Triangle &triangle, bool textureTriangle )
{
	glBegin( GL_TRIANGLE_STRIP );
	{
		gl::vertex( triangle.c() );
		gl::vertex( triangle.b() );
		gl::vertex( triangle.a() );
	}
	glEnd();
}

void drawStrokedTriangle( const Triangle &triangle )
{
	glBegin( GL_LINE_STRIP );
	{
		gl::vertex( triangle.c() );
		gl::vertex( triangle.b() );
		gl::vertex( triangle.a() );
		gl::vertex( triangle.c() );
	}
	glEnd();
}

} }
