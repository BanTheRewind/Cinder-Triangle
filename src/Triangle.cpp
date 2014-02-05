/*
* 
* Copyright (c) 2014, Ban the Rewind
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

#include "Triangle.h"

#include "cinder/Matrix22.h"
#include "cinder/MatrixAffine2.h"

using namespace ci;
using namespace std;

///////////////////////////////////////////////////////////////////////////////

template<typename T>
TriangleT<T>::TriangleT( const Vec2<T> &origin, const Vec2<T> &destination, const Vec2<T> &apex ) 
	: mApex( apex ), mDestination( destination ), mOrigin( origin )
{
}

template<typename T>
TriangleT<T> TriangleT<T>::one()
{
	T degToRad	= (T)( (float)M_PI / 180.0f );
	T apex		= (T)0.0f;
	T dest		= (T)60.0f * degToRad;
	T origin	= (T)120.0f * degToRad;
	Vec2<T> a	= calcPolarToCartesian( Vec2<T>::zero(), Vec2<T>( (T)1.0f, origin ) );
	Vec2<T> b	= calcPolarToCartesian( Vec2<T>::zero(), Vec2<T>( (T)1.0f, dest ) );
	Vec2<T> c	= calcPolarToCartesian( Vec2<T>::zero(), Vec2<T>( (T)1.0f, apex ) );
	return TriangleT<T>( a, b, c );
}

template<typename T>
TriangleT<T> TriangleT<T>::zero()
{
	return TriangleT<T>();
}

///////////////////////////////////////////////////////////////////////////////

// Math operators
template<typename T>
TriangleT<T> TriangleT<T>::operator+( const ci::Vec2<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle += rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator+( const TriangleT<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle += rhs;
	return triangle;
}

template<typename T>
void TriangleT<T>::operator+=( const ci::Vec2<T> &rhs )
{
	mApex			+= rhs;
	mDestination	+= rhs;
	mOrigin			+= rhs;
}

template<typename T>
void TriangleT<T>::operator+=( const TriangleT<T> &rhs )
{
	mApex			+= rhs.mApex;
	mDestination	+= rhs.mDestination;
	mOrigin			+= rhs.mOrigin;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator-( const ci::Vec2<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle -= rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator-( const TriangleT<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle -= rhs;
	return triangle;
}

template<typename T>
void TriangleT<T>::operator-=( const ci::Vec2<T> &rhs )
{
	mApex			-= rhs;
	mDestination	-= rhs;
	mOrigin			-= rhs;
}

template<typename T>
void TriangleT<T>::operator-=( const TriangleT<T> &rhs )
{
	mApex			-= rhs.mApex;
	mDestination	-= rhs.mDestination;
	mOrigin			-= rhs.mOrigin;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator*( const T &rhs )
{
	TriangleT<T> triangle( *this );
	triangle *= rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator*( const ci::Vec2<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle *= rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator*( const TriangleT<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle *= rhs;
	return triangle;
}

template<typename T>
void TriangleT<T>::operator*=( const T &rhs )
{
	mApex			*= rhs;
	mDestination	*= rhs;
	mOrigin			*= rhs;
}

template<typename T>
void TriangleT<T>::operator*=( const ci::Vec2<T> &rhs )
{
	mApex			*= rhs;
	mDestination	*= rhs;
	mOrigin			*= rhs;
}

template<typename T> 
void TriangleT<T>::operator*=( const TriangleT<T> &rhs )
{
	mApex			*= rhs.mApex;
	mDestination	*= rhs.mDestination;
	mOrigin			*= rhs.mOrigin;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator/( const T &rhs )
{
	TriangleT<T> triangle( *this );
	triangle /= rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator/( const ci::Vec2<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle /= rhs;
	return triangle;
}

template<typename T>
TriangleT<T> TriangleT<T>::operator/( const TriangleT<T> &rhs )
{
	TriangleT<T> triangle( *this );
	triangle /= rhs;
	return triangle;
}

template<typename T>
void TriangleT<T>::operator/=( const T &rhs )
{
	mApex			/= rhs;
	mDestination	/= rhs;
	mOrigin			/= rhs;
}

template<typename T>
void TriangleT<T>::operator/=( const ci::Vec2<T> &rhs )
{
	mApex			/= rhs;
	mDestination	/= rhs;
	mOrigin			/= rhs;
}

template<typename T>
void TriangleT<T>::operator/=( const TriangleT<T> &rhs )
{
	mApex /= rhs.mApex;
	mDestination /= rhs.mDestination;
	mOrigin /= rhs.mOrigin;
}

template<typename T>
bool TriangleT<T>::operator==( const TriangleT<T> &rhs )
{
	return	mApex.x		== rhs.mApex.x			&& 
		mApex.y			== rhs.mApex.y			&& 
		mDestination.x	== rhs.mDestination.x	&& 
		mDestination.y	== rhs.mDestination.y	&& 
		mOrigin.x		== rhs.mOrigin.x		&& 
		mOrigin.y		== rhs.mOrigin.y;
}

template<typename T>
bool TriangleT<T>::operator!=( const TriangleT<T> &rhs )
{
	return !( *this == rhs );
}

template<typename T>
bool TriangleT<T>::operator<( const TriangleT<T> &rhs )
{
	return *this != rhs;
}

///////////////////////////////////////////////////////////////////////////////

template<typename T>
T TriangleT<T>::calcAngle( const Vec2<T> &point ) const
{
    return calcAngle( *this, point );
}

template<typename T>
T TriangleT<T>::calcAngle( const TriangleT<T> &triangle, const Vec2<T> &point )
{
    return calcAngle( calcCentroid( triangle ), point );
}

template<typename T>
T TriangleT<T>::calcAngle( const Vec2<T> &a, const Vec2<T> &b )
{
    return math<T>::atan2( b.y - a.y, b.x - a.x );
}

template<typename T>
T TriangleT<T>::calcArea() const
{
	return calcArea( *this );
}

template<typename T>
T TriangleT<T>::calcArea( const TriangleT<T> &triangle )
{
	Vec2<T> a = triangle.mOrigin;
	Vec2<T> b = triangle.mDestination;
	Vec2<T> c = triangle.mApex;
	return calcArea( a, b, c );
}

template<typename T>
T TriangleT<T>::calcArea( const Vec2<T> &a, const Vec2<T> &b, const Vec2<T> &c )
{
	T x1 = a.x - b.x;
	T y1 = a.y - b.y;
	T x2 = b.x - c.x;
	T y2 = b.y - c.y;

	return ( x1 * y2 - y1 * x2 ) * 0.5f;
}

template<typename T>
Vec2<T> TriangleT<T>::calcBarycentricToCartesian( const Vec3<T> &point ) const
{
	return calcBarycentricToCartesian( *this, point );
}

template<typename T>
Vec2<T> TriangleT<T>::calcBarycentricToCartesian( const TriangleT<T> &triangle, const Vec3<T> &point )
{
	return Vec2<T>( point.x * triangle.mOrigin + point.y * triangle.mDestination + point.z * triangle.mApex );
}

template<typename T>
RectT<T> TriangleT<T>::calcBoundingBox() const
{
	return calcBoundingBox( *this );
}

template<typename T> 
RectT<T> TriangleT<T>::calcBoundingBox( const TriangleT<T> &triangle ) 
{
	Vec2<T> a	= triangle.mOrigin;
	Vec2<T> b	= triangle.mDestination;
	Vec2<T> c	= triangle.mApex;
	T x1		= math<T>::min( a.x, b.x );
	x1			= math<T>::min(  x1, c.x );
	T y1		= math<T>::min( a.y, b.y );
	y1			= math<T>::min(  y1, c.y );
	T x2		= math<T>::max( a.x, b.x );
	x2			= math<T>::max(  x2, c.x );
	T y2		= math<T>::max( a.y, b.y );
	y2			= math<T>::max(  y2, c.y );
	return RectT<T>( x1, y1, x2, y2 );
}

template<typename T>
Vec3<T> TriangleT<T>::calcCartesianToBarycentric( const Vec2<T> &point ) const
{
    return calcCartesianToBarycentric( *this, point );
}

template<typename T>
Vec3<T> TriangleT<T>::calcCartesianToBarycentric( const TriangleT<T> &triangle, const Vec2<T> &point )
{
	Vec2<T> a = triangle.mOrigin;
	Vec2<T> b = triangle.mDestination;
	Vec2<T> c = triangle.mApex;
	
	Matrix22<T> m22( a.x - c.x, a.y - c.y, b.x - c.x, b.y - c.y );
	T invDet	= (T)1.0f / m22.determinant();
	T l1		= invDet * ( ( b.y - c.y ) * ( point.x - c.x ) +
							 ( c.x - b.x ) * ( point.y - c.y ) );
	T l2		= invDet * ( ( c.y - a.y ) * ( point.x - c.x ) +
							 ( a.x - c.x ) * ( point.y - c.y ) );
	T l3		= 1 - l1 - l2;
	return Vec3<T>( l1, l2, l3 );
}

template<typename T>
Vec2<T> TriangleT<T>::calcCartesianToPolar( const Vec2<T> &point ) const
{
	return calcCartesianToPolar( calcCentroid( *this ), point );
}

template<typename T>
Vec2<T> TriangleT<T>::calcCartesianToPolar( const Vec2<T> &a, const Vec2<T> &b )
{
	Vec2<T> result;
	result.x = a.distance( b );
	result.y = calcAngle( a, b );
	return result;
}

template<typename T>
Vec2<T> TriangleT<T>::calcCentroid() const
{
	return calcCentroid( *this );
}

template<typename T> 
Vec2<T> TriangleT<T>::calcCentroid( const TriangleT<T> &triangle ) 
{
	Vec2<T> a = triangle.mOrigin;
	Vec2<T> b = triangle.mDestination;
	Vec2<T> c = triangle.mApex;
	return ( a + b + c ) / 3.0f;
}

template<typename T>
T TriangleT<T>::calcHeight() const
{
	return calcBoundingBox( *this ).getHeight();
}

template<typename T>
Vec2<T> TriangleT<T>::calcPolarToCartesian( const Vec2<T> &point ) const
{
	return calcPolarToCartesian( calcCentroid( *this ), point );
}

template<typename T> 
Vec2<T> TriangleT<T>::calcPolarToCartesian( const ci::Vec2<T> &a, const Vec2<T> &b )
{
	Vec2<T> result( math<T>::cos( b.y ), math<T>::sin( b.y ) );
	return a + result * b.x;
}

template<typename T>
const Vec2<T> TriangleT<T>::calcSize() const
{
	return calcBoundingBox( *this ).getSize();
}

template<typename T>
T TriangleT<T>::calcWidth() const
{
	return calcBoundingBox( *this ).getWidth();
}

///////////////////////////////////////////////////////////////////////////////

template<typename T>
ci::Vec2<T> TriangleT<T>::closestPoint( const ci::Vec2<T> &point ) const
{
	return closestPoint( *this, point );
}

template<typename T>
Vec2<T> TriangleT<T>::closestPoint( const TriangleT<T> &triangle, const ci::Vec2<T> &p, int32_t n )
{
	Vec2<T> a	= triangle.mOrigin;
	Vec2<T> b	= triangle.mDestination;
	Vec2<T> c	= triangle.mApex;
	T distA		= p.distanceSquared( a );
	T distB		= p.distanceSquared( b );
	T distC		= p.distanceSquared( c );
	
	vector<PointSort> points;
	points.push_back( PointSort( a, distA ) );
	points.push_back( PointSort( b, distB ) );
	points.push_back( PointSort( c, distC ) );
	sort( points.begin(), points.end() );
	
	uint32_t index = math<int32_t>::clamp( n, 0, 2 );
	return points[ index ].mPoint;	
}

template<typename T> 
bool TriangleT<T>::contains( const Vec2<T> &point ) const
{
	return contains( *this, point );
}

template<typename T>
bool TriangleT<T>::contains( const TriangleT<T> &triangle, const ci::Vec2<T> &p )
{
	Vec2<T> a = triangle.mOrigin;
	Vec2<T> b = triangle.mDestination;
	Vec2<T> c = triangle.mApex;
	
	T area  = calcArea( a, b, c );
	T area0 = calcArea( b, c, p );
	T area1 = calcArea( c, a, p );
	T area2 = calcArea( a, b, p );
	
	T areaSum = math<T>::abs( area0 ) + math<T>::abs( area1 ) + math<T>::abs( area2 );
	
	// If sum of the three areas is the same as the triangle's
	// area, the point is inside
	return areaSum == area;
}

template<typename T> 
T TriangleT<T>::distance( const Vec2<T> &point ) const
{
	return distance( *this, point );
}

template<typename T>
T TriangleT<T>::distance( const TriangleT<T> &triangle, const ci::Vec2<T> &p )
{
	ci::Vec2<T> i;
	if ( intersection( triangle, p, &i ) ) {
		return p.distance( i );
	}
	return numeric_limits<T>::max();
}

template<typename T> 
T TriangleT<T>::distanceSquared( const Vec2<T> &point ) const
{
	return distanceSquared( *this, point );
}

template<typename T>
T TriangleT<T>::distanceSquared( const TriangleT<T> &triangle, const ci::Vec2<T> &p )
{
	T dist = distance( triangle, p );
	return math<T>::pow( dist, 2.0f );
}

template<typename T> 
bool TriangleT<T>::intersection( const ci::Vec2<T> &point, Vec2<T>* i ) const
{
	return intersection( *this, point, i );
}

template<typename T>
bool TriangleT<T>::intersection( const TriangleT<T> &triangle, const Vec2<T> &p, Vec2<T>* intersection )
{
	Vec2<T> a1	= p;
	Vec2<T> b1	= triangle.calcCentroid();
	Vec2<T> a2	= closestPoint( triangle, p );
	Vec2<T> b2	= closestPoint( triangle, p, 1 );
	T y1		= b1.y - a1.y;
	T x1		= b1.x - a1.x;
	T y2		= b2.y - a2.y;
	T x2		= b2.x - a2.x;
	T dist		= y2 * x1 - y1 * x2;
	if ( dist == (T)0.0f ) {
		return false;
	}
	T theta = ( x2 * ( a1.y - a2.y ) - y2 * ( a1.x - a2.x ) ) / dist;
	if ( theta < (T)0.0f || theta > (T)1.0f ) {
		return false;
	}
	intersection->x = a1.x + theta * x1;
	intersection->y = a1.y + theta * y1;
	return true;
}

template<typename T> 
bool TriangleT<T>::intersects( const TriangleT<T> &triangle ) const
{
	return intersects( *this, triangle );
}

template<typename T>
bool TriangleT<T>::intersects( const TriangleT<T> &a, const TriangleT<T> &b )
{
	if ( a.contains( b.mApex ) ) {
		return true;
	}
	if ( a.contains( b.mDestination ) ) {
		return true;
	}
	if ( a.contains( b.mOrigin ) ) {
		return true;
	}
	if ( b.contains( a.mApex ) ) {
		return true;
	}
	if ( b.contains( a.mDestination ) ) {
		return true;
	}
	if ( b.contains( a.mOrigin ) ) {
		return true;
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////

template<typename T> 
Vec2<T>& TriangleT<T>::a() 
{ 
	return mOrigin; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::a() const 
{ 
	return mOrigin; 
}

template<typename T> 
Vec2<T>& TriangleT<T>::b() 
{ 
	return mDestination; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::b() const 
{ 
	return mDestination; 
}

template<typename T> 
Vec2<T>& TriangleT<T>::c() 
{ 
	return mApex; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::c() const 
{ 
	return mApex; 
}

template<typename T> 
Vec2<T>& TriangleT<T>::getApex() 
{ 
	return mApex; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::getApex() const 
{ 
	return mApex; 
}

template<typename T> 
Vec2<T>& TriangleT<T>::getDestination() 
{ 
	return mDestination; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::getDestination() const 
{ 
	return mDestination; 
}

template<typename T> 
Vec2<T>& TriangleT<T>::getOrigin() 
{
	return mOrigin; 
}

template<typename T> 
const Vec2<T>& TriangleT<T>::getOrigin() const 
{ 
	return mOrigin; 
}

///////////////////////////////////////////////////////////////////////////////

template<typename T> 
void TriangleT<T>::a( const Vec2<T> &origin )
{
	setOrigin( origin );
}

template<typename T> 
void TriangleT<T>::b( const Vec2<T> &destination )
{
	setDestination( destination );
}

template<typename T> 
void TriangleT<T>::c( const Vec2<T> &apex )
{
	setApex( apex );
}

template<typename T> 
void TriangleT<T>::set( const ci::Vec2<T> &origin, const ci::Vec2<T> &destination, const ci::Vec2<T> &apex )
{
	mApex			= apex;
	mDestination	= destination;
	mOrigin			= origin;
}

template<typename T> 
void TriangleT<T>::setApex( const Vec2<T> &apex )
{
	mApex = apex;
}

template<typename T> 
void TriangleT<T>::setDestination( const Vec2<T> &destination )
{
	mDestination = destination;
}

template<typename T> 
void TriangleT<T>::setOrigin( const Vec2<T> &origin )
{
	mOrigin = origin;
}

///////////////////////////////////////////////////////////////////////////////

template<typename T>
TriangleT<T> TriangleT<T>::getCentered() const
{
	return getCentered( *this );
}

template<typename T>
TriangleT<T> TriangleT<T>::getCentered( const TriangleT<T> &triangle )
{
	Vec2<T> centroid = triangle.calcCentroid();
	TriangleT<T> result( triangle.mOrigin		- centroid,
						  triangle.mDestination - centroid,
						  triangle.mApex		- centroid );
	return result;
}

template<typename T>
TriangleT<T> TriangleT<T>::getCenteredFit( const TriangleT<T> &triangle ) const
{
	return getCenteredFit( *this, triangle );
}

template<typename T>
TriangleT<T> TriangleT<T>::getCenteredFit( const TriangleT<T> &a, const TriangleT<T> &b )
{
	return a; // TODO fit triangle A to triangle B
	// Find scale difference between triangles, center then scale and return
}

template<typename T>
void TriangleT<T>::include( const Vec2<T> &point )
{
	// TODO
}

template<typename T>
void TriangleT<T>::include( const std::vector<Vec2<T> > &points )
{
	// TODO
}

template<typename T>
void TriangleT<T>::include( const TriangleT<T> &triangle )
{
	// TODO
}

template<typename T> 
void TriangleT<T>::inflate( const Vec2<T> &scale )
{
	Vec2<T> centroid	= calcCentroid( *this );
	mApex				-= centroid;
	mDestination		-= centroid;
	mOrigin				-= centroid; 
	*this				*= scale;
	mApex				+= centroid;
	mDestination		+= centroid;
	mOrigin				+= centroid; 
}

template<typename T>
TriangleT<T> TriangleT<T>::inflated( const Vec2<T> &scale ) const
{
	return inflated( *this, scale );
}

template<typename T>
TriangleT<T> TriangleT<T>::inflated( const TriangleT<T> &triangle, const Vec2<T> &scale )
{
	TriangleT<T> result( triangle );
	result.inflate( scale );
	return result;
}

template<typename T> 
void TriangleT<T>::offset( const Vec2<T> & offset )
{
	mApex += offset;
	mDestination += offset;
	mOrigin += offset;
}

template<typename T> 
void TriangleT<T>::offsetCenterTo( const ci::Vec2<T> &center )
{
	Vec2<T> centroid	= calcCentroid( *this );
	mApex				= mApex - centroid + center;
	mDestination		= mDestination - centroid + center;
	mOrigin				= mOrigin - centroid + center;
}

template<typename T> 
void TriangleT<T>::rotate( T radians )
{
	Vec2<T> centroid	= calcCentroid( *this );
	T angleA			= calcAngle( centroid, mOrigin )		+ radians;
	T angleB			= calcAngle( centroid, mDestination )	+ radians;
	T angleC			= calcAngle( centroid, mApex )			+ radians;
	T distA				= centroid.distance( mOrigin );
	T distB				= centroid.distance( mDestination );
	T distC				= centroid.distance( mApex );
	mOrigin				= calcPolarToCartesian( mOrigin,		Vec2<T>( distA, angleA ) );
	mDestination		= calcPolarToCartesian( mDestination,	Vec2<T>( distB, angleB ) );
	mApex				= calcPolarToCartesian( mApex,			Vec2<T>( distC, angleC ) );
}

template<typename T>
TriangleT<T> TriangleT<T>::rotated( T radians ) const
{
	return rotated( *this, radians );
}

template<typename T>
TriangleT<T> TriangleT<T>::rotated( const TriangleT<T> &triangle, T radians )
{
	TriangleT<T> result( triangle );
	result.rotate( radians );
	return result;
}

template<typename T> 
void TriangleT<T>::scale( T scale )
{
	Vec2<T> tl		= calcBoundingBox( *this ).getUpperLeft();
	mApex			-= tl;
	mDestination	-= tl;
	mOrigin			-= tl; 
	*this			*= scale;
	mApex			+= tl;
	mDestination	+= tl;
	mOrigin			+= tl; 
}

template<typename T> 
void TriangleT<T>::scaleCentered( T scale )
{
	Vec2<T> centroid	= calcCentroid( *this );
	mApex				-= centroid;
	mDestination		-= centroid;
	mOrigin				-= centroid; 
	*this				*= scale;
	mApex				+= centroid;
	mDestination		+= centroid;
	mOrigin				+= centroid; 
}

template<typename T>
TriangleT<T> TriangleT<T>::scaledCentered( T scale ) const
{
	return scaledCentered( *this, scale );
}

template<typename T>
TriangleT<T> TriangleT<T>::scaledCentered( const TriangleT<T> &triangle, T scale )
{
	TriangleT<T> result( triangle );
	result.scaleCentered( scale );
	return result;
}

template<typename T>
TriangleT<T> TriangleT<T>::scaled( T scale ) const
{
	return scaled( *this, scale );
}

template<typename T>
TriangleT<T> TriangleT<T>::scaled( const TriangleT<T> &triangle, T scale )
{
	TriangleT<T> result( triangle );
	result.scale( scale );
	return result;
}

template<typename T>
TriangleT<T> TriangleT<T>::transformCopy( const MatrixAffine2<T> &matrix ) const
{
	return transformCopy( *this, matrix );
}

template<typename T>
TriangleT<T> TriangleT<T>::transformCopy( const TriangleT<T> &triangle, const MatrixAffine2<T> &matrix )
{
	TriangleT<T> result( triangle );
	result.include( matrix.transformPoint( triangle.mOrigin ) );
	result.include( matrix.transformPoint( triangle.mDestination ) );
	result.include( matrix.transformPoint( triangle.mApex ) );
	return result;
}

///////////////////////////////////////////////////////////////////////////////

template<typename T> 
TriangleT<T>::PointSort::PointSort( const ci::Vec2<T> &point, T distance )
{
	mDistance = distance;
	mPoint = point;
}

template<typename T> 
bool TriangleT<T>::PointSort::operator<( const PointSort &rhs ) const
{
	return mDistance < rhs.mDistance;
}

template<typename T> 
bool TriangleT<T>::PointSort::operator==( const PointSort &rhs ) const
{
	return mDistance == rhs.mDistance && 
		mPoint.x == rhs.mPoint.x && 
		mPoint.y == rhs.mPoint.y;
}

template<typename T> 
bool TriangleT<T>::PointSort::operator!=( const PointSort &rhs ) const
{
	return !( *this == rhs );
}

///////////////////////////////////////////////////////////////////////////////

#include "cinder/gl/gl.h"

namespace cinder { namespace gl {

void drawSolidTriangle( const Triangled &triangle, bool textureTriangle )
{
	drawSolidTriangle<double>( triangle );
}

void drawStrokedTriangle( const Triangled &triangle )
{
	drawStrokedTriangle<double>( triangle );
}
	
void drawSolidTriangle( const Trianglef &triangle, bool textureTriangle )
{
	drawSolidTriangle<float>( triangle, textureTriangle );
}
	
void drawStrokedTriangle( const Trianglef &triangle )
{
	drawStrokedTriangle<float>( triangle );
}
	
template<typename T> 
void drawSolidTriangle( const TriangleT<T> &triangle, bool textureTriangle )
{
	// TO DO: calculate and set texture coords
	begin( GL_TRIANGLE_STRIP );
	{
		vertex( (float)triangle.c().x, (float)triangle.c().y );
		vertex( (float)triangle.b().x, (float)triangle.b().y );
		vertex( (float)triangle.a().x, (float)triangle.a().y );
	}
	end();
}

template<typename T>
void drawStrokedTriangle( const TriangleT<T> &triangle )
{
	begin( GL_LINE_STRIP );
	{
		vertex( (float)triangle.c().x, (float)triangle.c().y );
		vertex( (float)triangle.b().x, (float)triangle.b().y );
		vertex( (float)triangle.a().x, (float)triangle.a().y );
		vertex( (float)triangle.c().x, (float)triangle.c().y );
	}
	end();
}

} }

template<typename T>
std::ostream& operator<< ( std::ostream &out, const TriangleT<T> &triangle )
{
	out << "(Orig: " << triangle.mOrigin.x << ", " << triangle.mOrigin.y << ")-";
	out << "(Dest: " << triangle.mDestination.x << ", " << triangle.mDestination.y << ")-";
	out << "(Apex: " << triangle.mApex.x << ", " << triangle.mApex.y << ")";
	return out;
}

template class TriangleT<float>;
template class TriangleT<double>;
 