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

#pragma once

#include "cinder/Rect.h"

namespace cinder {
template<typename T> class MatrixAffine2;
}

template<typename T>
class TriangleT
{

public:

	// Constructors
	TriangleT( const ci::Vec2<T> &origin = ci::Vec2<T>::zero(), const ci::Vec2<T> &destination = ci::Vec2<T>::zero(), 
			  const ci::Vec2<T> &apex = ci::Vec2<T>::zero() );
	static TriangleT<T>	one();
	static TriangleT<T>	zero();

	///////////////////////////////////////////////////////////////////////////////

	// Math operators
	TriangleT<T>		operator+( const ci::Vec2<T> &rhs );
	TriangleT<T>		operator+( const TriangleT<T> &rhs );
	void				operator+=( const ci::Vec2<T> &rhs );
	void				operator+=( const TriangleT<T> &rhs );
	
	TriangleT<T>		operator-( const ci::Vec2<T> &rhs );
	TriangleT<T>		operator-( const TriangleT<T> &rhs );
	void				operator-=( const ci::Vec2<T> &rhs );
	void				operator-=( const TriangleT<T> &rhs );
	
	TriangleT<T>		operator*( const T &rhs );
	TriangleT<T>		operator*( const ci::Vec2<T> &rhs );
	TriangleT<T>		operator*( const TriangleT<T> &rhs );
	void				operator*=( const T &rhs );
	void				operator*=( const ci::Vec2<T> &rhs );
	void				operator*=( const TriangleT<T> &rhs );
	
	TriangleT<T>		operator/( const T &rhs );
	TriangleT<T>		operator/( const ci::Vec2<T> &rhs );
	TriangleT<T>		operator/( const TriangleT<T> &rhs );
	void				operator/=( const T &rhs );
	void				operator/=( const ci::Vec2<T> &rhs );
	void				operator/=( const TriangleT<T> &rhs );

	// Comparison operators
	bool				operator==( const TriangleT<T> &rhs );
	bool				operator!=( const TriangleT<T> &rhs );

	///////////////////////////////////////////////////////////////////////////////

	// Point getting shortcuts 
	ci::Vec2<T>&		a();
	const ci::Vec2<T>&	a() const;
	ci::Vec2<T>&		b();
	const ci::Vec2<T>&	b() const;
	ci::Vec2<T>&		c();
	const ci::Vec2<T>&	c() const;

	///////////////////////////////////////////////////////////////////////////////

	// Measurement
	T					distance( const ci::Vec2<T> &point );
	T					distance( const ci::Vec2<T> &point ) const;
	T					distanceSquared( const ci::Vec2<T> &point );
	T					distanceSquared( const ci::Vec2<T> &point ) const;
	ci::Vec2<T>&		getApex();
	const ci::Vec2<T>&	getApex() const;
	T					getArea();
	T					getArea() const;
	ci::RectT<T>		getBounds();
	const ci::RectT<T>&	getBounds() const;
	ci::Vec2<T>&		getCentroid();
	const ci::Vec2<T>&	getCentroid() const;
	ci::Vec2<T>&		getDestination();
	const ci::Vec2<T>&	getDestination() const;
	T					getHeight();
	T					getHeight() const;
	ci::Vec2<T>&		getOrigin();
	const ci::Vec2<T>&	getOrigin() const;
	ci::Vec2<T>			getSize();
	const ci::Vec2<T>	getSize() const;
	T					getWidth();
	T					getWidth() const;

	///////////////////////////////////////////////////////////////////////////////

	// Collision
	ci::Vec2<T>			closestPoint( const ci::Vec2<T> &point );
	ci::Vec2<T>			closestPoint( const ci::Vec2<T> &point ) const;
	bool				contains( const ci::Vec2<T> &point );
	bool				contains( const ci::Vec2<T> &point ) const;
	bool				intersects( const TriangleT<T> &triangle );
	bool				intersects( const TriangleT<T> &triangle ) const;
	ci::Vec2<T>			intersection( const ci::Vec2<T> &point );
	ci::Vec2<T>			intersection( const ci::Vec2<T> &point ) const;

	///////////////////////////////////////////////////////////////////////////////

	// Point setting shortcuts
	void				a( const ci::Vec2<T> &origin );
	void				b( const ci::Vec2<T> &destination );
	void				c( const ci::Vec2<T> &apex );
	
	///////////////////////////////////////////////////////////////////////////////

	// Transformation
	void				include( const ci::Vec2<T> &point );
	void				include( const std::vector<ci::Vec2<T> > &points );
	void				include( const TriangleT &triangle );
	void				inflate( const ci::Vec2<T> &scale );
	TriangleT<T>		inflated( const ci::Vec2<T> &scale );
	void				offset( const ci::Vec2<T> &position );
	void				offsetCenterTo( const ci::Vec2<T> &offset );
	void				rotate( T radians );
	TriangleT<T>		rotated( T radians );
	void				scale( T scale );
	TriangleT<T>		scaled( T scale );
	void				scaleCentered( T scale );
	TriangleT<T>		scaledCentered( T scale );
	void				set( const ci::Vec2<T> &origin, const ci::Vec2<T> &destination, const ci::Vec2<T> &apex );
	void				setApex( const ci::Vec2<T> &apex );
	void				setDestination( const ci::Vec2<T> &destination );
	void				setOrigin( const ci::Vec2<T> &origin );
	TriangleT<T>		transformCopy( const ci::MatrixAffine2<T> &matrix ) const;

	///////////////////////////////////////////////////////////////////////////////

protected:

	static T			calcAngle( const ci::Vec2<T> &a, const ci::Vec2<T> &b );
	static T			calcArea( const TriangleT<T> &triangle );
	static T			calcArea( const ci::Vec2<T> &a, const ci::Vec2<T> &b, const ci::Vec2<T> &c );
	static ci::RectT<T>	calcBoundingBox( const TriangleT<T> &triangle );
	static ci::Vec2<T>	calcCentroid( const TriangleT<T> &triangle );
	static ci::Vec2<T>	calcPoint( const ci::Vec2<T> &origin, T distance, T radians );
	static ci::Vec2<T>	closestPoint( const TriangleT<T> &triangle, const ci::Vec2<T> &p );
	static bool			contains( const TriangleT<T> &triangle, const ci::Vec2<T> &p );
	static T			distance( const TriangleT<T> &triangle, const ci::Vec2<T> &p );
	static T			distanceSquared( const TriangleT<T> &triangle, const ci::Vec2<T> &p );
	static TriangleT<T>	getCentered( const TriangleT<T> &triangle );
	static ci::Vec2<T>	intersection( const TriangleT<T> &triangle, const ci::Vec2<T> &p );
	static bool			intersects( const TriangleT<T> &a, const TriangleT<T> &b );

	ci::Vec2<T>			mApex;			// C
	ci::Vec2<T>			mDestination;	// B
	ci::Vec2<T>			mOrigin;		// A

	T					mArea;
	ci::RectT<T>		mBounds;
	ci::Vec2<T>			mCentroid;

	void				update();

	void				setCentroid( const ci::Vec2<T> &centroid );

	friend				std::ostream& operator<<( std::ostream &out, const TriangleT<T> &triangle );

};

///////////////////////////////////////////////////////////////////////////////

typedef TriangleT<float>	Trianglef;
typedef TriangleT<double>	Triangled;

namespace cinder { namespace gl {
void					drawSolidTriangle( const Trianglef &triangle, bool textureTriangle = false );
void					drawStrokedTriangle( const Trianglef &triangle );
} }
