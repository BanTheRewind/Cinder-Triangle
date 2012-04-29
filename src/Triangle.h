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

// Includes
#include "cinder/app/AppBasic.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"

// Triangle data
class Triangle
{

public:

	// Calculate area between three points
	static float		calcArea( const ci::Vec2f &a, const ci::Vec2f &b, const ci::Vec2f &c );

	// Calculate centroid
	static ci::Vec2f	calcCentroid( const ci::Vec2f &a, const ci::Vec2f &b, const ci::Vec2f &c );

	// Constructor
	Triangle( const ci::Vec2f &origin = ci::Vec2f::zero(), const ci::Vec2f &destination = ci::Vec2f::zero(), 
			  const ci::Vec2f &apex = ci::Vec2f::zero(), int32_t id = 0 );

	// Hit test triangle
	bool				contains( const ci::Vec2f &position );
	bool				contains( const ci::Vec2f &position ) const;

	// Move triangle
	void				move( const ci::Vec2f &offset );

	// Point getter shortcuts
	ci::Vec2f&			a();
	const ci::Vec2f&	a() const;
	ci::Vec2f&			b();
	const ci::Vec2f&	b() const;
	ci::Vec2f&			c();
	const ci::Vec2f&	c() const;

	// Getters
	ci::Vec2f&			getApex();
	const ci::Vec2f&	getApex() const;
	float				getArea();
	float				getArea() const;
	ci::Vec2f&			getCentroid();
	const ci::Vec2f&	getCentroid() const;
	ci::Vec2f&			getDestination();
	const ci::Vec2f&	getDestination() const;
	int32_t				getId();
	int32_t				getId() const;
	ci::Vec2f&			getOrigin();
	const ci::Vec2f&	getOrigin() const;
	ci::Vec2f			getVelocity();
	const ci::Vec2f		getVelocity() const;

	// Point setter shortcuts
	void				a( const ci::Vec2f &origin );
	void				b( const ci::Vec2f &destination );
	void				c( const ci::Vec2f &apex );
	
	// Update area for this triangle
	float				calcArea();

	// Setters
	void				setApex( const ci::Vec2f &apex );
	void				setArea( float area );
	void				setDestination( const ci::Vec2f &destination );
	void				setId( int32_t id );
	void				setOrigin( const ci::Vec2f &origin );
	void				setPosition( const ci::Vec2f &position );

private:

	ci::Vec2f			mApex;			// C
	ci::Vec2f			mDestination;	// B
	ci::Vec2f			mOrigin;		// A

	float				mArea;
	ci::Vec2f			mCentroid;
	int32_t				mId;
	ci::Vec2f			mPrevCentroid;

	void				setCentroid( const ci::Vec2f &centroid );

};

namespace cinder { namespace gl {
void					drawSolidTriangle( const Triangle &triangle, bool textureTriangle = false );
void					drawStrokedTriangle( const Triangle &triangle );
} }
