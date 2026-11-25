// nam_json.h - streaming .nam (JSON) reader for embedded targets.
//
// Feeds a .nam file through in arbitrary chunks and keeps only what the
// A2-Lite engine needs:
//   - the first "weights" array with exactly NAMJ_LITE_WEIGHTS numbers
//     (the A2-Lite slim point of an A2 SlimmableContainer, or a bare A2-Lite WaveNet)
//   - top-level "sample_rate"
//   - top-level metadata: "loudness", "input_level_dbu", "output_level_dbu"
// No allocation, no libc number parsing. Fixed memory: the caller supplies the
// weight buffers.
//
// Copyright (c) 2026 Safie Flato. MIT License.

#ifndef NAM_JSON_H
#define NAM_JSON_H

#include <stdint.h>
#include <string.h>

enum { NAMJ_LITE_WEIGHTS = 1871 };
enum { NAMJ_MAX_DEPTH = 24, NAMJ_KEY_LEN = 24, NAMJ_NUM_LEN = 40 };

struct NamJson
{
	// --- results ---
	float*		weights;		// final destination, NAMJ_LITE_WEIGHTS floats
	float*		scratch;		// candidate buffer, NAMJ_LITE_WEIGHTS floats
	bool		found;			// a Lite-sized weights array was captured
	bool		sawWaveNet;
	bool		sawLeakyReLU;
	bool		hasLoudness, hasInputLevel, hasOutputLevel, hasSampleRate;
	double		loudness, inputLevel, outputLevel, sampleRate;
	bool		error;			// malformed JSON

	// --- tokenizer state ---
	enum { kValue, kKeyOrClose, kKey, kColon, kCommaOrClose, kString, kNumber, kLiteral, kDone };
	uint8_t		state;
	uint8_t		resume;			// state to return to after a string (key or value)
	bool		stringIsKey;
	bool		escape;
	int			depth;
	uint8_t		isObj[ NAMJ_MAX_DEPTH ];
	char		key[ NAMJ_MAX_DEPTH ][ NAMJ_KEY_LEN ];	// current member name at each object level
	char		str[ NAMJ_KEY_LEN ];
	int			strLen;
	char		num[ NAMJ_NUM_LEN ];
	int			numLen;
	// weights capture
	int			wDepth;			// depth of the active "weights" array, or -1
	int			wCount;

	void begin( float* dst, float* tmp )
	{
		memset( this, 0, sizeof *this );
		weights = dst;
		scratch = tmp;
		state = kValue;
		depth = 0;
		wDepth = -1;
	}

	// ---- number conversion (decimal -> float, correctly rounded via double) ----
	static double parseNumber( const char* s, int n )
	{
		static const double p10[] = { 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10,
			1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22 };
		int i = 0;
		bool neg = false;
		if ( i < n && ( s[i] == '-' || s[i] == '+' ) ) { neg = s[i] == '-'; ++i; }
		uint32_t hi = 0, lo = 0;	// 19 digits split 10 + 9 to stay in 32-bit maths
		int digits = 0, exp10 = 0;
		bool seenDot = false;
		for ( ; i < n; ++i )
		{
			char c = s[i];
			if ( c == '.' ) { seenDot = true; continue; }
			if ( c < '0' || c > '9' ) break;
			if ( digits == 0 && c == '0' ) { if ( seenDot ) --exp10; continue; }	// leading zeros
			if ( digits < 18 )
			{
				if ( digits < 9 ) hi = hi * 10 + ( c - '0' );
				else lo = lo * 10 + ( c - '0' );
				++digits;
				if ( seenDot ) --exp10;
			}
			else if ( !seenDot ) ++exp10;	// drop excess integer digits
		}
		if ( i < n && ( s[i] == 'e' || s[i] == 'E' ) )
		{
			++i;
			bool eneg = false;
			if ( i < n && ( s[i] == '-' || s[i] == '+' ) ) { eneg = s[i] == '-'; ++i; }
			int e = 0;
			for ( ; i < n && s[i] >= '0' && s[i] <= '9'; ++i )
				if ( e < 10000 ) e = e * 10 + ( s[i] - '0' );
			exp10 += eneg ? -e : e;
		}
		double v;
		if ( digits > 9 )
		{
			int loDigits = digits - 9;
			v = (double)hi * p10[ loDigits ] + (double)lo;
		}
		else
			v = (double)hi;
		if ( v != 0.0 )
		{
			while ( exp10 > 22 ) { v *= 1e22; exp10 -= 22; if ( v > 1e300 ) break; }
			while ( exp10 < -22 ) { v /= 1e22; exp10 += 22; if ( v < 1e-300 ) { v = 0.0; break; } }
			if ( exp10 > 0 ) v *= p10[ exp10 ];
			else if ( exp10 < 0 ) v /= p10[ -exp10 ];
		}
		return neg ? -v : v;
	}

	// ---- structure helpers ----
	const char* parentKey() const { return depth > 0 && isObj[ depth - 1 ] ? key[ depth - 1 ] : ""; }

	void push( bool obj )
	{
		if ( depth >= NAMJ_MAX_DEPTH ) { error = true; state = kDone; return; }
		// starting an array that is the value of a "weights" member?
		if ( !obj && wDepth < 0 && !found && depth > 0 && isObj[ depth - 1 ] && !strcmp( key[ depth - 1 ], "weights" ) )
		{
			wDepth = depth + 1;
			wCount = 0;
		}
		isObj[ depth ] = obj;
		key[ depth ][0] = 0;
		++depth;
		state = obj ? kKeyOrClose : kValue;
	}

	void pop()
	{
		if ( depth <= 0 ) { error = true; state = kDone; return; }
		if ( wDepth == depth )
		{
			if ( wCount == NAMJ_LITE_WEIGHTS && !found )
			{
				memcpy( weights, scratch, NAMJ_LITE_WEIGHTS * sizeof(float) );
				found = true;
			}
			wDepth = -1;
		}
		--depth;
		state = depth == 0 ? kDone : kCommaOrClose;
	}

	void valueDone() { state = depth == 0 ? kDone : kCommaOrClose; }

	void numberValue()
	{
		if ( wDepth == depth )
		{
			if ( wCount < NAMJ_LITE_WEIGHTS )
				scratch[ wCount ] = (float)parseNumber( num, numLen );
			++wCount;
			return;
		}
		// top level member: depth 1 inside root object
		if ( depth == 1 && isObj[0] && !strcmp( key[0], "sample_rate" ) )
		{
			sampleRate = parseNumber( num, numLen ); hasSampleRate = true;
		}
		// root.metadata.<x>
		else if ( depth == 2 && isObj[0] && isObj[1] && !strcmp( key[0], "metadata" ) )
		{
			if ( !strcmp( key[1], "loudness" ) ) { loudness = parseNumber( num, numLen ); hasLoudness = true; }
			else if ( !strcmp( key[1], "input_level_dbu" ) ) { inputLevel = parseNumber( num, numLen ); hasInputLevel = true; }
			else if ( !strcmp( key[1], "output_level_dbu" ) ) { outputLevel = parseNumber( num, numLen ); hasOutputLevel = true; }
		}
	}

	void stringValue()
	{
		str[ strLen < NAMJ_KEY_LEN ? strLen : NAMJ_KEY_LEN - 1 ] = 0;
		if ( !strcmp( str, "WaveNet" ) ) sawWaveNet = true;
		else if ( !strcmp( str, "LeakyReLU" ) ) sawLeakyReLU = true;
	}

	static bool isWs( char c ) { return c == ' ' || c == '\n' || c == '\r' || c == '\t'; }
	static bool isNumChar( char c ) { return ( c >= '0' && c <= '9' ) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E'; }

	// Feed a chunk. Returns false once the document is complete or broken.
	bool feed( const char* p, uint32_t n )
	{
		for ( uint32_t i = 0; i < n; ++i )
		{
			char c = p[i];
		again:
			switch ( state )
			{
			case kDone:
				return false;
			case kString:
				if ( escape ) { escape = false; break; }
				if ( c == '\\' ) { escape = true; break; }
				if ( c == '"' )
				{
					if ( stringIsKey )
					{
						int l = strLen < NAMJ_KEY_LEN ? strLen : NAMJ_KEY_LEN - 1;
						memcpy( key[ depth - 1 ], str, l );
						key[ depth - 1 ][ l ] = 0;
						state = kColon;
					}
					else
					{
						stringValue();
						valueDone();
					}
					break;
				}
				if ( strLen < NAMJ_KEY_LEN ) str[ strLen ] = c;
				++strLen;
				break;
			case kNumber:
				if ( isNumChar( c ) )
				{
					if ( numLen < NAMJ_NUM_LEN ) num[ numLen++ ] = c;
					break;
				}
				numberValue();
				valueDone();
				goto again;
			case kLiteral:		// true / false / null
				if ( c >= 'a' && c <= 'z' ) break;
				valueDone();
				goto again;
			case kValue:
				if ( isWs( c ) ) break;
				if ( c == '{' ) { push( true ); break; }
				if ( c == '[' ) { push( false ); break; }
				if ( c == ']' && depth > 0 && !isObj[ depth - 1 ] ) { pop(); break; }	// empty array
				if ( c == '"' ) { state = kString; stringIsKey = false; strLen = 0; escape = false; break; }
				if ( isNumChar( c ) ) { state = kNumber; numLen = 0; num[ numLen++ ] = c; break; }
				if ( c >= 'a' && c <= 'z' ) { state = kLiteral; break; }
				error = true; state = kDone; return false;
			case kKeyOrClose:
				if ( isWs( c ) ) break;
				if ( c == '}' ) { pop(); break; }
				// fallthrough
			case kKey:
				if ( isWs( c ) ) break;
				if ( c == '"' ) { state = kString; stringIsKey = true; strLen = 0; escape = false; break; }
				error = true; state = kDone; return false;
			case kColon:
				if ( isWs( c ) ) break;
				if ( c == ':' ) { state = kValue; break; }
				error = true; state = kDone; return false;
			case kCommaOrClose:
				if ( isWs( c ) ) break;
				if ( c == ',' ) { state = isObj[ depth - 1 ] ? kKey : kValue; break; }
				if ( c == '}' && isObj[ depth - 1 ] ) { pop(); break; }
				if ( c == ']' && !isObj[ depth - 1 ] ) { pop(); break; }
				error = true; state = kDone; return false;
			}
		}
		return state != kDone;
	}

	bool complete() const { return state == kDone && !error; }
	// The file looks like an A2-Lite capture (or an A2 container holding one).
	bool isA2Lite() const { return complete() && found && sawWaveNet && sawLeakyReLU; }
};

#endif // NAM_JSON_H
