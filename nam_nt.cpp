// NAM - Neural Amp Modeler for the Expert Sleepers disting NT
//
// Runs NAM A2-Lite captures (e.g. from tone3000.com) on the disting NT.
// Drop .nam (or .namb) files in /nam on the SD card and pick one with the
// Model parameter.
//
// Inference: TONE3000's dependency-free Cortex-M7 A2-Lite engine (MIT),
// https://github.com/tone-3000/embedded
//
// Copyright (c) 2026 Safie Flato. MIT License.

#include <string.h>
#include <new>
#include <distingnt/api.h>
#include <distingnt/file.h>
#include <distingnt/serialisation.h>
#include <distingnt/wav.h>

extern "C" {
#include "engine/nam_a2lite.h"
}
#include "engine/namb_reader.h"
#include "nam_json.h"

// ---------------------------------------------------------------------------
// configuration

#define NAM_FOLDER			"/nam"
enum { kMaxFiles = 128, kNameLen = 128 };
enum { kChunk = NAM_A2LITE_MAX_BUFFER_SIZE };		// engine frames per call
enum { kMaxNambBytes = 16 * 1024 };					// A2-Lite .namb is ~8 KB
static const float kVoltsFullScale = 5.0f;			// 5 V peak = 0 dBFS into the model
static const float kOwnDbu = 13.2f;					// 5 V peak sine = 3.54 Vrms = +13.2 dBu
static const float kTargetLoudness = -18.0f;		// NAM loudness normalisation target

// ---------------------------------------------------------------------------
// parameters

enum
{
	kParamModel,
	kParamInputGain,
	kParamOutputGain,
	kParamMix,
	kParamCalibrate,
	kParamNormalize,
	kParamGate,
	kParamStereoMode,
	kParamInput,
	kParamInputR,
	kParamOutput,
	kParamOutputMode,
	kParamOutputR,
	kParamOutputRMode,
	kParamDriveCv,

	kNumParams,
};

static char const * const offOn[] = { "Off", "On", NULL };
static char const * const stereoModes[] = { "Dual", "Summed", NULL };
enum { kStereoDual, kStereoSummed };

static const _NT_parameter parameters[] = {
	{ .name = "Model", .min = 0, .max = 32767, .def = 0, .unit = kNT_unitConfirm, .scaling = 0, .enumStrings = NULL },
	{ .name = "Input gain", .min = -400, .max = 400, .def = 0, .unit = kNT_unitDb, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Output gain", .min = -400, .max = 240, .def = 0, .unit = kNT_unitDb, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Mix", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Calibrate in", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOn },
	{ .name = "Normalize out", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOn },
	{ .name = "Gate", .min = -90, .max = -30, .def = -90, .unit = kNT_unitDb_minInf, .scaling = 0, .enumStrings = NULL },
	{ .name = "Stereo mode", .min = 0, .max = 1, .def = kStereoDual, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = stereoModes },
	NT_PARAMETER_AUDIO_INPUT( "Input", 1, 1 )
	NT_PARAMETER_AUDIO_INPUT( "Input R", 0, 2 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output R", 0, 14 )
	NT_PARAMETER_CV_INPUT( "Drive CV", 0, 0 )
};

static const uint8_t pageModel[] = { kParamModel, kParamInputGain, kParamOutputGain, kParamMix };
static const uint8_t pageLevels[] = { kParamCalibrate, kParamNormalize, kParamGate };
static const uint8_t pageRoutingMono[] = { kParamInput, kParamOutput, kParamOutputMode, kParamDriveCv };
static const uint8_t pageRoutingStereo[] = { kParamStereoMode, kParamInput, kParamInputR, kParamOutput, kParamOutputMode,
		kParamOutputR, kParamOutputRMode, kParamDriveCv };

static const _NT_parameterPage pagesMono[] = {
	{ .name = "Amp", .numParams = ARRAY_SIZE(pageModel), .params = pageModel },
	{ .name = "Levels", .numParams = ARRAY_SIZE(pageLevels), .params = pageLevels },
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRoutingMono), .params = pageRoutingMono },
};
static const _NT_parameterPage pagesStereo[] = {
	{ .name = "Amp", .numParams = ARRAY_SIZE(pageModel), .params = pageModel },
	{ .name = "Levels", .numParams = ARRAY_SIZE(pageLevels), .params = pageLevels },
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRoutingStereo), .params = pageRoutingStereo },
};

static const _NT_parameterPages parameterPagesMono = { .numPages = ARRAY_SIZE(pagesMono), .pages = pagesMono };
static const _NT_parameterPages parameterPagesStereo = { .numPages = ARRAY_SIZE(pagesStereo), .pages = pagesStereo };

// specification: 1 = mono, 2 = stereo (chosen when the algorithm is added)
static const _NT_specification specifications[] = {
	{ .name = "Channels", .min = 1, .max = 2, .def = 1, .type = kNT_typeGeneric },
};

// ---------------------------------------------------------------------------
// 2x halfband resampler for 96 kHz operation (63 taps, -75 dB stopband from 28 kHz)

enum { kHbTaps = 63, kHbCenter = 31 };
static const float hbCoef[16] = {	// taps at center -/+ 1, 3, 5, ... 31
	3.171568977e-01f, -1.026843181e-01f, 5.810452681e-02f, -3.797827477e-02f,
	2.619762634e-02f, -1.839548359e-02f, 1.289847136e-02f, -8.917750670e-03f,
	6.018522613e-03f, -3.927030553e-03f, 2.450615651e-03f, -1.442142586e-03f,
	7.837316743e-04f, -3.793261595e-04f, 1.512411428e-04f, -3.829076312e-05f,
};

struct Halfband
{
	float	h[ 2 * kHbTaps ];	// doubled history, avoids wrapping on read
	int		pos;

	void reset() { memset( this, 0, sizeof *this ); }
	inline void push( float x )
	{
		pos = pos == 0 ? kHbTaps - 1 : pos - 1;
		h[ pos ] = x;
		h[ pos + kHbTaps ] = x;
	}
	// newest sample at h[pos], oldest at h[pos+62]
	inline float filter() const
	{
		const float* x = h + pos;
		float acc = 0.5f * x[ kHbCenter ];
		for ( int k = 0; k < 16; ++k )
			acc += hbCoef[k] * ( x[ kHbCenter - 1 - 2 * k ] + x[ kHbCenter + 1 + 2 * k ] );
		return acc;
	}
};

// ---------------------------------------------------------------------------
// memory layout

// one engine channel
struct NamChan
{
	nam_a2lite_t			inst;
	float					in[ kChunk ];
	float					out[ kChunk ];
	Halfband				down, up;
};

// hot data: the capture (shared by both channels) and the channels (DTC)
struct NamHot
{
	nam_a2lite_weights_t	weights;
	NamChan					ch[1];		// [numChannels]
};
static inline uint32_t hotSize( int channels ) { return sizeof(NamHot) + ( channels - 1 ) * sizeof(NamChan); }

// big, rarely touched data (DRAM)
struct NamCold
{
	char		files[ kMaxFiles ][ kNameLen ];
	NamJson		json;
	float		parsed[ NAMJ_LITE_WEIGHTS ];
	float		candidate[ NAMJ_LITE_WEIGHTS ];
	uint32_t	nambBuf[ kMaxNambBytes / 4 ];	// 4-byte aligned for namb_parse
};

enum Status
{
	kStatusNoCard,
	kStatusScanning,
	kStatusNoFiles,
	kStatusLoading,
	kStatusReady,
	kStatusError,
};

struct _namAlgorithm : public _NT_algorithm
{
	_NT_parameter	params[ kNumParams ];

	NamHot*					hot;
	NamCold*				cold;
	nam_a2lite_state_t*		state;		// ring buffers (SRAM), [numChannels]
	int						numChannels;

	// file system / loading state machine
	bool			cardMounted;
	volatile bool	fsBusy;				// a file system callback is queued or running
	volatile bool	scanDone;			// scan finished - step() publishes the result
	volatile bool	loadDone;			// load finished - step() publishes the result
	volatile bool	engineReady;		// engine may be processed
	bool			loadPending;		// a load was requested, waiting to start
	int				numFiles;
	int				loadIndex;
	char			wantedName[ kNameLen ];	// from a preset, matched after scanning
	Status			status;
	const char*		errorText;
	char			modelName[ kNameLen ];

	// model gain staging
	bool			hasLoudness, hasInputLevel;
	float			loudness, inputLevelDbu;
	bool			sampleRateWarn;

	// runtime gains and smoothing
	float			inGainTarget, outGainTarget, mix;
	float			inGain, outGain;	// smoothed
	float			fade;				// 0..1 fade after (re)load
	float			gateThresh;			// linear, 0 = off
	float			env, gateGain;
	float			inPeak, outPeak;	// meters
	// 96k: dry path delayed to match the resampler latency (61 samples)
	float			dryBuf[ 2 ][ 64 ];
	int				dryPos;
};

// ---------------------------------------------------------------------------
// helpers

// small, libm-free exp2/log2 (gain maths and meters only)
static inline float fastExp2( float x )
{
	if ( x < -126.0f ) return 0.0f;
	if ( x > 126.0f ) x = 126.0f;
	int i = (int)x;
	if ( x < (float)i ) --i;
	float f = x - (float)i;
	float r = 1.0f + f * ( 0.6931472f + f * ( 0.2402265f + f * ( 0.0555041f + f * ( 0.0096181f + f * 0.0013334f ) ) ) );
	union { uint32_t u; float f; } b;
	b.u = (uint32_t)( i + 127 ) << 23;
	return r * b.f;
}
static inline float fastLog2( float x )
{
	union { float f; uint32_t u; } b = { x };
	float e = (float)( (int)( ( b.u >> 23 ) & 255 ) - 127 );
	b.u = ( b.u & 0x007fffff ) | 0x3f800000;		// mantissa in [1,2)
	float m = b.f - 1.0f;
	return e + m * ( 1.4425449f + m * ( -0.7181452f + m * 0.2754018f ) );
}
static inline float dbToGain( float db ) { return fastExp2( db * 0.16609640474f ); }

static int nameCompare( const char* a, const char* b )
{
	for ( ;; ++a, ++b )
	{
		char ca = *a, cb = *b;
		if ( ca >= 'A' && ca <= 'Z' ) ca += 32;
		if ( cb >= 'A' && cb <= 'Z' ) cb += 32;
		if ( ca != cb || !ca ) return (unsigned char)ca - (unsigned char)cb;
	}
}

static bool endsWith( const char* s, const char* ext )
{
	size_t ls = strlen( s ), le = strlen( ext );
	return ls >= le && nameCompare( s + ls - le, ext ) == 0;
}

static void copyName( char* dst, const char* src )
{
	strncpy( dst, src, kNameLen - 1 );
	dst[ kNameLen - 1 ] = 0;
}

static void updateGains( _namAlgorithm* p )
{
	float inDb = p->v[ kParamInputGain ] * 0.1f;
	if ( p->v[ kParamCalibrate ] && p->hasInputLevel )
		inDb += kOwnDbu - p->inputLevelDbu;
	p->inGainTarget = dbToGain( inDb ) / kVoltsFullScale;

	float outDb = p->v[ kParamOutputGain ] * 0.1f;
	if ( p->v[ kParamNormalize ] && p->hasLoudness )
	{
		float n = kTargetLoudness - p->loudness;
		n = n < -12.0f ? -12.0f : ( n > 12.0f ? 12.0f : n );
		outDb += n;
	}
	p->outGainTarget = dbToGain( outDb ) * kVoltsFullScale;

	p->mix = p->v[ kParamMix ] * 0.01f;
	int g = p->v[ kParamGate ];
	p->gateThresh = g <= -90 ? 0.0f : dbToGain( (float)g );
}

// ---------------------------------------------------------------------------
// file system callbacks (run on the host's file system task, never during step())

static void scanCallback( void* data )
{
	_namAlgorithm* p = (_namAlgorithm*)data;
	NamCold* c = p->cold;
	int n = 0;
	if ( NT_chdir( NAM_FOLDER ) == kNT_fileOK )
	{
		uint32_t found = NT_findFiles( "*.nam*", kNT_findFiles );
		char buf[ 256 ];
		for ( uint32_t i = 0; i < found && n < kMaxFiles; ++i )
		{
			NT_getFoundFile( i, buf, sizeof buf );
			if ( buf[0] == '.' ) continue;					// macOS ._ files
			if ( !endsWith( buf, ".nam" ) && !endsWith( buf, ".namb" ) ) continue;
			if ( strlen( buf ) >= kNameLen ) continue;		// can't reopen a truncated name
			// insertion sort, case-insensitive
			int j = n;
			while ( j > 0 && nameCompare( c->files[ j - 1 ], buf ) > 0 )
			{
				memcpy( c->files[ j ], c->files[ j - 1 ], kNameLen );
				--j;
			}
			copyName( c->files[ j ], buf );
			++n;
		}
	}
	p->numFiles = n;
	p->scanDone = true;
	p->fsBusy = false;
}

static const char* loadNam( _namAlgorithm* p, _NT_fileHandle F, void* scratch, size_t scratchSize )
{
	NamCold* c = p->cold;
	NamJson& j = c->json;
	j.begin( c->parsed, c->candidate );
	for ( ;; )
	{
		uint32_t got = 0;
		if ( NT_fileRead( F, scratch, scratchSize, got ) != kNT_fileOK )
			return "Read error";
		if ( got == 0 )
			break;
		if ( !j.feed( (const char*)scratch, got ) )
			break;
	}
	if ( j.error || !j.complete() )
		return "Bad .nam file";
	if ( !j.found )
		return "No A2-Lite inside";
	if ( !j.sawWaveNet || !j.sawLeakyReLU )
		return "Not an A2 capture";
	if ( nam_a2lite_load_weights( &p->hot->weights, c->parsed, NAMJ_LITE_WEIGHTS ) != 0 )
		return "Weight count";
	p->hasLoudness = j.hasLoudness;
	p->loudness = (float)j.loudness;
	p->hasInputLevel = j.hasInputLevel;
	p->inputLevelDbu = (float)j.inputLevel;
	return NULL;
}

static const char* loadNamb( _namAlgorithm* p, _NT_fileHandle F, uint32_t size )
{
	NamCold* c = p->cold;
	if ( size > kMaxNambBytes )
		return "A2-Full: too big";
	uint32_t got = 0;
	if ( NT_fileRead( F, c->nambBuf, size, got ) != kNT_fileOK || got != size )
		return "Read error";
	namb_model_t m;
	namb_status_t s = namb_parse( c->nambBuf, size, &m );
	if ( s == NAMB_OK ) s = namb_verify_crc( c->nambBuf, size );
	if ( s == NAMB_OK ) s = namb_check_a2lite( &m );
	if ( s != NAMB_OK )
		return s == NAMB_ERR_SHAPE ? "Not A2-Lite" : namb_status_str( s );
	if ( nam_a2lite_load_weights( &p->hot->weights, m.weights, (int)m.weight_count ) != 0 )
		return "Weight count";
	// metadata block at offset 32: u8 ver[3], u8 flags, f64 sr, f64 loudness, f64 in, f64 out
	const uint8_t* b = (const uint8_t*)c->nambBuf;
	uint8_t flags = b[35];
	double d;
	p->hasLoudness = flags & 0x01;
	memcpy( &d, b + 44, 8 ); p->loudness = (float)d;
	p->hasInputLevel = flags & 0x02;
	memcpy( &d, b + 52, 8 ); p->inputLevelDbu = (float)d;
	return NULL;
}

static void loadCallback( void* data )
{
	_namAlgorithm* p = (_namAlgorithm*)data;
	const char* err = NULL;
	const char* name = p->cold->files[ p->loadIndex ];

	if ( NT_chdir( NAM_FOLDER ) != kNT_fileOK )
		err = "No /nam folder";
	else
	{
		_NT_fileInfo info;
		_NT_fileHandle F = 0;
		size_t scratchSize = 0;
		void* scratch = NT_getFileScratchBuffer( scratchSize );
		if ( !scratch || scratchSize == 0 )
			err = "No FS buffer";
		else if ( NT_fileInfo( name, info ) != kNT_fileOK || NT_fileOpen( F, name, kNT_modeRead ) != kNT_fileOK )
			err = "Can't open file";
		else
		{
			if ( endsWith( name, ".namb" ) )
				err = loadNamb( p, F, (uint32_t)info.size );
			else
				err = loadNam( p, F, scratch, scratchSize );
			NT_fileClose( F );
		}
	}

	if ( !err )
	{
		// bind, clear and prewarm (not real time - fine here, audio is muted)
		for ( int c = 0; c < p->numChannels; ++c )
		{
			NamChan& ch = p->hot->ch[c];
			nam_a2lite_init( &ch.inst, &p->hot->weights, p->state + c );
			nam_a2lite_prewarm( &ch.inst );
			ch.down.reset();
			ch.up.reset();
		}
		p->env = 0.0f;
		p->gateGain = 1.0f;
		copyName( p->modelName, name );
		p->status = kStatusReady;
		p->errorText = NULL;
		p->fade = 0.0f;
		updateGains( p );					// new capture's levels; output is faded in anyway
		p->inGain = p->inGainTarget;
		p->outGain = p->outGainTarget;
		__sync_synchronize();
		p->engineReady = true;
	}
	else
	{
		p->status = kStatusError;
		p->errorText = err;
		copyName( p->modelName, name );
	}
	p->loadDone = true;
	p->fsBusy = false;
}

// ---------------------------------------------------------------------------
// factory functions

static void calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	int channels = specifications[0] == 2 ? 2 : 1;
	req.numParameters = kNumParams;
	req.sram = sizeof(_namAlgorithm) + channels * sizeof(nam_a2lite_state_t) + 32;
	req.dram = sizeof(NamCold) + 32;
	req.dtc = hotSize( channels ) + 32;
	req.itc = 0;
}

static inline uint8_t* align32( uint8_t* q ) { return (uint8_t*)( ( (uintptr_t)q + 31 ) & ~(uintptr_t)31 ); }

static _NT_algorithm* construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	static_assert( kNumParams == ARRAY_SIZE(parameters), "parameter count" );

	_namAlgorithm* p = new ( ptrs.sram ) _namAlgorithm();
	p->numChannels = specifications[0] == 2 ? 2 : 1;
	p->state = (nam_a2lite_state_t*)align32( ptrs.sram + sizeof(_namAlgorithm) );
	p->hot = (NamHot*)align32( ptrs.dtc );
	p->cold = (NamCold*)align32( ptrs.dram );
	memset( p->hot, 0, hotSize( p->numChannels ) );
	memset( p->state, 0, p->numChannels * sizeof(nam_a2lite_state_t) );

	memcpy( p->params, parameters, sizeof parameters );
	if ( p->numChannels == 2 )
	{
		p->params[ kParamInput ].name = "Input L";
		p->params[ kParamOutput ].name = "Output L";
		p->params[ kParamOutputMode ].name = "Output L mode";
	}
	p->parameters = p->params;
	p->parameterPages = p->numChannels == 2 ? &parameterPagesStereo : &parameterPagesMono;

	p->cardMounted = false;
	p->fsBusy = p->scanDone = p->loadDone = p->engineReady = false;
	p->loadPending = false;
	p->numFiles = 0;
	p->loadIndex = 0;
	p->wantedName[0] = 0;
	p->modelName[0] = 0;
	p->status = kStatusNoCard;
	p->errorText = NULL;
	p->hasLoudness = p->hasInputLevel = false;
	p->sampleRateWarn = NT_globals.sampleRate != 48000 && NT_globals.sampleRate != 96000;
	p->inGain = p->outGain = 0.0f;
	p->fade = 0.0f;
	p->env = 0.0f;
	p->gateGain = 1.0f;
	p->inPeak = p->outPeak = 0.0f;
	memset( p->dryBuf, 0, sizeof p->dryBuf );
	p->dryPos = 0;
	return p;
}

static void requestLoad( _namAlgorithm* p )
{
	if ( p->numFiles > 0 )
		p->loadPending = true;
}

static void parameterChanged( _NT_algorithm* self, int param )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	switch ( param )
	{
	case kParamModel:
		requestLoad( p );
		break;
	default:
		updateGains( p );
		break;
	}
}

static int parameterString( _NT_algorithm* self, int param, int v, char* buff )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	if ( param != kParamModel )
		return 0;
	const char* s;
	if ( p->numFiles > 0 && v >= 0 && v < p->numFiles )
		s = p->cold->files[ v ];
	else if ( p->status == kStatusNoCard )
		s = "No SD card";
	else if ( p->status == kStatusScanning )
		s = "Scanning...";
	else
		s = "No files in /nam";
	strncpy( buff, s, kNT_parameterStringSize - 1 );
	buff[ kNT_parameterStringSize - 1 ] = 0;
	return strlen( buff );
}

// ---------------------------------------------------------------------------
// audio

static void housekeeping( _namAlgorithm* p, _NT_algorithm* self )
{
	// SD card mount changes -> rescan
	bool mounted = NT_isSdCardMounted();
	if ( mounted != p->cardMounted )
	{
		p->cardMounted = mounted;
		if ( mounted )
			p->status = kStatusScanning;
		else
		{
			p->status = kStatusNoCard;
			p->numFiles = 0;
		}
	}
	if ( p->cardMounted && p->status == kStatusScanning && !p->fsBusy && !p->scanDone )
	{
		p->fsBusy = true;
		if ( !NT_callFileSystemCode( scanCallback, p ) )
			p->fsBusy = false;
	}

	int algIndex = NT_algorithmIndex( self );

	// publish scan results
	if ( p->scanDone )
	{
		p->scanDone = false;
		p->params[ kParamModel ].max = p->numFiles > 0 ? p->numFiles - 1 : 0;
		NT_updateParameterDefinition( algIndex, kParamModel );
		if ( p->numFiles == 0 )
			p->status = kStatusNoFiles;
		else
			p->loadPending = true;
	}

	// a preset named its model: select it by name once the files are known
	if ( p->wantedName[0] && p->numFiles > 0 )
	{
		for ( int i = 0; i < p->numFiles; ++i )
			if ( !nameCompare( p->cold->files[i], p->wantedName ) )
			{
				if ( p->v[ kParamModel ] != i )
					NT_setParameterFromAudio( algIndex, kParamModel + NT_parameterOffset(), i );	// -> parameterChanged -> load
				break;
			}
		p->wantedName[0] = 0;
	}

	if ( p->loadDone )
	{
		p->loadDone = false;
		updateGains( p );
	}

	// start a pending load once the output has faded out
	if ( p->loadPending && !p->fsBusy && p->numFiles > 0 && ( !p->engineReady || p->fade <= 0.0f ) )
	{
		int idx = p->v[ kParamModel ];
		if ( idx < 0 ) idx = 0;
		if ( idx >= p->numFiles ) idx = p->numFiles - 1;
		p->engineReady = false;
		__sync_synchronize();
		p->loadIndex = idx;
		p->status = kStatusLoading;
		p->fsBusy = true;
		if ( NT_callFileSystemCode( loadCallback, p ) )
			p->loadPending = false;
		else
		{
			p->fsBusy = false;
			p->status = kStatusError;
			p->errorText = "FS busy";
		}
	}
}

static void step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	const int numFrames = numFramesBy4 * 4;

	housekeeping( p, self );

	// ---- routing ----
	// Mono spec: one input, one output.
	// Stereo spec: Input R / Output R may be None.
	//   Dual:   L and R through their own engines (true stereo, 2x CPU)
	//   Summed: (L+R)/2 through one engine, wet to both sides, dry stays stereo
	//   No Input R: mono in, the wet signal goes to both outputs.
	const bool stereo = p->numChannels == 2;
	const int vInL = p->v[ kParamInput ], vOutL = p->v[ kParamOutput ];
	const int vInR = stereo ? p->v[ kParamInputR ] : 0;
	const int vOutR = stereo ? p->v[ kParamOutputR ] : 0;
	const float* inL = vInL ? busFrames + ( vInL - 1 ) * numFrames : NULL;
	const float* inR = vInR ? busFrames + ( vInR - 1 ) * numFrames : NULL;
	float* outL = vOutL ? busFrames + ( vOutL - 1 ) * numFrames : NULL;
	float* outR = vOutR ? busFrames + ( vOutR - 1 ) * numFrames : NULL;
	const bool replaceL = p->v[ kParamOutputMode ];
	const bool replaceR = stereo && p->v[ kParamOutputRMode ];
	const float* cv = p->v[ kParamDriveCv ] ? busFrames + ( p->v[ kParamDriveCv ] - 1 ) * numFrames : NULL;

	if ( !outL && !outR )
		return;

	const int engines = ( stereo && inR && p->v[ kParamStereoMode ] == kStereoDual ) ? 2 : 1;
	const float sumScale = ( inL && inR && engines == 1 ) ? 0.5f : 1.0f;

	const bool running = p->engineReady;
	// fade out ahead of a load, fade in after one (about 10 ms)
	const float fadeTarget = ( running && !p->loadPending ) ? 1.0f : 0.0f;
	const float fadeStep = 1.0f / ( 0.01f * NT_globals.sampleRate );

	// drive CV: 1 V = +6 dB, sampled once per block
	float inTarget = p->inGainTarget;
	if ( cv )
		inTarget *= dbToGain( 6.0f * cv[0] );

	const float smooth = 0.002f;
	const float mix = p->mix;
	const float gateThresh = p->gateThresh;
	const bool is96k = NT_globals.sampleRate == 96000;

	NamHot* h = p->hot;
	NamChan& c0 = h->ch[0];
	NamChan& c1 = h->ch[ engines == 2 ? 1 : 0 ];
	float inPeak = p->inPeak * 0.95f, outPeak = p->outPeak * 0.95f;

	// flush denormals to zero and use default NaN while the engine runs
#ifdef __arm__
	uint32_t fpscr;
	__asm__ volatile( "vmrs %0, fpscr" : "=r"( fpscr ) );
	__asm__ volatile( "vmsr fpscr, %0" : : "r"( fpscr | ( 1u << 24 ) | ( 1u << 25 ) ) );
#endif

	int done = 0;
	while ( done < numFrames )
	{
		// frames at the host rate in this chunk
		int n = numFrames - done;
		int maxHost = is96k ? 2 * kChunk : kChunk;
		if ( n > maxHost ) n = maxHost;
		if ( is96k ) n &= ~1;

		const float* xl = inL ? inL + done : NULL;
		const float* xr = inR ? inR + done : NULL;

		// --- gain + linked gate, down to model rate ---
		int m = 0;
		float ig = p->inGain;
		float env = p->env, gg = p->gateGain;
		for ( int i = 0; i < n; ++i )
		{
			ig += smooth * ( inTarget - ig );
			float l = xl ? xl[i] : 0.0f;
			float r = xr ? xr[i] : 0.0f;
			float a, b;
			if ( engines == 2 ) { a = l * ig; b = r * ig; }
			else { a = ( l + r ) * sumScale * ig; b = 0.0f; }
			float pk = __builtin_fabsf( a ) > __builtin_fabsf( b ) ? __builtin_fabsf( a ) : __builtin_fabsf( b );
			if ( pk > inPeak ) inPeak = pk;
			// gate on the model input: fast attack, slow release, both sides together
			if ( gateThresh > 0.0f )
			{
				env = pk > env ? pk : env * 0.9995f;
				float tgt = env > gateThresh ? 1.0f : 0.0f;
				gg += ( tgt > gg ? 0.01f : 0.0005f ) * ( tgt - gg );
				a *= gg;
				b *= gg;
			}
			if ( is96k )
			{
				c0.down.push( a );
				if ( engines == 2 ) c1.down.push( b );
				if ( i & 1 )
				{
					c0.in[ m ] = c0.down.filter();
					if ( engines == 2 ) c1.in[ m ] = c1.down.filter();
					++m;
				}
			}
			else
			{
				c0.in[ m ] = a;
				if ( engines == 2 ) c1.in[ m ] = b;
				++m;
			}
		}
		p->inGain = ig;
		p->env = env;
		p->gateGain = gg;

		// --- model(s) ---
		for ( int e = 0; e < engines; ++e )
		{
			NamChan& ch = h->ch[e];
			if ( running )
			{
				const float* ins[1] = { ch.in };
				float* outs[1] = { ch.out };
				nam_a2lite_process( &ch.inst, ins, outs, m );
			}
			else
				memset( ch.out, 0, m * sizeof(float) );
		}

		// --- back up to host rate, gain, mix ---
		float og = p->outGain, fade = p->fade;
		for ( int i = 0; i < n; ++i )
		{
			float wl, wr;
			if ( is96k )
			{
				c0.up.push( ( i & 1 ) ? 0.0f : 2.0f * c0.out[ i >> 1 ] );
				wl = c0.up.filter();
				if ( engines == 2 )
				{
					c1.up.push( ( i & 1 ) ? 0.0f : 2.0f * c1.out[ i >> 1 ] );
					wr = c1.up.filter();
				}
				else
					wr = wl;
			}
			else
			{
				wl = c0.out[i];
				wr = engines == 2 ? c1.out[i] : wl;
			}
			og += smooth * ( p->outGainTarget - og );
			fade += fade < fadeTarget ? fadeStep : ( fade > fadeTarget ? -fadeStep : 0.0f );
			fade = fade < 0.0f ? 0.0f : ( fade > 1.0f ? 1.0f : fade );
			float g = og * fade;
			wl *= g;
			wr *= g;
			float pk = __builtin_fabsf( wl ) > __builtin_fabsf( wr ) ? __builtin_fabsf( wl ) : __builtin_fabsf( wr );
			if ( pk > outPeak ) outPeak = pk;

			float dl = xl ? xl[i] : 0.0f;
			float dr = xr ? xr[i] : dl;		// mono in: dry to both sides
			if ( is96k )
			{
				// delay the dry path to match the resampler latency
				p->dryBuf[0][ p->dryPos ] = dl;
				p->dryBuf[1][ p->dryPos ] = dr;
				int rd = ( p->dryPos - 61 ) & 63;
				dl = p->dryBuf[0][ rd ];
				dr = p->dryBuf[1][ rd ];
				p->dryPos = ( p->dryPos + 1 ) & 63;
			}
			if ( outL )
			{
				float o = dl + mix * ( wl - dl );
				if ( replaceL ) outL[ done + i ] = o; else outL[ done + i ] += o;
			}
			if ( outR )
			{
				float o = dr + mix * ( wr - dr );
				if ( replaceR ) outR[ done + i ] = o; else outR[ done + i ] += o;
			}
		}
		p->outGain = og;
		p->fade = fade;

		done += n;
	}

#ifdef __arm__
	__asm__ volatile( "vmsr fpscr, %0" : : "r"( fpscr ) );
#endif

	p->inPeak = inPeak;
	p->outPeak = outPeak;
}

// ---------------------------------------------------------------------------
// display

static void meter( int x, int y, const char* label, float level )
{
	NT_drawText( x, y + 6, label, 8 );
	// level in dB relative to full scale, -48..0 dB over 80 px
	float db = level > 1e-6f ? 6.0206f * fastLog2( level ) : -120.0f;
	int w = (int)( ( db + 48.0f ) * ( 80.0f / 48.0f ) );
	w = w < 0 ? 0 : ( w > 80 ? 80 : w );
	NT_drawShapeI( kNT_box, x + 22, y, x + 102, y + 5, 5 );
	if ( w > 0 )
		NT_drawShapeI( kNT_rectangle, x + 22, y, x + 22 + w, y + 5, db > -1.0f ? 15 : 10 );
}

static bool draw( _NT_algorithm* self )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	char buf[ 48 ];

	const char* title = "NAM";
	if ( p->numChannels == 2 )
		title = !p->v[ kParamInputR ] ? "NAM  mono > stereo"
			: ( p->v[ kParamStereoMode ] == kStereoDual ? "NAM  stereo (dual)" : "NAM  stereo (summed)" );
	NT_drawText( 0, 20, title, 15 );

	const char* line;
	switch ( p->status )
	{
	case kStatusNoCard:		line = "No SD card"; break;
	case kStatusScanning:	line = "Scanning " NAM_FOLDER "..."; break;
	case kStatusNoFiles:	line = "Put .nam files in " NAM_FOLDER; break;
	case kStatusLoading:	line = "Loading..."; break;
	case kStatusError:		line = p->errorText ? p->errorText : "Error"; break;
	default:				line = p->modelName; break;
	}
	NT_drawText( 0, 32, line, p->status == kStatusError ? 15 : 12 );
	if ( p->status == kStatusError && p->modelName[0] )
		NT_drawText( 0, 42, p->modelName, 6 );
	else if ( p->status == kStatusReady )
	{
		int x = 0;
		if ( p->hasLoudness )
		{
			int len = NT_floatToString( buf, p->loudness, 1 );
			strcpy( buf + len, "dB loud" );
			NT_drawText( x, 42, buf, 6 );
			x += 80;
		}
		if ( p->hasInputLevel )
		{
			int len = NT_floatToString( buf, p->inputLevelDbu, 1 );
			strcpy( buf + len, p->v[ kParamCalibrate ] ? "dBu cal" : "dBu (off)" );
			NT_drawText( x, 42, buf, 6 );
		}
	}
	if ( p->sampleRateWarn )
		NT_drawText( 150, 20, "SR not 48k!", 15 );

	// meters: model input (re full scale) and output (re 5 V)
	meter( 140, 44, "In", p->inPeak );
	meter( 140, 54, "Out", p->outPeak / kVoltsFullScale );
	return false;
}

// ---------------------------------------------------------------------------
// presets: remember the model by name, so reordering files doesn't change it

static void serialise( _NT_algorithm* self, _NT_jsonStream& stream )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	stream.addMemberName( "model" );
	if ( p->numFiles > 0 && p->v[ kParamModel ] < p->numFiles )
		stream.addString( p->cold->files[ p->v[ kParamModel ] ] );
	else
		stream.addString( p->wantedName );
}

static bool deserialise( _NT_algorithm* self, _NT_jsonParse& parse )
{
	_namAlgorithm* p = (_namAlgorithm*)self;
	int members;
	if ( !parse.numberOfObjectMembers( members ) )
		return false;
	for ( int i = 0; i < members; ++i )
	{
		if ( parse.matchName( "model" ) )
		{
			const char* s;
			if ( !parse.string( s ) )
				return false;
			copyName( p->wantedName, s );		// matched in step() once files are known
		}
		else if ( !parse.skipMember() )
			return false;
	}
	return true;
}

// ---------------------------------------------------------------------------

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'S', 'f', 'N', 'm' ),
	.name = "NAM",
	.description = "Neural Amp Modeler (A2-Lite captures)",
	.numSpecifications = ARRAY_SIZE(specifications),
	.specifications = specifications,
	.calculateStaticRequirements = NULL,
	.initialise = NULL,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = parameterChanged,
	.step = step,
	.draw = draw,
	.midiRealtime = NULL,
	.midiMessage = NULL,
	.tags = kNT_tagEffect,
	.hasCustomUi = NULL,
	.customUi = NULL,
	.setupUi = NULL,
	.serialise = serialise,
	.deserialise = deserialise,
	.midiSysEx = NULL,
	.parameterUiPrefix = NULL,
	.parameterString = parameterString,
};

extern "C" uintptr_t pluginEntry( _NT_selector selector, uint32_t data )
{
	switch ( selector )
	{
	case kNT_selector_version:
		return kNT_apiVersion14;	// newest feature used: filesystem API (added after v14). Nothing from v15/v16.
	case kNT_selector_numFactories:
		return 1;
	case kNT_selector_factoryInfo:
		return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
