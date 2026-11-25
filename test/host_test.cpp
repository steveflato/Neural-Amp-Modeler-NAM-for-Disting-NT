// Host test for the NAM disting NT plugin.
// Compiles the real plugin source against a mocked NT API and a fake SD card
// (a local folder standing in for /nam).
//
//   host_test parse  <file.nam> <ref_weights.f32>        streaming parser vs Python json
//   host_test render <namdir> <model index> <in.f32> <out.f32> [block]
//        renders mono float32 (volts) through the plugin at NT_SR (env, default 48000)
//        with calibration/normalisation off, unity gains, 100% wet

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fnmatch.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

#include "../nam_nt.cpp"

// ---------------------------------------------------------------- mocks

static _NT_globals makeGlobals()
{
	_NT_globals g;
	memset( &g, 0, sizeof g );
	const char* sr = getenv( "NT_SR" );
	g.sampleRate = sr ? atoi( sr ) : 48000;
	g.maxFramesPerStep = 128;
	return g;
}
extern "C" const _NT_globals NT_globals = makeGlobals();

static std::string g_namDir;
static std::string g_cwd;
static std::vector<std::string> g_found;
static FILE* g_files[ kNT_numAvailableFileHandles ];
static char g_scratch[ 4096 ];
static std::vector< std::pair< void(*)(void*), void* > > g_fsQueue;
static int16_t g_v[ kNumParams ];
static _NT_algorithm* g_alg;
static const _NT_factory* g_factory;
static int g_setParamCalls;

extern "C" {
uint32_t NT_getCpuCycleCount(void) { return 0; }
int32_t NT_algorithmIndex( const _NT_algorithm* ) { return 0; }
uint32_t NT_parameterOffset(void) { return 0; }
bool NT_isSdCardMounted(void) { return true; }
void NT_updateParameterDefinition( uint32_t, uint32_t ) {}
void NT_setParameterFromAudio( uint32_t, uint32_t parameter, int16_t value )
{
	++g_setParamCalls;
	g_v[ parameter ] = value;
	g_factory->parameterChanged( g_alg, parameter );
}
bool NT_callFileSystemCode( void (*cb)( void* ), void* data ) { g_fsQueue.push_back( std::make_pair( cb, data ) ); return true; }
_NT_fileResult NT_chdir( const char* path )
{
	if ( strcmp( path, "/nam" ) ) return kNT_noPath;
	g_cwd = g_namDir;
	return kNT_fileOK;
}
uint32_t NT_findFiles( const char* pattern, _NT_findFlags )
{
	g_found.clear();
	DIR* d = opendir( g_cwd.c_str() );
	if ( !d ) return 0;
	while ( dirent* e = readdir( d ) )
		if ( e->d_type == DT_REG && !fnmatch( pattern, e->d_name, 0 ) )
			g_found.push_back( e->d_name );
	closedir( d );
	return g_found.size();
}
void NT_getFoundFile( uint32_t i, char* buf, size_t size ) { snprintf( buf, size, "%s", g_found[i].c_str() ); }
_NT_fileResult NT_fileInfo( const char* path, _NT_fileInfo& info )
{
	std::string f = g_cwd + "/" + path;
	FILE* fp = fopen( f.c_str(), "rb" );
	if ( !fp ) return kNT_noFile;
	fseek( fp, 0, SEEK_END );
	info.size = ftell( fp );
	fclose( fp );
	snprintf( info.name, sizeof info.name, "%s", path );
	return kNT_fileOK;
}
_NT_fileResult NT_fileOpen( _NT_fileHandle F, const char* name, int )
{
	std::string f = g_cwd + "/" + name;
	g_files[F] = fopen( f.c_str(), "rb" );
	return g_files[F] ? kNT_fileOK : kNT_noFile;
}
_NT_fileResult NT_fileClose( _NT_fileHandle F ) { if ( g_files[F] ) fclose( g_files[F] ); g_files[F] = NULL; return kNT_fileOK; }
_NT_fileResult NT_fileRead( _NT_fileHandle F, void* dst, uint32_t bytes, uint32_t& got )
{
	got = fread( dst, 1, bytes, g_files[F] );
	return kNT_fileOK;
}
void* NT_getFileScratchBuffer( size_t& size ) { size = sizeof g_scratch; return g_scratch; }
void NT_drawText( int, int, const char*, int, _NT_textAlignment, _NT_textSize ) {}
void NT_drawShapeI( _NT_shape, int, int, int, int, int ) {}
int NT_floatToString( char* b, float v, int dp ) { return sprintf( b, "%.*f", dp, v ); }
}
void _NT_jsonStream::addMemberName( const char* ) {}
void _NT_jsonStream::addString( const char* ) {}
bool _NT_jsonParse::numberOfObjectMembers( int& n ) { n = 0; return true; }
bool _NT_jsonParse::matchName( const char* ) { return false; }
bool _NT_jsonParse::string( const char*& ) { return false; }
bool _NT_jsonParse::skipMember(void) { return true; }

// ---------------------------------------------------------------- helpers

static std::vector<float> readF32( const char* path )
{
	std::vector<float> v;
	FILE* f = fopen( path, "rb" );
	if ( !f ) { fprintf( stderr, "can't open %s\n", path ); exit( 1 ); }
	float x;
	while ( fread( &x, 4, 1, f ) == 1 ) v.push_back( x );
	fclose( f );
	return v;
}

static void runFs() { auto q = g_fsQueue; g_fsQueue.clear(); for ( auto& e : q ) e.first( e.second ); }

static int doParse( const char* nam, const char* ref )
{
	std::vector<float> r = readF32( ref );
	FILE* f = fopen( nam, "rb" );
	std::vector<char> data;
	int c;
	while ( ( c = fgetc( f ) ) != EOF ) data.push_back( (char)c );
	fclose( f );

	static float w[ NAMJ_LITE_WEIGHTS ], tmp[ NAMJ_LITE_WEIGHTS ];
	static NamJson j;
	int fails = 0;
	srand( 1 );
	for ( int trial = 0; trial < 6; ++trial )
	{
		j.begin( w, tmp );
		size_t pos = 0;
		while ( pos < data.size() )
		{
			size_t n = trial == 0 ? 1 : ( trial == 1 ? data.size() : 1 + rand() % ( trial * 1500 ) );
			if ( n > data.size() - pos ) n = data.size() - pos;
			j.feed( &data[pos], n );
			pos += n;
		}
		int mism = 0;
		for ( int i = 0; i < NAMJ_LITE_WEIGHTS && i < (int)r.size(); ++i )
			if ( w[i] != r[i] ) ++mism;
		bool ok = j.isA2Lite() && (int)r.size() == NAMJ_LITE_WEIGHTS && mism == 0;
		printf( "  chunking %d: complete=%d found=%d wavenet=%d leaky=%d  bit-exact mismatches=%d  loud=%.4f in_dbu=%.2f sr=%.0f  %s\n",
			trial, j.complete(), j.found, j.sawWaveNet, j.sawLeakyReLU, mism, j.loudness, j.inputLevel, j.sampleRate, ok ? "ok" : "FAIL" );
		if ( !ok ) ++fails;
	}
	return fails ? 1 : 0;
}

static int doRender( const char* dir, int model, int spec, int stereoMode, const char* inPath, const char* inRPath,
		const char* outPath, const char* outRPath, int block )
{
	g_namDir = dir;
	uintptr_t pe = pluginEntry( kNT_selector_factoryInfo, 0 );
	g_factory = (const _NT_factory*)pe;
	int32_t specs[1] = { spec };

	_NT_algorithmRequirements req;
	memset( &req, 0, sizeof req );
	g_factory->calculateRequirements( req, specs );
	printf( "  requirements: sram %u  dram %u  dtc %u\n", req.sram, req.dram, req.dtc );
	_NT_algorithmMemoryPtrs ptrs;
	ptrs.sram = (uint8_t*)aligned_alloc( 64, ( req.sram + 63 ) & ~63 );
	ptrs.dram = (uint8_t*)aligned_alloc( 64, ( req.dram + 63 ) & ~63 );
	ptrs.dtc = (uint8_t*)aligned_alloc( 64, ( req.dtc + 63 ) & ~63 );
	ptrs.itc = NULL;
	g_alg = g_factory->construct( ptrs, req, specs );
	g_alg->v = g_v;
	for ( int i = 0; i < kNumParams; ++i ) g_v[i] = parameters[i].def;
	g_v[ kParamCalibrate ] = 0;
	g_v[ kParamNormalize ] = 0;
	g_v[ kParamModel ] = model;
	g_v[ kParamStereoMode ] = stereoMode;
	g_v[ kParamInputR ] = strcmp( inRPath, "-" ) ? 2 : 0;
	g_v[ kParamOutputR ] = strcmp( outRPath, "-" ) ? 14 : 0;
	for ( int i = 0; i < kNumParams; ++i ) g_factory->parameterChanged( g_alg, i );

	std::vector<float> in = readF32( inPath );
	std::vector<float> inR = strcmp( inRPath, "-" ) ? readF32( inRPath ) : std::vector<float>();
	std::vector<float> out( in.size() ), outR( in.size() );
	std::vector<float> bus( 64 * block );

	_namAlgorithm* p = (_namAlgorithm*)g_alg;
	// let it scan and load: a few silent blocks with the fs task running in between
	for ( int i = 0; i < 20; ++i )
	{
		std::fill( bus.begin(), bus.end(), 0.0f );
		g_factory->step( g_alg, bus.data(), block / 4 );
		runFs();
	}
	printf( "  files: %d  status: %d  model: %s  err: %s\n", p->numFiles, p->status, p->modelName, p->errorText ? p->errorText : "-" );
	for ( int i = 0; i < p->numFiles; ++i ) printf( "    [%d] %s\n", i, p->cold->files[i] );
	if ( p->status != kStatusReady ) return 1;
	// run until the fade-in finishes so the comparison starts clean
	p->fade = 1.0f;

	for ( size_t pos = 0; pos + block <= in.size(); pos += block )
	{
		std::fill( bus.begin(), bus.end(), 0.0f );
		memcpy( &bus[0], &in[pos], block * 4 );
		if ( !inR.empty() ) memcpy( &bus[ block ], &inR[pos], block * 4 );	// input bus 2
		g_factory->step( g_alg, bus.data(), block / 4 );
		memcpy( &out[pos], &bus[ 12 * block ], block * 4 );	// output bus 13
		memcpy( &outR[pos], &bus[ 13 * block ], block * 4 );	// output bus 14
	}
	FILE* f = fopen( outPath, "wb" );
	fwrite( out.data(), 4, out.size(), f );
	fclose( f );
	if ( strcmp( outRPath, "-" ) )
	{
		f = fopen( outRPath, "wb" );
		fwrite( outR.data(), 4, outR.size(), f );
		fclose( f );
	}
	g_factory->draw( g_alg );
	return 0;
}

static int doMisc( const char* dir )
{
	int fails = 0;
	// fast maths
	float worstE = 0, worstL = 0;
	for ( float db = -60; db <= 40; db += 0.37f )
	{
		float g = dbToGain( db ), r = powf( 10.0f, db / 20.0f );
		worstE = std::max( worstE, fabsf( g / r - 1.0f ) );
		float l = 6.0206f * fastLog2( r );
		worstL = std::max( worstL, fabsf( l - db ) );
	}
	printf( "  dbToGain max rel err %.2e, meter dB max err %.3f dB  %s\n", worstE, worstL, worstE < 1e-4 && worstL < 0.1 ? "ok" : "FAIL" );
	fails += !( worstE < 1e-4 && worstL < 0.1 );

	// set up an instance on the mixed folder
	g_namDir = dir;
	g_factory = (const _NT_factory*)pluginEntry( kNT_selector_factoryInfo, 0 );
	_NT_algorithmRequirements req; memset( &req, 0, sizeof req );
	int32_t specs[1] = { 1 };
	g_factory->calculateRequirements( req, specs );
	_NT_algorithmMemoryPtrs ptrs;
	ptrs.sram = (uint8_t*)aligned_alloc( 64, ( req.sram + 63 ) & ~63 );
	ptrs.dram = (uint8_t*)aligned_alloc( 64, ( req.dram + 63 ) & ~63 );
	ptrs.dtc = (uint8_t*)aligned_alloc( 64, ( req.dtc + 63 ) & ~63 );
	g_alg = g_factory->construct( ptrs, req, specs );
	g_alg->v = g_v;
	for ( int i = 0; i < kNumParams; ++i ) g_v[i] = parameters[i].def;
	for ( int i = 0; i < kNumParams; ++i ) g_factory->parameterChanged( g_alg, i );
	_namAlgorithm* p = (_namAlgorithm*)g_alg;

	// preset recall by name, before the scan has happened
	copyName( p->wantedName, "z_lite.namb" );

	const int block = 24;
	std::vector<float> bus( 64 * block );
	float phase = 0, peak = 0; bool bad = false;
	auto run = [&]( int blocks ) {
		for ( int b = 0; b < blocks; ++b ) {
			std::fill( bus.begin(), bus.end(), 0.0f );
			for ( int i = 0; i < block; ++i ) { bus[i] = 3.0f * sinf( phase ); phase += 2 * 3.14159265f * 220.0f / NT_globals.sampleRate; }
			g_factory->step( g_alg, bus.data(), block / 4 );
			for ( int i = 0; i < block; ++i ) { float o = bus[ 12 * block + i ]; if ( o != o ) bad = true; peak = std::max( peak, fabsf( o ) ); }
			runFs();
		}
	};
	run( 200 );
	auto idx = [&]( const char* n ) { for ( int i = 0; i < p->numFiles; ++i ) if ( !strcmp( p->cold->files[i], n ) ) return i; return -1; };
	bool ok = p->status == kStatusReady && !strcmp( p->modelName, "z_lite.namb" ) && g_v[ kParamModel ] == idx( "z_lite.namb" );
	printf( "  preset recall by name: model=%s index=%d  %s\n", p->modelName, g_v[ kParamModel ], ok ? "ok" : "FAIL" );
	fails += !ok;

	// default gain staging on the A2 capture: calibrated input, normalised output
	g_v[ kParamModel ] = idx( "Example A2 capture.nam" ); g_factory->parameterChanged( g_alg, kParamModel );
	peak = 0; run( 400 );
	float expIn = powf( 10.0f, ( 13.2f - 14.76f ) / 20.0f ) / 5.0f;
	float expOut = powf( 10.0f, ( -18.0f + 26.3839f ) / 20.0f ) * 5.0f;
	ok = p->status == kStatusReady && fabsf( p->inGainTarget / expIn - 1 ) < 1e-3 && fabsf( p->outGainTarget / expOut - 1 ) < 1e-3 && !bad;
	printf( "  switch while running -> %s, in gain %.4f (exp %.4f), out gain %.3f (exp %.3f), out peak %.2f V, NaN %d  %s\n",
		p->modelName, p->inGainTarget, expIn, p->outGainTarget, expOut, peak, bad, ok ? "ok" : "FAIL" );
	fails += !ok;

	// bad model while running: stays muted, reports, recovers on next good one
	g_v[ kParamModel ] = idx( "old_a1.nam" ); g_factory->parameterChanged( g_alg, kParamModel );
	peak = 0; run( 100 );
	float mutedPeak = 0; { float pk = peak; peak = 0; run( 50 ); mutedPeak = peak; (void)pk; }
	ok = p->status == kStatusError && mutedPeak == 0.0f;
	printf( "  bad file: status=%d err=%s, output peak while muted %.3g  %s\n", p->status, p->errorText, mutedPeak, ok ? "ok" : "FAIL" );
	fails += !ok;
	g_v[ kParamModel ] = idx( "z_lite.namb" ); g_factory->parameterChanged( g_alg, kParamModel );
	peak = 0; run( 400 );
	ok = p->status == kStatusReady && peak > 0.01f && !bad;
	printf( "  recover: %s peak %.2f V  %s\n", p->modelName, peak, ok ? "ok" : "FAIL" );
	fails += !ok;

	// gate fully closes on silence-ish input, mix 0 passes dry
	g_v[ kParamMix ] = 0; g_factory->parameterChanged( g_alg, kParamMix );
	std::fill( bus.begin(), bus.end(), 0.0f );
	for ( int i = 0; i < block; ++i ) bus[i] = 1.234f;
	for ( int k = 0; k < 20; ++k ) { for ( int i = 0; i < block; ++i ) bus[i] = 1.234f; for ( int i = 0; i < block; ++i ) bus[12*block+i] = 0; g_factory->step( g_alg, bus.data(), block / 4 ); }
	float dry = bus[ 12 * block + 5 ];
	ok = fabsf( dry - 1.234f ) < 1e-6f;
	printf( "  mix 0 = dry: %.4f  %s\n", dry, ok ? "ok" : "FAIL" );
	fails += !ok;
	g_factory->draw( g_alg );
	return fails ? 1 : 0;
}

int main( int argc, char** argv )
{
	if ( argc >= 3 && !strcmp( argv[1], "misc" ) )
		return doMisc( argv[2] );
	if ( argc >= 4 && !strcmp( argv[1], "parse" ) )
		return doParse( argv[2], argv[3] );
	// render <dir> <model> <spec 1|2> <stereo mode 0 dual|1 summed> <inL> <inR|-> <outL> <outR|-> [block]
	if ( argc >= 10 && !strcmp( argv[1], "render" ) )
		return doRender( argv[2], atoi( argv[3] ), atoi( argv[4] ), atoi( argv[5] ), argv[6], argv[7], argv[8], argv[9],
			argc > 10 ? atoi( argv[10] ) : 24 );
	fprintf( stderr, "usage: see source\n" );
	return 2;
}
